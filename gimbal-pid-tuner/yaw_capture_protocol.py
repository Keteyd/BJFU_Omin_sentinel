"""Version 1 dual-yaw observation protocol; no serial or motion dependencies."""

import math
import struct

COMMAND = 0x32
VERSION = 1
PARTS = 11
RECORD = struct.Struct("<IIHHHhHH17f")
FIELDS = [
    "tick_ms", "imu_sequence", "imu_age_ms", "big_age_ms", "small_age_ms",
    "rc_yaw", "flags", "skipped", "imu_dt_s", "yaw_deg", "roll_deg",
    "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "big_encoder_deg",
    "small_encoder_deg", "big_rpm", "small_rpm", "big_command", "small_command",
    "yaw_target_deg", "big_current_raw", "small_current_raw",
    "pitch_deg", "small_speed_target_rpm",
]


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x31 if crc & 128 else 0)) & 255
    return crc


def frame(command, payload):
    if len(payload) > 12:
        raise ValueError("payload exceeds 12 bytes")
    data = bytes((255, command)) + payload.ljust(12, b"\0")
    return data + bytes((crc8(data), 13))


def heartbeat(version=1):
    # Non-locking monitor only. Never enable an axis, set a gain or send a target.
    if version not in (1, 2):
        raise ValueError("unsupported capture version")
    return frame(0x24, bytes((0xA5, 1, 0xD0 + version, version)))


class Decoder:
    def __init__(self, version=1):
        if version not in (1, 2):
            raise ValueError("unsupported capture version")
        self.version = version
        self.parts = 11 if version == 1 else 12
        self.record = RECORD if version == 1 else struct.Struct("<IIHHHhHH19f")
        self.fields = FIELDS + ([] if version == 1 else ["control_dt_s", "gimbal_dt_s"])
        self.csv_fields = CSV_FIELDS[:5] + self.fields
        self.buffer = bytearray()
        self.pending = bytearray()
        self.sequence = None
        self.next_part = 0
        self.started = 0
        self.previous_sequence = None
        self.previous_tick = None
        self.segment = 0
        self.ranges = {}
        self.diagnostics = []
        self.stats = dict(crc_errors=0, discarded_bytes=0, other_frames=0,
                          incomplete_groups=0, unexpected_parts=0,
                          unsupported_versions=0, missing_groups=0,
                          duplicate_groups=0, records=0, invalid_records=0)

    def abandon(self):
        if self.sequence is not None:
            self.stats["incomplete_groups"] += 1
        self.sequence = None
        self.pending.clear()
        self.next_part = 0

    def finish(self):
        self.abandon()

    def feed(self, chunk, host_s):
        rows = []
        if self.sequence is not None and host_s - self.started > .25:
            self.abandon()
        self.buffer.extend(chunk)
        while len(self.buffer) >= 16:
            if self.buffer[0] != 255 or self.buffer[15] != 13:
                del self.buffer[0]
                self.stats["discarded_bytes"] += 1
                continue
            packet = bytes(self.buffer[:16])
            if crc8(packet[:14]) != packet[14]:
                del self.buffer[0]
                self.stats["crc_errors"] += 1
                self.abandon()
                continue
            del self.buffer[:16]
            if packet[1] in (0x34, 0x35):
                if packet[1] == 0x34:
                    keys = ("failed", "succeeded", "failed_stage", "hal_status", "acc_id", "gyro_id")
                    values = struct.unpack_from("<IIBBBB", packet, 2)
                else:
                    keys = ("read_rejected", "dt_rejected", "last_dt_s")
                    values = struct.unpack_from("<IIf", packet, 2)
                self.diagnostics.append(dict(zip(keys, values), command=packet[1], host_s=host_s))
                continue
            if packet[1] != COMMAND:
                self.stats["other_frames"] += 1
                continue
            sequence, part, version = struct.unpack_from("<HBB", packet, 2)
            if version != self.version:
                self.stats["unsupported_versions"] += 1
                self.abandon()
                continue
            if part == 0:
                self.abandon()
                self.sequence, self.started = sequence, host_s
            if self.sequence != sequence or part != self.next_part or part >= self.parts:
                self.stats["unexpected_parts"] += 1
                self.abandon()
                continue
            self.pending.extend(packet[6:14])
            self.next_part += 1
            if self.next_part != self.parts:
                continue
            row = dict(zip(self.fields, self.record.unpack(self.pending)))
            self.sequence = None
            self.pending.clear()
            self.next_part = 0
            delta_tick = None if self.previous_tick is None else (
                row["tick_ms"] - self.previous_tick) & 0xFFFFFFFF
            if delta_tick is not None and delta_tick >= 0x80000000:
                self.segment += 1
                self.previous_sequence = None
                delta_tick = None
            if self.previous_sequence is not None:
                delta_seq = (sequence - self.previous_sequence) & 0xFFFF
                if delta_seq == 0:
                    self.stats["duplicate_groups"] += 1
                    continue
                if delta_seq >= 0x8000:
                    self.segment += 1
                    delta_tick = None
                else:
                    self.stats["missing_groups"] += delta_seq - 1
            self.previous_sequence, self.previous_tick = sequence, row["tick_ms"]
            finite = all(math.isfinite(row[k]) for k in self.fields[8:])
            fresh = (row["imu_sequence"] != 0 and row["imu_age_ms"] <= 10
                     and row["big_age_ms"] <= 50 and row["small_age_ms"] <= 50)
            valid = (finite and fresh and (row["flags"] & 14) == 14
                     and 0 < row["imu_dt_s"] <= .01
                     and 0 <= row["big_encoder_deg"] < 360
                     and 0 <= row["small_encoder_deg"] < 360)
            if self.version == 2:
                valid = valid and 0 < row["control_dt_s"] <= .02 and 0 < row["gimbal_dt_s"] <= .01
            row.update(sequence=sequence, host_s=host_s, segment=self.segment,
                       delta_ms=delta_tick, valid=int(valid))
            self.stats["records"] += 1
            self.stats["invalid_records"] += int(not valid)
            for key in self.fields:
                value = row[key]
                if math.isfinite(value):
                    low, high = self.ranges.get(key, (value, value))
                    self.ranges[key] = (min(low, value), max(high, value))
            rows.append(row)
        return rows


CSV_FIELDS = ["sequence", "host_s", "segment", "delta_ms", "valid"] + FIELDS
