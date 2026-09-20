"""0x59490301 compact CAN diagnostics stream. No hardware access."""
import binascii
import struct

from yaw_capture_protocol import crc8
from yaw_slow_protocol import Decoder as SlowDecoder, FIELDS

BUILD = 0x59490301
FRAME_BYTES = 163
AXIS = struct.Struct('<IIiIIIHhhhhHhHH6B')
AXIS_FIELDS = ('sequence', 'attempt_us', 'integral_raw_us', 'feedback_us', 'complete_sequence',
               'complete_us', 'known_us', 'minimum', 'maximum', 'command', 'current', 'encoder',
               'feedback_rpm', 'wait_us', 'trace_flags', 'attempts', 'queued', 'completed', 'failed', 'errors', 'aborted')
TRACE_FIELDS = list(FIELDS)+['trace_us', 'interval_us']+[
    axis+'_'+name for axis in ('big', 'small') for name in AXIS_FIELDS]+[
    axis+'_'+name for axis in ('big', 'small') for name in ('mean_attempted_command_raw', 'feedback_age_us', 'pending')]


class Decoder(SlowDecoder):
    BUILD, STREAM_VERSION, CAPACITY, RECORD_BYTES = BUILD, 3, 320, 152
    # Streaming begins after metadata has been sent.  A bounded first-sample
    # offset is normal; subsequent cadence is checked sample to sample and by
    # the independent microsecond trace interval below.
    FIRST_TICK_RANGE_MS = (0, 10)
    TICK_INTERVAL_RANGE_MS = (1, 10)

    def __init__(self, trial_id=None, expected=None):
        super().__init__(trial_id, expected)
        self.wire_buffer = bytearray()
        self.pre_sync_discarded_bytes = 0
        self.wire_synchronized = False

    def _discard_presync_byte(self):
        del self.wire_buffer[0]
        self.bytes_consumed += 1
        self.discarded_bytes += 1
        self.pre_sync_discarded_bytes += 1

    def feed(self, chunk):
        self.bytes_received += len(chunk)
        self.wire_buffer.extend(chunk)
        while len(self.wire_buffer) >= 16:
            if self.wire_buffer[0] != 255:
                del self.wire_buffer[0]
                self.bytes_consumed += 1
                self.discarded_bytes += 1
                stream_started = bool(self.wire_synchronized or self.profile is not None
                                      or self.initial is not None or self.rows
                                      or self.metadata is not None or self.groups)
                if stream_started:
                    self.issues.add('discarded_wire_bytes')
                else:
                    # Opening an active UART may start in the middle of a status
                    # reply. Bytes before the first trial fragment are framing
                    # synchronization, not loss from the captured trial.
                    self.pre_sync_discarded_bytes += 1
                continue
            size = FRAME_BYTES if self.wire_buffer[1] == 0x3f else 16
            if len(self.wire_buffer) < size:
                break
            packet = bytes(self.wire_buffer[:size])
            if size == 16:
                # A continuously transmitting UART can be opened in a payload
                # byte equal to 0xff.  Before the first valid frame, treat an
                # invalid 16-byte candidate as synchronization noise and keep
                # looking for the next header.  Once synchronized, corruption
                # remains fatal so trial data cannot be silently accepted.
                if (packet[-1] != 13 or crc8(packet[:14]) != packet[14]) and not self.wire_synchronized:
                    self._discard_presync_byte()
                    continue
                if packet[-1] != 13:
                    raise ValueError('malformed legacy control frame in trace stream')
                if packet[1] == 0x3d:
                    raise ValueError('legacy data frame in CAN trace session')
                self.bytes_received -= 16
                super().feed(packet)
                self.wire_synchronized = True
            else:
                if packet[-1] != 13 or binascii.crc_hqx(packet[:-3], 0xffff) != int.from_bytes(packet[-3:-1], 'little'):
                    self.crc_errors += 1
                    self.first_crc_error_offset = self.bytes_consumed
                    raise ValueError('CAN trace CRC16/tail mismatch; receipt stopped')
                trial, sequence = struct.unpack_from('<IH', packet, 2)
                if trial != self.trial_id or sequence != len(self.rows)+1:
                    raise ValueError('CAN trace trial/sequence mismatch; receipt stopped')
                self._trace_row(packet[8:-3])
                self.bytes_consumed += size
                self.frame_counts[0x3f] = self.frame_counts.get(0x3f, 0)+1
                self.wire_synchronized = True
            del self.wire_buffer[:size]

    def _trace_row(self, raw):
        now, duration = struct.unpack_from('<II', raw, 48)
        extra = dict(trace_us=now, interval_us=duration)
        if not self.rows and duration != now:
            raise ValueError('CAN trace first interval must start at epoch zero')
        if self.rows and ((now-self.rows[-1]['trace_us']) & 0xffffffff) != duration:
            raise ValueError('CAN trace interval boundary mismatch')
        if (self.rows and not duration) or duration > 10000:
            self.issues.add('trace_clock_or_interval_invalid')
        for i, axis in enumerate(('big', 'small')):
            r = dict(zip(AXIS_FIELDS, AXIS.unpack_from(raw, 56+48*i)))
            flags = r['trace_flags']
            if flags & ~0x3ff or r['encoder'] > 8191 or r['known_us'] > duration:
                raise ValueError('invalid CAN trace flags/feedback/coverage')
            if r['minimum'] > r['maximum'] or not r['minimum'] <= r['command'] <= r['maximum']:
                raise ValueError('invalid CAN trace command extrema')
            if not flags & 4 and r['queued']+r['failed'] != r['attempts']:
                raise ValueError('CAN trace enqueue accounting mismatch')
            previous_pending = self.rows[-1][axis+'_pending'] if self.rows else 0
            if not flags & (4 | 64) and previous_pending+r['queued']-r['completed']-r['errors']-r['aborted'] != flags >> 8:
                raise ValueError('CAN trace mailbox accounting mismatch')
            if r['complete_sequence'] > r['sequence'] or r['complete_us'] > now or r['feedback_us'] > now or r['attempt_us'] > now:
                raise ValueError('CAN trace event lies outside capture history')
            if self.rows and not flags & 4 and ((r['sequence']-self.rows[-1][axis+'_sequence']) & 0xffffffff) != r['attempts']:
                raise ValueError('CAN trace attempt sequence gap')
            if flags & 0xfc or r['failed'] or r['errors'] or r['aborted']:
                self.issues.add(axis+'_CAN_status_or_counter_fault')
            if self.rows and (r['known_us'] != duration or not flags & 1):
                self.issues.add(axis+'_incomplete_command_coverage')
            if self.rows and not flags & 2:
                self.issues.add(axis+'_no_feedback_in_capture')
            if (not flags & 8 and
                    not r['minimum']*r['known_us'] <= r['integral_raw_us'] <= r['maximum']*r['known_us']):
                raise ValueError('CAN trace integral outside command extrema')
            if self.profile and self.profile['profile'] == 2 and any(r[k] for k in ('minimum', 'maximum', 'command', 'integral_raw_us')):
                raise ValueError('nonzero attempted CAN command in zero-output bench')
            extra.update({axis+'_'+k: v for k, v in r.items()})
            extra[axis+'_mean_attempted_command_raw'] = r['integral_raw_us']/r['known_us'] if r['known_us'] else None
            extra[axis+'_feedback_age_us'] = (now-r['feedback_us']) & 0xffffffff if flags & 2 else None
            extra[axis+'_pending'] = flags >> 8
        # The firmware schedules phases from the microsecond trace epoch.  Its
        # millisecond tick can reach a knot up to 999 us earlier by rounding,
        # so use the independent trace clock for boundary validation.
        super()._row(raw[:48], profile_elapsed_ms=now/1000.0)
        self.rows[-1].update(extra)

    @property
    def complete(self):
        if not super().complete:
            return False
        if not self.wire_buffer:
            return True
        # KEEPALIVE/RECEIPT status replies are independent of the already
        # complete trial stream. A serial read may end after only ff or after
        # the ff/37 prefix of the next status response.
        # Keep incomplete trace-data prefixes strict.
        return (len(self.wire_buffer) < 16 and
                (self.wire_buffer == b'\xff' or self.wire_buffer[:2] == b'\xff\x37'))

    def report(self):
        report = super().report()
        summary = {}
        for axis in ('big', 'small'):
            ages = [r[axis+'_feedback_age_us'] for r in self.rows if r[axis+'_feedback_age_us'] is not None]
            summary[axis] = {
                'interval_counter_sums': {name: sum(r[axis+'_'+name] for r in self.rows)
                                          for name in ('attempts', 'queued', 'completed', 'failed', 'errors', 'aborted')},
                'counter_saturated': any(r[axis+'_trace_flags'] & 4 for r in self.rows),
                'max_feedback_age_us': max(ages, default=None),
                'max_pending': max((r[axis+'_pending'] for r in self.rows), default=0),
                'terminal_pending': self.rows[-1][axis+'_pending'] if self.rows else None,
                'incomplete_coverage_intervals_after_first': sum(
                    r[axis+'_known_us'] != r['interval_us'] or not r[axis+'_trace_flags'] & 1
                    for r in self.rows[1:]),
            }
        trailing_status = (len(self.wire_buffer) < 16 and
                           (self.wire_buffer == b'\xff' or
                            self.wire_buffer[:2] == b'\xff\x37'))
        report.update(trace_version=3, buffered_wire_bytes=len(self.wire_buffer),
                      buffered_wire_is_partial_status_after_complete=bool(trailing_status),
                      pre_sync_discarded_wire_bytes=self.pre_sync_discarded_bytes,
                      in_stream_discarded_wire_bytes=(self.discarded_bytes-
                                                      self.pre_sync_discarded_bytes),
                      can_summary=summary,
                      trace_semantics='attempted raw command integral; completion timestamps observe TSR flags; current is raw feedback, not torque')
        return report
