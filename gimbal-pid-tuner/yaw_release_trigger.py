"""Host-only release detection. This module cannot control an actuator."""

import math


class ReleaseTrigger:
    def __init__(self):
        self.state = "safe_baseline"
        self.previous = None
        self.safe_ms = 0
        self.active_center_ms = 0
        self.excursion_ms = 0
        self.direction = 0
        self.event = None

    def feed(self, row):
        if self.event is not None:
            return False
        if not row["valid"]:
            raise RuntimeError("invalid observation; return right switch UP")
        flags = row["flags"]
        if flags & 206 != 206 or flags & (32 | 256) or row["big_command"] != 0:
            raise RuntimeError("big-yaw isolation / small-yaw IMU mode not confirmed; switch UP")
        joint = (row["small_encoder_deg"] - 299.53125 + 180) % 360 - 180
        if not math.isfinite(joint) or not -53.677734 < joint < 26.080078:
            raise RuntimeError("small yaw too close to soft boundary; switch UP")
        delta = 0
        if self.previous is not None:
            delta = (row["tick_ms"] - self.previous["tick_ms"]) & 0xffffffff
            host_delta = row["host_s"] - self.previous["host_s"]
            if (row["segment"] != self.previous["segment"] or delta != 25 or
                    (row["sequence"] - self.previous["sequence"]) & 0xffff != 1 or
                    not 0 <= host_delta <= .3):
                raise RuntimeError("observation discontinuity; switch UP and start a new capture")
        self.previous = row.copy()
        up = bool(flags & 1)
        enabled = bool(flags & 16)
        stick = row["rc_yaw"]
        if up:
            if enabled or row["small_command"] != 0:
                raise RuntimeError("SAFE/output inconsistency; stop the test")
            if self.state not in ("safe_baseline", "ready"):
                raise RuntimeError("returned to SAFE before release trigger; test cancelled")
            self.safe_ms = self.safe_ms + delta if stick == 0 else 0
            if self.safe_ms >= 500:
                self.state = "ready"
            return False
        if self.state == "safe_baseline" or not enabled:
            raise RuntimeError("SAFE baseline missing or small output disabled; switch UP")
        if self.state == "ready":
            if stick != 0:
                raise RuntimeError("stick must remain centered when enabling; switch UP")
            self.state = "active_center"
            self.active_center_ms = 0
        elif self.state == "active_center":
            if stick != 0:
                raise RuntimeError("wait for centered enabled baseline before moving; switch UP")
            self.active_center_ms += delta
            if self.active_center_ms >= 100:
                self.state = "wait_excursion"
        elif self.state == "wait_excursion":
            if abs(stick) >= 40:
                self.direction = 1 if stick > 0 else -1
                self.excursion_ms = 0
                self.state = "excursion"
        elif self.state == "excursion":
            if stick == 0:
                if self.excursion_ms >= 100:
                    self.event = row.copy()
                    self.state = "triggered"
                    return True
                self.state = "wait_excursion"
            elif stick * self.direction < 0:
                self.state = "wait_excursion"
                self.excursion_ms = 0
            else:
                self.excursion_ms += delta
        return False
