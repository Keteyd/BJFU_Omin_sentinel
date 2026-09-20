"""Historical 0x59490404 trace-v4 decoder with the trace-clock phase fix."""

import struct

from yaw_can_trace_protocol import AXIS, AXIS_FIELDS, Decoder as TraceV3Decoder
from yaw_identification_protocol import FIELDS as V3_FIELDS, META, RECORD
from yaw_slow_protocol import KNOTS as SLOW_KNOTS


BUILD = 0x59490404
STREAM_VERSION = 4
DUAL_A, DUAL_B = 10, 11
DUAL_KNOTS = (0, 2000, 16000, 20000, 0, 0, 0, 0)
FIELDS = list(V3_FIELDS)
FIELDS[9:11] = ['big_reference_offset_cdeg', 'small_heading_reference_offset_cdeg']
TRACE_FIELDS = FIELDS+['trace_us', 'interval_us']+[
    axis+'_'+name for axis in ('big', 'small') for name in AXIS_FIELDS]+[
    axis+'_'+name for axis in ('big', 'small') for name in
    ('mean_attempted_command_raw', 'feedback_age_us', 'pending')]
REASONS = ('ok', 'operator_stop', 'link_lost', 'feedback_bad', 'travel', 'timing',
           'arm_expired', 'capture_not_ready', 'config_changed', 'control_failed',
           'stream_stalled', 'buffer_full', 'yaw_control_inactive',
           'output_nonfinite', 'pitch_offline')
BIG_STOP_REASONS = ('none', 'allow_dropped', 'tune_invalid', 'effort_limit_invalid',
                    'feedback_guard', 'small_interlock', 'small_limits_invalid',
                    'direction_invalid', 'coordinator_invalid', 'angle_pid_invalid',
                    'speed_reference_invalid', 'speed_pid_invalid', 'effort_output_invalid')


class Decoder(TraceV3Decoder):
    BUILD = BUILD
    STREAM_VERSION = STREAM_VERSION
    FIELDS = FIELDS

    def _metadata(self, seq, raw):
        # The parent accepts only the old single-axis metadata. Validate the
        # common immutable fields here, including the new fixed dual request.
        if self.profile is None:
            raise ValueError('metadata arrived before profile')
        v = META.unpack(raw)
        m = dict(zip(('id', 'start_ms', 'build', 'count', 'period_ms', 'phase',
                      'reason', 'axis', 'version', 'setup'), v[:10]))
        from yaw_identification_protocol import VALUES
        m['configuration'] = dict(zip(VALUES, v[10:]))
        c = m['configuration']
        import math
        if (m['id'] != self.trial_id or m['build'] != self.BUILD or m['period_ms'] != 4
                or m['version'] != self.STREAM_VERSION or m['setup'] != 1 or m['count'] > 5001
                or not all(math.isfinite(x) for x in v[10:])):
            raise ValueError('invalid metadata')
        profile = self.profile['profile']
        valid_request = ((profile == 2 and m['axis'] == 0 and c['amplitude_deg'] == 0)
                         or (profile == 1 and m['axis'] in (1, 2) and
                             0 < c['amplitude_deg'] <= (15 if m['axis'] == 1 else 10))
                         or (profile == 3 and m['axis'] == 3 and c['amplitude_deg'] == 0))
        if not valid_request:
            raise ValueError('metadata axis/amplitude mismatch')
        if self.expected and (m['axis'] != self.expected[2]
                              or abs(c['amplitude_deg']-self.expected[3]) > 1e-4):
            raise ValueError('firmware request differs from expected target')
        if not 0 < c['big_effort_limit'] <= 30 or not 0 < c['small_effort_limit'] <= 6:
            raise ValueError('unexpected output limits')
        if seq == 0 and (m['count'] or m['phase'] != 2 or m['reason']):
            raise ValueError('invalid initial metadata')
        if seq == 1 and (m['phase'] not in (5, 6) or m['count'] < len(self.rows)):
            raise ValueError('invalid terminal metadata')
        frozen = lambda item: {k: x for k, x in item.items() if k not in ('count', 'phase', 'reason')}
        if self.initial and frozen(self.initial) != frozen(m):
            raise ValueError('configuration or anchors changed')
        if self.initial is None:
            self.initial = m
        if seq == 1:
            if self.metadata is not None and self.metadata != m:
                raise ValueError('terminal metadata changed')
            self.metadata = m
            self.groups.pop((0x39, 0), None)

    def _profile(self, raw):
        from yaw_slow_protocol import PROFILE
        v = PROFILE.unpack(raw)
        p = dict(zip(('id', 'build', 'baud', 'samples', 'period_ms', 'capacity', 'duration_ms'), v[:7]))
        p.update(knots_ms=list(v[7:15]), big_peak=v[15], small_peak=v[16], big_travel=v[17],
                 small_travel=v[18], heading_travel=v[19], center_deg=v[20],
                 profile=v[21], reverse=v[22], version=v[23], reserved=v[24])
        common = (tuple(v[:7]) == (self.trial_id, self.BUILD, 460800, 5001, 4, self.CAPACITY, 20000)
                  and tuple(v[17:21]) == (25., 20., 20., 10.)
                  and v[23:] == (self.STREAM_VERSION, 0))
        old = (p['profile'] in (1, 2) and tuple(v[7:15]) == SLOW_KNOTS
               and tuple(v[15:17]) == (15., 10.) and p['reverse'] in (0, 1)
               and not (p['profile'] == 2 and p['reverse']))
        dual = (p['profile'] == 3 and tuple(v[7:15]) == DUAL_KNOTS
                and tuple(v[15:17]) == (3., 2.) and p['reverse'] in (0, 1))
        if not common or not (old or dual):
            raise ValueError('unexpected profile/limits/build')
        if self.expected and (p['profile'], p['reverse']) != self.expected[:2]:
            raise ValueError('firmware started a different profile')
        if self.profile is not None and self.profile != p:
            raise ValueError('profile changed midstream')
        self.profile = p

    def _trace_row(self, raw):
        super()._trace_row(raw)
        row = self.rows[-1]
        bench = self.profile['profile'] == 2
        if (bench or row['phase'] == 5) and (row['big_reference_offset_cdeg'] or
                                             row['small_heading_reference_offset_cdeg']):
            raise ValueError('nonzero reference in bench/terminal sample')
        if row['offset_cdeg'] != row['big_reference_offset_cdeg'] + row['small_heading_reference_offset_cdeg']:
            raise ValueError('reference sum/check field mismatch')

    def report(self):
        report = super().report()
        reason = (self.metadata or self.status or {}).get('reason')
        report['trace_version'] = self.STREAM_VERSION
        report['reason_name'] = (REASONS[reason] if isinstance(reason, int) and
                                 0 <= reason < len(REASONS) else None)
        stop_code = (((self.status or {}).get('extra', 0) >> 4) & 0x0f
                     if reason == 12 else 0)
        report['big_stop_reason_code'] = stop_code
        report['big_stop_reason_name'] = (BIG_STOP_REASONS[stop_code]
                                          if stop_code < len(BIG_STOP_REASONS) else None)
        report['trace_semantics'] = ('attempted raw command integral; independent scheduled references in centidegrees; '
                                     'completion timestamps observe TSR flags; current is raw feedback, not torque')
        return report


def request(op=0, trial_id=0, axis=0, value=0):
    from yaw_slow_protocol import request as old_request
    from yaw_capture_protocol import frame
    from yaw_identification_protocol import MAGIC
    if op in (DUAL_A, DUAL_B):
        if type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff or axis != 3 or value != 0:
            raise ValueError('dual request needs nonzero ID, axis 3 and fixed amplitudes')
        return frame(0x36, struct.pack('<IHBBI', trial_id, 0, op, 3, MAGIC))
    return old_request(op, trial_id, axis, value)
