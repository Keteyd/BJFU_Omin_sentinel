import json
import unittest
from pathlib import Path

import numpy as np

from validate_speed_reference_s3_offline import DEFAULT_PLAN, data_gate


class FrozenS3ValidatorTests(unittest.TestCase):
    def setUp(self):
        self.plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        count = 8994
        missing = {700, 1700, 2700, 3700, 4700, 5700, 6700}
        index = np.asarray([i for i in range(9001) if i not in missing])
        self.values = np.zeros(count, dtype=[('trace_us', 'u8'), ('tick_ms', 'u8')])
        self.values['trace_us'] = 500+index*4000
        self.values['tick_ms'] = 1000+index*4
        axis = {
            'interval_counter_sums': {'failed': 0, 'errors': 0, 'aborted': 0},
            'counter_saturated': False,
            'incomplete_coverage_intervals_after_first': 0,
        }
        self.report = {
            'download_complete': True, 'error': None,
            'quality_issues': [],
            'diagnostic_quality_issues': ['sample_count_not_9001'],
            'crc_errors': 0, 'last_status': {'phase': 5, 'reason': 0},
            'metadata': {'count': count},
            'initial_metadata': {'start_ms': 1000},
            'firmware_stream_quality_latched': False,
            'feedback_failure': None, 'in_stream_discarded_wire_bytes': 0,
            'profile': {'build': 0x59490801, 'profile': 5, 'reverse': 2,
                        'samples': 9001, 'period_ms': 4, 'duration_ms': 36000},
            'frozen_reference_plan_sha256':
                'BF41B63E5FA37AE20F065BB69603BF496E3E75CC5C4828D5A616F90267828B19',
            'can_summary': {'big': axis, 'small': axis},
        }

    def test_accepts_full_time_coverage_with_seven_missing_raw_slots(self):
        audit, passed = data_gate(self.report, self.values, self.plan)
        self.assertTrue(passed)
        self.assertEqual(audit['timing']['record_shortfall_diagnostic_only'], 7)
        self.assertTrue(audit['checks']['time_domain_coverage'])

    def test_rejects_can_fault(self):
        self.report['can_summary']['small'] = dict(self.report['can_summary']['small'])
        self.report['can_summary']['small']['interval_counter_sums'] = {
            'failed': 0, 'errors': 1, 'aborted': 0}
        audit, passed = data_gate(self.report, self.values, self.plan)
        self.assertFalse(passed)
        self.assertFalse(audit['checks']['CAN_both_axes'])

    def test_rejects_unfrozen_quality_label(self):
        self.report['diagnostic_quality_issues'] = ['discarded_wire_bytes']
        audit, passed = data_gate(self.report, self.values, self.plan)
        self.assertFalse(passed)
        self.assertFalse(audit['checks']['only_permitted_generic_quality_labels'])


if __name__ == '__main__':
    unittest.main()
