import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

from identify_offline import (ARX, DT, TRAIN_END, TUNE_END, Trial, analyze,
                              check_pair, evaluate, fit, load_trial, select)
from yaw_identification_protocol import BUILD, META, RECORD, VALUES, Decoder
from yaw_capture_protocol import frame


def synthetic(seed=5, axis=1):
    rng = np.random.default_rng(seed)
    u = rng.normal(size=(1000, 2))
    y = np.zeros((1000, 3))
    a = np.array([[.85, .03, 0], [.01, .8, .02], [.02, .01, .7]])
    b = np.array([[.1, .02], [-.03, .08], [.06, .07]])
    for k in range(2, len(y)):
        y[k] = a @ y[k-1] + b @ u[k-2]
    metadata = dict(id=axis, axis=axis, build=BUILD, configuration=dict(
        amplitude_deg=axis, big_angle_kp=1., small_joint_start_deg=0.,
        imu_roll_start_deg=0., imu_pitch_start_deg=0., pitch_target_rad=0.))
    return Trial(str(seed), y, u, metadata, str(seed))


def packet_group(command, sequence, data):
    return b''.join(frame(command, struct.pack('<HBB', sequence, part, 1)+data[part*8:part*8+8])
                    for part in range(len(data)//8))


def make_capture(directory, gap=False):
    values = dict.fromkeys(VALUES, 0.)
    values.update(big_effort_limit=30., small_effort_limit=6., amplitude_deg=1.)
    meta = META.pack(1, 100, BUILD, 1001, 4, 5, 0, 1, 1, 1, *[values[k] for k in VALUES])
    raw = packet_group(0x39, 0, meta)
    phases = [2]*125+[3]*750+[4]*125+[5]
    for i, phase in enumerate(phases):
        row = RECORD.pack(100+i*4+(1 if gap and i == 500 else 0),
                          0., 0., 0., 0., 0., 0., 0, 0, 0, 0, 0, 0, 0,
                          29 if i < 1000 else 5, 0, 0, 0, phase)
        raw += packet_group(0x3a, i+1, row)
    d = Decoder(1); d.feed(raw)
    (directory/'raw.bin').write_bytes(raw)
    (directory/'report.json').write_text(json.dumps(dict(trial_id=1, error=None, metadata=d.metadata)), encoding='utf-8')


class IdentificationTests(unittest.TestCase):
    def test_known_mimo_delay_and_free_run(self):
        t = synthetic()
        model = fit([t], 1, 1, 2, 0.)
        predicted = model.rollout(t, 750, 250)
        np.testing.assert_allclose(predicted, t.y[750:], atol=1e-10)
        self.assertLess(model.radius(), 1)
        self.assertFalse(model.artifact()['hardware_takeover_allowed'])

    def test_future_outputs_cannot_enter_rollout(self):
        t = synthetic(); model = fit([t], 2, 3, 1, 1e-6)
        expected = model.rollout(t, 750, 100)
        changed = copy.deepcopy(t); changed.y[750:] = 1e9
        np.testing.assert_array_equal(model.rollout(changed, 750, 100), expected)

    def test_future_training_data_and_scalers_do_not_leak(self):
        t = synthetic(); model = fit([t], 2, 2, 2, .001)
        changed = copy.deepcopy(t)
        changed.y[TRAIN_END:] = 1e6; changed.u[TRAIN_END:] = -1e9
        other = fit([changed], 2, 2, 2, .001)
        np.testing.assert_array_equal(model.coefficient, other.coefficient)
        np.testing.assert_array_equal(model.output_scale, other.output_scale)
        np.testing.assert_array_equal(model.intercept, other.intercept)

    def test_selection_does_not_use_test_segment(self):
        trials = [synthetic(5, 1), synthetic(6, 2)]
        first, entries = select(trials)
        changed = copy.deepcopy(trials)
        for t in changed:
            t.y[TUNE_END:] = 10000.; t.u[TUNE_END:] = -10000.
        second, other_entries = select(changed)
        self.assertEqual(entries, other_entries)
        np.testing.assert_array_equal(first.coefficient, second.coefficient)

    def test_pair_configuration_pose_duplicate_and_excitation_checks(self):
        a, b = synthetic(5, 1), synthetic(6, 2)
        check_pair([a, b])
        for field, value in [('big_angle_kp', 2.), ('imu_roll_start_deg', 1.)]:
            changed = copy.deepcopy(b); changed.metadata['configuration'][field] = value
            with self.assertRaises(ValueError): check_pair([a, changed])
        with self.assertRaises(ValueError): check_pair([a, a])
        a.u[:] = 0; b.u[:] = 0
        with self.assertRaises(ValueError): check_pair([a, b])

    def test_distinct_trials_do_not_share_training_history(self):
        a, b = synthetic(5, 1), synthetic(6, 2)
        model = fit([a, b], 2, 2, 1, .001)
        reverse = fit([b, a], 2, 2, 1, .001)
        np.testing.assert_allclose(model.coefficient, reverse.coefficient, atol=1e-10)

    def test_raw_loader_phase_validation_and_terminal_removal(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp); make_capture(p)
            trial = load_trial(p)
            self.assertEqual(trial.y.shape, (1000, 3))
            self.assertEqual(trial.u.shape, (1000, 2))
            np.testing.assert_array_equal(trial.y, 0)
            self.assertEqual(len(trial.sha256), 64)
            report = json.loads((p/'report.json').read_text())
            report['metadata']['axis'] = 2
            (p/'report.json').write_text(json.dumps(report))
            with self.assertRaises(ValueError): load_trial(p)
            make_capture(p, gap=True)
            with self.assertRaises(ValueError): load_trial(p)

    def test_bad_numbers_delays_and_evaluation_windows(self):
        t = synthetic()
        with self.assertRaises(ValueError): fit([t], 2, 1, 0, .1)
        model = fit([t], 2, 2, 1, .001)
        with self.assertRaises(ValueError): model.rollout(t, 0, 2)
        with self.assertRaises(ValueError): evaluate(model, t, 750, 760, 50)
        t.y[100, 0] = np.nan
        with self.assertRaises(ValueError): fit([t], 2, 2, 1, .001)

    def test_synthetic_success_never_authorizes_hardware(self):
        report, curves = analyze([synthetic(5, 1), synthetic(6, 2)])
        self.assertEqual(report['model_status'], 'experimental_unvalidated')
        self.assertFalse(report['hardware_takeover_allowed'])
        self.assertIn('no_independent_repeated_trial_validation', report['blockers'])
        self.assertEqual(len(curves), 2)
        self.assertEqual(report['split_ms']['test'], [3000, 4000])


if __name__ == '__main__':
    unittest.main()
