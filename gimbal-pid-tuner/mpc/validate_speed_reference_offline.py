"""Apply the frozen S1 speed model and gates once to S2; never refits coefficients."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from identify_speed_reference_offline import ACTIVE, OUTPUTS, evaluate, load_capture, sha256


def apply_gate(metrics, gate):
    primary = metrics[str(gate['primary_horizon_ms'])]
    improvement = primary['improvement_over_hold_fraction']
    rmse = primary['rmse_dps']
    checks = {}
    for output in OUTPUTS:
        checks[output+'_improvement'] = bool(
            improvement[output] >= gate['minimum_improvement_over_hold_fraction'][output])
        checks[output+'_rmse'] = bool(rmse[output] <= gate['maximum_rmse_dps'][output])
    return checks, bool(all(checks.values()))


def validate(model_path, capture):
    model_path = Path(model_path)
    frozen = json.loads(model_path.read_text(encoding='utf-8'))
    if frozen.get('status') != 'S1_SPEED_REFERENCE_MODEL_FROZEN_PENDING_S2':
        raise ValueError('model file is not the frozen pre-S2 artifact')
    data = load_capture(capture, phase_set=1)
    item = frozen['frozen_model']
    definition = item['structure']
    structure = (definition['output_lags'], definition['reference_lags'],
                 definition['reference_delay_samples'], definition['ridge'])
    coefficient = np.asarray(
        item['normalized_coefficient_feature_rows_by_output_columns'], dtype=float)
    scales = {
        'output_baseline': np.asarray(list(item['output_baseline_dps'].values())),
        'output_scale': np.asarray(list(item['output_scale_dps'].values())),
        'input_scale': np.asarray(list(item['input_scale_dps'].values())),
    }
    metrics = evaluate(data, coefficient, structure, scales, ACTIVE)
    checks, passed = apply_gate(metrics, frozen['frozen_S2_acceptance_gate'])
    return {
        'status': ('FROZEN_SPEED_REFERENCE_MODEL_PASSED_S2'
                   if passed else 'FROZEN_SPEED_REFERENCE_MODEL_REJECTED_BY_S2'),
        'hardware_takeover_allowed': False,
        'frozen_model_report': str(model_path.resolve()),
        'frozen_model_report_sha256': sha256(model_path),
        'S2_source': {key: data[key] for key in ('capture', 'raw_sha256', 'samples_csv_sha256')},
        'S2_rolling_metrics': metrics,
        'gate_checks': checks,
        'all_frozen_gates_passed': passed,
        'policy': frozen['frozen_S2_acceptance_gate'],
        'interpretation': ('validation of the retained PID closed-loop speed-reference predictor; '
                           'no voltage/current-to-mechanics claim and no hardware takeover'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = validate(args.model, args.capture)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['all_frozen_gates_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
