#!/usr/bin/env python3
"""Generate the 0x59490905 field-rebalanced gain from the frozen S1 model."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from design_speed_reference_mpc import augmented_model


BUILD = 0x59490905
HORIZON = 15
HEADING_WEIGHT = 300.0
JOINT_WEIGHT = 30.0
RATE_WEIGHT = 0.02
REFERENCE_WEIGHT = np.asarray([0.16, 0.02])
REFERENCE_DELTA_WEIGHT = np.asarray([2.0, 0.5])
JOINT_ENVELOPE_DEG = np.asarray([-53.677734375, 26.080078125])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite_horizon_gain(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    n, m = a.shape[0], b.shape[1]
    q = np.zeros((n, n), dtype=float)
    q[0, 0] = HEADING_WEIGHT
    q[1, 1] = JOINT_WEIGHT
    q[2, 2] = q[3, 3] = RATE_WEIGHT
    r = np.diag(REFERENCE_WEIGHT)
    rd = np.diag(REFERENCE_DELTA_WEIGHT)
    select_previous = np.zeros((m, n), dtype=float)
    select_previous[:, 6:8] = np.eye(m)
    p = q.copy()
    for _ in range(HORIZON):
        h = r + rd + b.T @ p @ b
        cross = b.T @ p @ a - rd @ select_previous
        gain = np.linalg.solve(h, cross)
        closed = a - b @ gain
        p = (q + closed.T @ p @ closed + gain.T @ r @ gain
             + (gain + select_previous).T @ rd @ (gain + select_previous))
    return gain


def first_move(gain: np.ndarray, heading_error_deg: float) -> list[float]:
    state = np.zeros(gain.shape[1], dtype=float)
    state[0] = heading_error_deg
    state[13] = 1.0
    return [float(value) for value in -gain @ state]


def generate(model_path: Path) -> dict:
    source = json.loads(model_path.read_text(encoding="utf-8"))
    a, b, _, _ = augmented_model(source["frozen_model"])
    gain = finite_horizon_gain(a, b)
    dynamic_radius = max(abs(np.linalg.eigvals((a - b @ gain)[:12, :12])))
    return {
        "status": "OPERATOR_REQUESTED_FIELD_REBALANCE_DIAGNOSTIC",
        "build": f"0x{BUILD:08X}",
        "model_report": str(model_path.resolve()),
        "model_report_sha256": sha256(model_path),
        "model_scope": source["model_scope"],
        "controller": {
            "sample_period_ms": 20,
            "prediction_horizon_steps": HORIZON,
            "first_move_feedback_gain": gain.tolist(),
            "weights": {
                "heading_error": HEADING_WEIGHT,
                "small_joint": JOINT_WEIGHT,
                "rate_deviation_each": RATE_WEIGHT,
                "reference": REFERENCE_WEIGHT.tolist(),
                "reference_delta": REFERENCE_DELTA_WEIGHT.tolist(),
            },
            "limits": {
                "speed_reference_abs_dps": None,
                "speed_reference_delta_per_20ms_dps": None,
                "target_heading_rate_internal_clamp_dps": None,
                "reference_joint_envelope_deg": JOINT_ENVELOPE_DEG.tolist(),
                "speed_pid_effort_limits": [30.0, 6.0],
            },
            "dynamic_closed_loop_spectral_radius": float(dynamic_radius),
        },
        "zero_history_first_moves": [
            {"heading_error_deg": value,
             "reference_dps": first_move(gain, value)}
            for value in (0.5, 1.0, 5.0, 10.0, 30.0)
        ],
        "change_from_59490904": {
            "small_joint_weight": [100.0, JOINT_WEIGHT],
            "big_reference_delta_weight": [0.5, float(REFERENCE_DELTA_WEIGHT[0])],
            "small_reference_delta_weight": [0.5, float(REFERENCE_DELTA_WEIGHT[1])],
            "speed_limit_policy": "unchanged: no ordinary MPC speed or slew clamp",
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = generate(args.model)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
