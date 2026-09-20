#!/usr/bin/env python3
"""Generate the 0x59490907 third field-rebalanced gain from frozen S1."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from design_speed_reference_mpc import augmented_model


BUILD = 0x59490907
DT_S = 0.020
HORIZON = 15
HEADING_WEIGHT = 300.0
JOINT_WEIGHT = 5.0
RATE_WEIGHT = 0.02
REFERENCE_WEIGHT = np.asarray([0.16, 0.02])
REFERENCE_DELTA_WEIGHT = np.asarray([10.0, 0.5])
PREVIOUS_JOINT_WEIGHT = 10.0
PREVIOUS_REFERENCE_DELTA_WEIGHT = np.asarray([5.0, 0.5])
JOINT_ENVELOPE_DEG = np.asarray([-53.677734375, 26.080078125])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite_horizon_gain(a: np.ndarray, b: np.ndarray, joint_weight: float,
                        reference_delta_weight: np.ndarray) -> np.ndarray:
    n, m = a.shape[0], b.shape[1]
    q = np.zeros((n, n), dtype=float)
    q[0, 0] = HEADING_WEIGHT
    q[1, 1] = joint_weight
    q[2, 2] = q[3, 3] = RATE_WEIGHT
    r = np.diag(REFERENCE_WEIGHT)
    rd = np.diag(reference_delta_weight)
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


def project_joint_envelope(reference: np.ndarray, joint_deg: float) -> np.ndarray:
    result = reference.copy()
    relative = np.clip(result[1] - result[0],
                       (JOINT_ENVELOPE_DEG[0] - joint_deg) / DT_S,
                       (JOINT_ENVELOPE_DEG[1] - joint_deg) / DT_S)
    result[1] = result[0] + relative
    return result


def simulate(a: np.ndarray, b: np.ndarray, gain: np.ndarray, *,
             heading_error_deg: float = 0.0, target_rate_dps: float = 0.0,
             target_rate_seconds: float = 0.0, initial_big_rate_dps: float = 0.0,
             seconds: float = 8.0) -> dict:
    state = np.zeros(a.shape[0], dtype=float)
    state[0] = heading_error_deg
    state[2] = state[4] = initial_big_rate_dps
    state[13] = 1.0
    references: list[np.ndarray] = []
    joints: list[float] = []
    headings: list[float] = []
    for step in range(round(seconds / DT_S)):
        state[12] = target_rate_dps if step * DT_S < target_rate_seconds else 0.0
        headings.append(float(state[0]))
        joints.append(float(state[1]))
        reference = project_joint_envelope(-gain @ state, state[1])
        references.append(reference)
        state = a @ state + b @ reference
        state[13] = 1.0
    refs = np.asarray(references)
    headings_array = np.asarray(headings)
    outside = np.flatnonzero(np.abs(headings_array) >= 0.2)
    return {
        "heading_settle_within_0p2deg_s":
            float((outside[-1] + 1) * DT_S) if len(outside) else 0.0,
        "peak_abs_small_joint_deg": float(np.max(np.abs(joints))),
        "peak_abs_reference_dps": [float(value) for value in np.max(np.abs(refs), axis=0)],
        "maximum_reference_step_dps":
            [float(value) for value in np.max(np.abs(np.diff(refs, axis=0)), axis=0)],
    }


def generate(model_path: Path) -> dict:
    source = json.loads(model_path.read_text(encoding="utf-8"))
    a, b, _, _ = augmented_model(source["frozen_model"])
    gain = finite_horizon_gain(a, b, JOINT_WEIGHT, REFERENCE_DELTA_WEIGHT)
    previous_gain = finite_horizon_gain(
        a, b, PREVIOUS_JOINT_WEIGHT, PREVIOUS_REFERENCE_DELTA_WEIGHT)
    dynamic_radius = max(abs(np.linalg.eigvals((a - b @ gain)[:12, :12])))
    scenarios = {
        "heading_step_10deg": dict(heading_error_deg=10.0),
        "target_30dps_then_hard_stop": dict(target_rate_dps=30.0,
                                              target_rate_seconds=1.2),
        "initial_big_rate_30dps": dict(initial_big_rate_dps=30.0),
    }
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
        "change_from_59490906": {
            "small_joint_weight": [PREVIOUS_JOINT_WEIGHT, JOINT_WEIGHT],
            "big_reference_delta_weight":
                [float(PREVIOUS_REFERENCE_DELTA_WEIGHT[0]),
                 float(REFERENCE_DELTA_WEIGHT[0])],
            "small_reference_delta_weight": [0.5, 0.5],
            "speed_limit_policy": "unchanged: no ordinary MPC speed or slew clamp",
        },
        "frozen_model_comparison": {
            name: {
                "build_59490906": simulate(a, b, previous_gain, **arguments),
                "build_59490907": simulate(a, b, gain, **arguments),
            }
            for name, arguments in scenarios.items()
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
