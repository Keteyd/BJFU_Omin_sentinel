#!/usr/bin/env python3
"""Generate the 0x59490908 frequency-selective big-yaw MPC gain."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from design_speed_reference_mpc import augmented_model


BUILD = 0x59490908
DT_S = 0.020
HORIZON = 15
HEADING_WEIGHT = 300.0
JOINT_WEIGHT = 5.0
RATE_WEIGHT = 0.02
REFERENCE_WEIGHT = np.asarray([0.16, 0.02])
REFERENCE_DELTA_WEIGHT = np.asarray([10.0, 0.5])
REFERENCE_CURVATURE_WEIGHT = np.asarray([40.0, 0.0])
PREVIOUS_CURVATURE_WEIGHT = np.asarray([0.0, 0.0])
JOINT_ENVELOPE_DEG = np.asarray([-53.677734375, 26.080078125])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite_horizon_gain(a: np.ndarray, b: np.ndarray,
                        curvature_weight: np.ndarray) -> np.ndarray:
    n, m = a.shape[0], b.shape[1]
    q = np.zeros((n, n), dtype=float)
    q[0, 0] = HEADING_WEIGHT
    q[1, 1] = JOINT_WEIGHT
    q[2, 2] = q[3, 3] = RATE_WEIGHT
    r = np.diag(REFERENCE_WEIGHT)
    rd = np.diag(REFERENCE_DELTA_WEIGHT)
    rdd = np.diag(curvature_weight)

    # u(k)-u(k-1), where state[6:8] is u(k-1).
    select_previous = np.zeros((m, n), dtype=float)
    select_previous[:, 6:8] = np.eye(m)
    # u(k)-2u(k-1)+u(k-2).  The state already carries both histories.
    select_curvature = np.zeros((m, n), dtype=float)
    select_curvature[:, 6:8] = 2.0 * np.eye(m)
    select_curvature[:, 8:10] = -np.eye(m)

    p = q.copy()
    for _ in range(HORIZON):
        h = r + rd + rdd + b.T @ p @ b
        cross = (b.T @ p @ a - rd @ select_previous
                 - rdd @ select_curvature)
        gain = np.linalg.solve(h, cross)
        closed = a - b @ gain
        p = (q + closed.T @ p @ closed + gain.T @ r @ gain
             + (gain + select_previous).T @ rd @ (gain + select_previous)
             + (gain + select_curvature).T @ rdd
             @ (gain + select_curvature))
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


def sine_response(a: np.ndarray, b: np.ndarray, gain: np.ndarray,
                  frequency_hz: float, amplitude_dps: float = 30.0,
                  seconds: float = 20.0) -> dict:
    state = np.zeros(a.shape[0], dtype=float)
    state[13] = 1.0
    references: list[np.ndarray] = []
    joints: list[float] = []
    steps = round(seconds / DT_S)
    for step in range(steps):
        state[12] = amplitude_dps * np.sin(
            2.0 * np.pi * frequency_hz * step * DT_S)
        reference = project_joint_envelope(-gain @ state, state[1])
        references.append(reference)
        joints.append(float(state[1]))
        state = a @ state + b @ reference
        state[13] = 1.0
    steady = np.asarray(references)[steps // 2:]
    amplitudes = np.sqrt(2.0) * np.std(steady, axis=0)
    return {
        "reference_amplitude_dps": [float(value) for value in amplitudes],
        "peak_abs_small_joint_deg": float(np.max(np.abs(joints))),
    }


def generate(model_path: Path) -> dict:
    source = json.loads(model_path.read_text(encoding="utf-8"))
    a, b, _, _ = augmented_model(source["frozen_model"])
    gain = finite_horizon_gain(a, b, REFERENCE_CURVATURE_WEIGHT)
    previous_gain = finite_horizon_gain(a, b, PREVIOUS_CURVATURE_WEIGHT)
    dynamic_radius = max(abs(np.linalg.eigvals((a - b @ gain)[:12, :12])))
    frequencies = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)
    return {
        "status": "OPERATOR_ACCEPTED_FREQUENCY_SELECTIVE_REBALANCE",
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
                "reference_second_difference":
                    REFERENCE_CURVATURE_WEIGHT.tolist(),
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
        "change_from_59490907": {
            "small_joint_weight": [5.0, 5.0],
            "big_reference_delta_weight": [10.0, 10.0],
            "big_reference_second_difference_weight": [0.0, 40.0],
            "small_reference_second_difference_weight": [0.0, 0.0],
            "speed_limit_policy":
                "unchanged: no ordinary MPC speed or slew clamp",
        },
        "frozen_frequency_comparison": {
            f"{frequency:g}_Hz": {
                "build_59490907": sine_response(
                    a, b, previous_gain, frequency),
                "build_59490908": sine_response(a, b, gain, frequency),
            }
            for frequency in frequencies
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
