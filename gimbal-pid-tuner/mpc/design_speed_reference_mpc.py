#!/usr/bin/env python3
"""Freeze a small dense speed-reference MPC from the accepted S1 ARX model.

The generated controller predicts the two already closed speed loops.  Its
outputs are speed references, so the existing speed PIDs and effort limits stay
in the real-time path.  The unconstrained first move is precomputed offline;
firmware applies the frozen speed and slew bounds after that move.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


BUILD = 0x59490903
DT_S = 0.020
HORIZON = 15
STATE_NAMES = [
    "heading_error_deg",
    "small_joint_deg",
    "big_rate_deviation_km1_dps",
    "small_rate_deviation_km1_dps",
    "big_rate_deviation_km2_dps",
    "small_rate_deviation_km2_dps",
    "big_reference_km1_dps",
    "small_reference_km1_dps",
    "big_reference_km2_dps",
    "small_reference_km2_dps",
    "big_reference_km3_dps",
    "small_reference_km3_dps",
    "target_heading_rate_dps",
    "affine_one",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def physical_arx(frozen: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    c = np.asarray(frozen["normalized_coefficient_feature_rows_by_output_columns"], dtype=float)
    sy = np.asarray(list(frozen["output_scale_dps"].values()), dtype=float)
    su = np.asarray(list(frozen["input_scale_dps"].values()), dtype=float)
    # Rows are [y(k-1), y(k-2), u(k), u(k-1), u(k-2), u(k-3)],
    # each group containing big then small.  C stores feature x output.
    blocks = []
    for start, source_scale in ((0, sy), (2, sy), (4, su), (6, su), (8, su), (10, su)):
        normalized = c[start : start + 2, :].T
        blocks.append(np.diag(sy) @ normalized @ np.diag(1.0 / source_scale))
    return np.stack(blocks[:2]), np.stack(blocks[2:]), sy


def augmented_model(frozen: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    a_lags, b_lags, _ = physical_arx(frozen)
    baseline = np.asarray(list(frozen["output_baseline_dps"].values()), dtype=float)
    n, m = len(STATE_NAMES), 2
    a = np.zeros((n, n), dtype=float)
    b = np.zeros((n, m), dtype=float)

    # Current physical output y(k) = baseline + z(k), where z is the ARX output deviation.
    f = np.zeros((m, n), dtype=float)
    f[:, 2:4] = a_lags[0]
    f[:, 4:6] = a_lags[1]
    f[:, 6:8] = b_lags[1]
    f[:, 8:10] = b_lags[2]
    f[:, 10:12] = b_lags[3]
    g = b_lags[0]

    # e = target heading - inertial heading; qdot = small inertial rate - big motor rate.
    a[0, 0] = 1.0
    a[0, 12] = DT_S
    a[0, :] -= DT_S * f[1, :]
    a[0, 13] -= DT_S * baseline[1]
    b[0, :] -= DT_S * g[1, :]
    a[1, 1] = 1.0
    a[1, :] += DT_S * (f[1, :] - f[0, :])
    a[1, 13] += DT_S * (baseline[1] - baseline[0])
    b[1, :] += DT_S * (g[1, :] - g[0, :])

    # Shift output and input histories after applying u(k).
    a[2:4, :] = f
    b[2:4, :] = g
    a[4:6, 2:4] = np.eye(2)
    b[6:8, :] = np.eye(2)
    a[8:10, 6:8] = np.eye(2)
    a[10:12, 8:10] = np.eye(2)
    a[12, 12] = 1.0
    a[13, 13] = 1.0
    return a, b, f, g


def finite_horizon_gain(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    n, m = a.shape[0], b.shape[1]
    # Cost state: heading tracking, strong joint centering, and mild rate damping.
    q = np.zeros((n, n), dtype=float)
    q[0, 0] = 300.0
    q[1, 1] = 100.0
    q[2, 2] = q[3, 3] = 0.02
    r = np.diag([0.16, 0.02])  # big reference is deliberately more expensive.
    rd = np.diag([0.5, 0.5])

    # Dynamic programming with (u-u_prev)'Rd(u-u_prev); u_prev is state[6:8].
    p = q.copy()
    k0 = None
    select_prev = np.zeros((m, n), dtype=float)
    select_prev[:, 6:8] = np.eye(m)
    for _ in range(HORIZON):
        h = r + rd + b.T @ p @ b
        cross = b.T @ p @ a - rd @ select_prev
        k = np.linalg.solve(h, cross)
        closed = a - b @ k
        p = q + closed.T @ p @ closed + k.T @ r @ k + (k + select_prev).T @ rd @ (k + select_prev)
        k0 = k
    assert k0 is not None
    return k0


def apply_limits(u: np.ndarray, previous: np.ndarray, q_deg: float) -> np.ndarray:
    speed = np.asarray([30.0, 60.0])
    slew = np.asarray([6.0, 12.0])
    bounded = np.clip(u, -speed, speed)
    bounded = np.clip(bounded, previous - slew, previous + slew)
    # Reference-domain one-step joint envelope; the existing measured-state
    # YawLimits_Speed/Effort guards remain authoritative in firmware.
    q_lo, q_hi = -53.677734375, 26.080078125
    relative = bounded[1] - bounded[0]
    relative = np.clip(relative, (q_lo - q_deg) / DT_S, (q_hi - q_deg) / DT_S)
    bounded[1] = np.clip(bounded[0] + relative, -speed[1], speed[1])
    return bounded


def simulate(a: np.ndarray, b: np.ndarray, gain: np.ndarray, scenario: dict) -> dict:
    x = np.zeros(a.shape[0], dtype=float)
    x[0] = scenario.get("heading_error_deg", 0.0)
    x[1] = scenario.get("small_joint_deg", 0.0)
    x[12] = scenario.get("target_heading_rate_dps", 0.0)
    x[13] = 1.0
    previous = np.zeros(2, dtype=float)
    peak_q = abs(x[1])
    peak_u = np.zeros(2, dtype=float)
    for step in range(int(scenario.get("seconds", 8.0) / DT_S)):
        if scenario.get("target_heading_rate_dps", 0.0):
            # A finite rate command then a hold catches accumulated joint travel.
            x[12] = scenario["target_heading_rate_dps"] if step * DT_S < scenario.get("rate_seconds", 2.0) else 0.0
        u = apply_limits(-gain @ x, previous, x[1])
        x = a @ x + b @ u
        x[13] = 1.0
        previous = u
        peak_q = max(peak_q, abs(x[1]))
        peak_u = np.maximum(peak_u, np.abs(u))
    return {
        "name": scenario["name"],
        "final_heading_error_deg": float(x[0]),
        "final_small_joint_deg": float(x[1]),
        "peak_abs_small_joint_deg": float(peak_q),
        "peak_abs_reference_dps": [float(v) for v in peak_u],
    }


def design(model_path: Path) -> dict:
    report = json.loads(model_path.read_text(encoding="utf-8"))
    frozen = report["frozen_model"]
    if frozen["sample_period_ms"] != 20:
        raise ValueError("accepted model is not the frozen 20 ms model")
    a_lags, b_lags, _ = physical_arx(frozen)
    a, b, _, _ = augmented_model(frozen)
    gain = finite_horizon_gain(a, b)
    scenarios = [
        {"name": "positive_10deg_step", "heading_error_deg": 10.0, "seconds": 8.0},
        {"name": "negative_10deg_step", "heading_error_deg": -10.0, "seconds": 8.0},
        {"name": "positive_joint_recovery", "small_joint_deg": 25.0, "seconds": 8.0},
        {"name": "negative_joint_recovery", "small_joint_deg": -50.0, "seconds": 8.0},
        {"name": "30dps_two_second_rate", "target_heading_rate_dps": 30.0, "rate_seconds": 2.0, "seconds": 8.0},
        {"name": "minus_30dps_two_second_rate", "target_heading_rate_dps": -30.0, "rate_seconds": 2.0, "seconds": 8.0},
    ]
    return {
        "status": "OPERATOR_ACCEPTED_MODEL_DEPLOYMENT_FROZEN",
        "build": f"0x{BUILD:08X}",
        "model_report": str(model_path),
        "model_report_sha256": sha256(model_path),
        "model_scope": report["model_scope"],
        "acceptance_provenance": {
            "S3_formal_data_gate": "rejected_after_active_window_by_transient_ins_not_ready",
            "S3_active_window_diagnostic": "passed_without_refit",
            "operator_decision": "deploy_current_frozen_model_without_new_path_or_refit",
        },
        "controller": {
            "sample_period_ms": int(DT_S * 1000),
            "prediction_horizon_steps": HORIZON,
            "prediction_horizon_ms": int(DT_S * 1000 * HORIZON),
            "state_names": STATE_NAMES,
            "state_transition": a.tolist(),
            "input_transition": b.tolist(),
            "first_move_feedback_gain": gain.tolist(),
            "law": "u_unconstrained=-Kx; then frozen speed, slew and joint-envelope limits",
            "weights": {
                "heading_error": 300.0,
                "small_joint": 100.0,
                "rate_deviation_each": 0.02,
                "reference": [0.16, 0.02],
                "reference_delta": [0.5, 0.5],
            },
            "limits": {
                "speed_reference_abs_dps": [30.0, 60.0],
                "speed_reference_delta_per_20ms_dps": [6.0, 12.0],
                "reference_joint_envelope_deg": [-53.677734375, 26.080078125],
            },
            "activation": "automatic gimbal mode only; manual, UP, tune lock, identification ownership or invalid feedback resets MPC",
            "fallback": "same-cycle legacy angle/coordinator control; stale MPC output is never reused",
        },
        "physical_arx": {
            "output_lag_matrices": a_lags.tolist(),
            "reference_lag_matrices": b_lags.tolist(),
            "output_baseline_dps": list(frozen["output_baseline_dps"].values()),
        },
        "deterministic_model_scenarios": [simulate(a, b, gain, s) for s in scenarios],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = design(args.model.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
