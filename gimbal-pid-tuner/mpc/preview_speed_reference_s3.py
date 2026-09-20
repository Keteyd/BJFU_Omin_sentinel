"""Reproduce and audit the frozen S3 reference without controlling hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN = Path(__file__).with_name("speed_reference_s3_plan.json")
OLD_PLAN = Path(__file__).with_name("speed_reference_excitation_plan.json")
DT = .004


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def wave(t, axis_plan, duration=28.):
    t = np.asarray(t, dtype=float)
    result = np.zeros_like(t)
    active = (t > 0) & (t < duration)
    x = t[active] / duration
    window = np.sin(np.pi*x)**2
    window_rate = np.pi/duration*np.sin(2*np.pi*x)
    harmonic = np.asarray(axis_plan["harmonics"], dtype=float)
    amplitude = np.asarray(axis_plan["primitive_amplitude_deg"], dtype=float)
    phase = np.asarray(axis_plan["phase_rad"], dtype=float)
    omega = 2*np.pi*harmonic/duration
    angle = t[active, None]*omega+phase
    position = np.sum(amplitude*np.sin(angle), axis=1)
    position_rate = np.sum(amplitude*omega*np.cos(angle), axis=1)
    result[active] = window_rate*position+window*position_rate
    return result


def reference_metrics(values):
    position = np.concatenate(([0.], np.cumsum((values[1:]+values[:-1])*.5*DT)))
    return {
        "peak_abs_speed_dps": float(np.max(np.abs(values))),
        "rms_speed_dps": float(np.sqrt(np.mean(values**2))),
        "peak_abs_position_deg": float(np.max(np.abs(position))),
        "net_position_deg": float(position[-1]),
        "peak_abs_acceleration_dps2": float(np.max(np.abs(np.gradient(values, DT)))),
        "start_dps": float(values[0]), "end_dps": float(values[-1]),
    }


def model_preview(plan):
    report_path = ROOT/plan["unchanged_frozen_model"]["report"]
    if sha256(report_path) != plan["unchanged_frozen_model"]["report_sha256"]:
        raise ValueError("frozen S1 model hash mismatch")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    model = report["frozen_model"]
    coefficient = np.asarray(model["normalized_coefficient_feature_rows_by_output_columns"])
    baseline = np.asarray(list(model["output_baseline_dps"].values()))
    output_scale = np.asarray(list(model["output_scale_dps"].values()))
    input_scale = np.asarray(list(model["input_scale_dps"].values()))
    time = np.arange(.020, 36.0001, .020)
    reference = np.zeros((len(time), 2))
    active = (time >= 3.) & (time < 31.)
    for axis, name in enumerate(("big", "small")):
        reference[active, axis] = wave(time[active]-3., plan["waveform"][name])
    normalized_reference = reference/input_scale
    normalized_output = np.zeros_like(reference)
    for k in range(4, len(time)):
        feature = np.concatenate([
            normalized_output[k-1], normalized_output[k-2],
            normalized_reference[k], normalized_reference[k-1],
            normalized_reference[k-2], normalized_reference[k-3]])
        normalized_output[k] = feature@coefficient
    output = normalized_output*output_scale+baseline
    integrate = lambda rate: np.concatenate(
        ([0.], np.cumsum((rate[1:]+rate[:-1])*.5*.020)))
    big_position = integrate(output[:, 0])
    heading = integrate(output[:, 1])
    small_joint_proxy = heading-big_position
    return {
        "method": "unchanged frozen S1 model; preview is not an S3 measurement or gate input",
        "peak_abs_big_rate_dps": float(np.max(np.abs(output[:, 0]))),
        "peak_abs_small_inertial_rate_dps": float(np.max(np.abs(output[:, 1]))),
        "peak_abs_big_motion_deg": float(np.max(np.abs(big_position))),
        "peak_abs_heading_motion_deg": float(np.max(np.abs(heading))),
        "peak_abs_small_joint_motion_proxy_deg": float(np.max(np.abs(small_joint_proxy))),
        "terminal_big_motion_deg": float(big_position[-1]),
        "terminal_heading_motion_deg": float(heading[-1]),
        "terminal_small_joint_motion_proxy_deg": float(small_joint_proxy[-1]),
    }


def preview(plan_path=DEFAULT_PLAN):
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    old = json.loads(OLD_PLAN.read_text(encoding="utf-8"))
    prior = {harmonic for phase in old["waveform"]["sets"].values()
             for axis in ("big", "small") for harmonic in phase[axis]["harmonics"]}
    current = [harmonic for axis in ("big", "small")
               for harmonic in plan["waveform"][axis]["harmonics"]]
    if len(current) != len(set(current)) or prior.intersection(current):
        raise ValueError("S3 frequencies are not globally disjoint from S1/S2")
    time = np.arange(0., 28.+DT/2, DT)
    metrics = {axis: reference_metrics(wave(time, plan["waveform"][axis]))
               for axis in ("big", "small")}
    for axis in ("big", "small"):
        if metrics[axis]["peak_abs_speed_dps"] > plan["waveform"][axis]["peak_budget_dps"]:
            raise ValueError("S3 reference exceeds speed budget")
        if abs(metrics[axis]["net_position_deg"]) > 1e-6:
            raise ValueError("S3 reference does not have zero net travel")
    gate = plan["prospective_S3_acceptance_gate"]
    if "diagnostic only" not in gate["data_quality"]["raw_record_count_policy"].lower():
        raise ValueError("S3 raw record count must remain diagnostic only")
    return {
        "status": "S3_DESIGN_FROZEN_NOT_COLLECTED",
        "plan": str(Path(plan_path)), "plan_sha256": sha256(plan_path),
        "reference_metrics_at_4ms": metrics,
        "frequencies_disjoint_from_S1_S2": True,
        "frozen_model_preview": model_preview(plan),
        "hardware_takeover_allowed": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = preview(args.plan)
    text = json.dumps(result, indent=2, allow_nan=False)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
