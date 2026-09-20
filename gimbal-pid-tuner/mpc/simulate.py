"""Run synthetic closed-loop regressions; never open a vehicle connection."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from controller import MPC
from model import HEADING, SOFT_MAX, SOFT_MIN, synthetic_model


def run_case(name, initial, target, inertia_scale=1., duration=4., big_motion_weight=25.):
    model = synthetic_model()
    # Check intermediate plant points, not just MPC grid nodes.
    plant = synthetic_model(dt=model.dt / 4, inertia_scale=inertia_scale)
    controller = MPC(model, big_motion_weight=big_motion_weight)
    state = np.array(initial, dtype=float)
    previous = np.zeros(2)
    anchor = state[0]
    rows = []
    intersample_min = intersample_max = state[1]
    for step in range(round(duration / model.dt)):
        now = step * model.dt
        reference = np.array([target(now + j * model.dt) for j in range(controller.n + 1)])
        result = controller.solve(state, reference, previous, big_anchor=anchor)
        if not result.accepted:
            raise RuntimeError(f"{name} stopped at {now:.3f}s: {result.status}; no command reused")
        rows.append(dict(time_s=now, heading_rad=float(HEADING @ state), target_rad=float(reference[0]),
            big_rad=float(state[0]), small_rad=float(state[1]), big_rad_s=float(state[2]),
            small_rad_s=float(state[3]), big_input=float(result.command[0]),
            small_input=float(result.command[1]), solve_ms=result.solve_s * 1000,
            constraint_violation=result.violation))
        for _ in range(4):
            state = plant.a @ state + plant.b @ result.command
            intersample_min = min(intersample_min, state[1])
            intersample_max = max(intersample_max, state[1])
            if not SOFT_MIN - 2e-5 <= state[1] <= SOFT_MAX + 2e-5:
                raise RuntimeError(f"{name}: simulated plant crossed a soft limit")
        previous = result.command
    summary = dict(name=name, model=model.provenance, inertia_scale=inertia_scale,
        samples=len(rows), final_error_deg=float(np.rad2deg(target(duration) - HEADING @ state)),
        max_big_displacement_deg=float(np.rad2deg(max(abs(r["big_rad"] - anchor) for r in rows))),
        small_range_deg=np.rad2deg([intersample_min, intersample_max]).tolist(),
        final_small_deg=float(np.rad2deg(state[1])),
        peak_inputs=[max(abs(r[k]) for r in rows) for k in ("big_input", "small_input")],
        max_solve_ms=max(r["solve_ms"] for r in rows),
        solves_exceeding_nominal_period=sum(r["solve_ms"] > model.dt * 1000 for r in rows),
        p95_solve_ms=float(np.percentile([r["solve_ms"] for r in rows], 95)))
    return rows, summary


def scenarios():
    zero = np.zeros(6)
    left = zero.copy(); left[1] = SOFT_MAX - .04
    right = zero.copy(); right[1] = SOFT_MIN + .04
    impulse = zero.copy(); impulse[2:4] = [.4, -.7]
    wrap = zero.copy(); wrap[0] = 2 * np.pi - .03
    return [
        ("heading_step", zero, lambda t: np.deg2rad(10), 1.),
        ("positive_boundary", left, lambda t: left[1] + np.deg2rad(20), 1.),
        ("negative_boundary", right, lambda t: right[1] - np.deg2rad(20), 1.),
        ("initial_rate_disturbance", impulse, lambda t: 0., 1.),
        ("inertia_plus_25_percent", zero, lambda t: np.deg2rad(10), 1.25),
        ("unwrapped_full_turn", wrap, lambda t: wrap[0] + .08, 1.),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    summary = {"hardware_validated": False, "synthetic_only": True, "cases": [], "error": None}
    try:
        for name, initial, target, scale in scenarios():
            rows, result = run_case(name, initial, target, scale)
            with (args.output / (name + ".csv")).open("x", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            summary["cases"].append(result)
            print(json.dumps(result), flush=True)
    except RuntimeError as exc:
        summary["error"] = str(exc)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return 1 if summary["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
