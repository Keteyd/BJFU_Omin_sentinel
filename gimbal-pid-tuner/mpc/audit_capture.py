"""Offline input-excitation audit. Passing rank checks is NOT model validation."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def audit(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        records = list(csv.DictReader(stream))
    if len(records) < 3:
        raise ValueError("need at least three records")
    if any(int(r["valid"]) != 1 for r in records):
        raise ValueError("invalid records; do not fit across invalid observations")
    ticks = np.array([int(r["tick_ms"]) for r in records], dtype=np.int64)
    delta = np.diff(ticks) % (2 ** 32)
    segments = {int(r["segment"]) for r in records}
    sequence = np.array([int(r["sequence"]) for r in records], dtype=np.int64)
    if len(segments) != 1 or np.any(delta != delta[0]) or delta[0] not in (2, 25) or \
            np.any(np.diff(sequence) % (2 ** 16) != 1):
        raise ValueError("discontinuous capture; split into validated uniform segments")
    inputs = np.array([[float(r["big_command"]), float(r["small_command"])] for r in records])
    if not np.isfinite(inputs).all():
        raise ValueError("nonfinite motor command")
    centered = inputs - inputs.mean(axis=0)
    singular_values = np.linalg.svd(centered, compute_uv=False)
    rank = int(np.linalg.matrix_rank(centered))
    reasons = []
    if rank < 2:
        reasons.append("two independent actuator input directions are absent")
    if np.all(inputs[:, 0] == 0):
        reasons.append("big-yaw input is zero throughout; its input dynamics are not identifiable here")
    if delta[0] == 25:
        reasons.append("40 Hz observations alone cannot establish the high-bandwidth actuator model")
    return dict(source=str(path), records=len(records), dt_s=float(delta[0] / 1000),
        input_rank=rank, centered_input_singular_values=singular_values.tolist(),
        input_min=inputs.min(axis=0).tolist(), input_max=inputs.max(axis=0).tolist(),
        blockers=reasons, model_identified=False, hardware_takeover_allowed=False,
        note="Even rank two requires persistent excitation, state/lag regressors, closed-loop bias checks and held-out validation.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.capture), indent=2))


if __name__ == "__main__":
    main()
