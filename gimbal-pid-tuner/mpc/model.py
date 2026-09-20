"""Offline local two-joint mechanics. Defaults are synthetic, NOT identified."""

from dataclasses import dataclass

import numpy as np
from scipy.signal import cont2discrete


SOFT_MIN = np.deg2rad((5458 - 6816) * 360 / 8192 + 3)
SOFT_MAX = np.deg2rad((7546 - 6816) * 360 / 8192 - 3)
HEADING = np.array([1., 1., 0., 0., 0., 0.])


@dataclass(frozen=True)
class Model:
    a: np.ndarray
    b: np.ndarray
    dt: float
    provenance: str

    def __post_init__(self):
        if (self.a.shape != (6, 6) or self.b.shape != (6, 2) or
                not np.isfinite(self.a).all() or not np.isfinite(self.b).all() or
                not np.isfinite(self.dt) or self.dt <= 0):
            raise ValueError("invalid six-state/two-input discrete model")


def synthetic_model(dt=.02, inertia_scale=1.):
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("sample period must be positive")
    if not np.isfinite(inertia_scale) or inertia_scale <= 0:
        raise ValueError("inertia scale must be positive")
    # General SPD local inertia matrix allows off-axis coupling. Its entries,
    # damping, voltage-to-torque gains and actuator time constants are invented.
    inertia = inertia_scale * np.array([[.08, .035], [.035, .06]])
    damping = np.diag([.06, .04])
    gain = np.diag([.04, .04])
    tau = np.array([.025, .020])
    ac = np.zeros((6, 6))
    ac[:2, 2:4] = np.eye(2)
    ac[2:4, 2:4] = -np.linalg.solve(inertia, damping)
    ac[2:4, 4:] = np.linalg.solve(inertia, gain)
    ac[4:, 4:] = -np.diag(1 / tau)
    bc = np.zeros((6, 2))
    bc[4:, :] = np.diag(1 / tau)
    ad, bd, _, _, _ = cont2discrete((ac, bc, np.eye(6), np.zeros((6, 2))), dt)
    return Model(ad, bd, dt, "SYNTHETIC_UNIDENTIFIED_NO_HARDWARE_USE")
