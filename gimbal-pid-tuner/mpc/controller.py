"""Sparse constrained MPC, offline only. No serial/CAN or fallback actuator path."""

from dataclasses import dataclass
import time

import numpy as np
import osqp
from scipy import sparse

from model import HEADING, SOFT_MIN, SOFT_MAX


@dataclass
class Result:
    accepted: bool
    status: str
    command: np.ndarray | None
    states: np.ndarray | None
    solve_s: float
    violation: float = float("inf")


class MPC:
    def __init__(self, model, horizon=30, big_motion_weight=25., max_iter=10000):
        if type(horizon) is not int or horizon < 2 or horizon > 200:
            raise ValueError("invalid horizon")
        if not np.isfinite(big_motion_weight) or big_motion_weight < 0:
            raise ValueError("invalid big-motion cost")
        self.model, self.n = model, horizon
        self.nx = (horizon + 1) * 6
        self.nu = horizon * 2
        self.slack_start = self.nx + self.nu
        self.size = self.slack_start + (horizon + 1) * 2
        self.effort = np.array([30., 6.])
        self.heading_weight = 2000.
        self.big_hold_weight = .2
        qx = self.heading_weight * np.outer(HEADING, HEADING)
        qx += np.diag([self.big_hold_weight, .02, big_motion_weight, .15, 0., 0.])
        ru = sparse.diags([.015, .015])
        delta = sparse.eye(horizon, format="csc") - sparse.eye(horizon, k=-1, format="csc")
        self.du = sparse.kron(delta, sparse.eye(2), format="csc")
        self.slew_weight = .05
        pu = sparse.kron(sparse.eye(horizon), ru) + self.slew_weight * self.du.T @ self.du
        # Soft comfort interval leaves small-yaw aiming reserve without forcing
        # recentering at every heading change. Travel bounds remain hard.
        self.comfort = (SOFT_MIN * .7, SOFT_MAX * .7)
        comfort_weight = 20.
        p = sparse.block_diag([sparse.kron(sparse.eye(horizon + 1), qx), pu,
            sparse.eye((horizon + 1) * 2) * comfort_weight], format="csc") * 2
        ax = sparse.kron(sparse.eye(horizon + 1), -sparse.eye(6))
        ax += sparse.kron(sparse.eye(horizon + 1, k=-1), model.a)
        bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, horizon)), sparse.eye(horizon)]), model.b)
        eq = sparse.hstack([ax, bu, sparse.csc_matrix((self.nx, (horizon + 1) * 2))], format="csc")
        # Require zero terminal relative small-joint rate as a braking condition.
        # This does not prove continuous-time safety or recursive feasibility.
        terminal = sparse.csc_matrix(([1.], ([0], [horizon * 6 + 3])), shape=(1, self.size))
        comfort = sparse.lil_matrix(((horizon + 1) * 2, self.size))
        for k in range(horizon + 1):
            comfort[2 * k, 6 * k + 1] = 1.
            comfort[2 * k + 1, 6 * k + 1] = -1.
            comfort[2 * k, self.slack_start + 2 * k] = -1.
            comfort[2 * k + 1, self.slack_start + 2 * k + 1] = -1.
        self.constraint = sparse.vstack([eq, sparse.eye(self.size), terminal, comfort], format="csc")
        low = np.full(self.size, -np.inf)
        high = np.full(self.size, np.inf)
        low[1:self.nx:6], high[1:self.nx:6] = SOFT_MIN, SOFT_MAX
        # Keep future predictions off the exact boundary: solver roundoff must
        # not turn the next measured state into an out-of-bounds initial state.
        self.prediction_margin = np.deg2rad(.1)
        low[7:self.nx:6] += self.prediction_margin
        high[7:self.nx:6] -= self.prediction_margin
        low[self.nx:self.slack_start] = np.tile(-self.effort, horizon)
        high[self.nx:self.slack_start] = np.tile(self.effort, horizon)
        low[self.slack_start:] = 0.
        self.lower = np.r_[np.zeros(self.nx), low, 0., np.full((horizon + 1) * 2, -np.inf)]
        self.upper = np.r_[np.zeros(self.nx), high, 0.,
                           np.tile([self.comfort[1], -self.comfort[0]], horizon + 1)]
        self.solver = osqp.OSQP()
        self.solver.setup(P=sparse.triu(p, format="csc"), q=np.zeros(self.size),
            A=self.constraint, l=self.lower, u=self.upper, verbose=False,
            eps_abs=1e-6, eps_rel=1e-6, max_iter=max_iter, warm_starting=True,
            polishing=True)

    def solve(self, state, heading_reference, previous_input=None, big_anchor=0.,
              sample_age_s=0., deadline_s=.2):
        started = time.perf_counter()
        state = np.asarray(state, dtype=float)
        previous = np.zeros(2) if previous_input is None else np.asarray(previous_input, dtype=float)
        target = np.asarray(heading_reference, dtype=float)
        if target.ndim == 0:
            target = np.full(self.n + 1, target)
        if (state.shape != (6,) or previous.shape != (2,) or target.shape != (self.n + 1,) or
                not all(np.isfinite(v).all() for v in (state, previous, target)) or
                not np.isfinite(big_anchor) or not np.isfinite(sample_age_s) or
                sample_age_s < 0 or sample_age_s > 2 * self.model.dt or
                not np.isfinite(deadline_s) or deadline_s <= 0 or
                np.any(np.abs(previous) > self.effort + 2e-5)):
            return Result(False, "invalid_or_stale_input", None, None, time.perf_counter() - started)
        if state[1] < SOFT_MIN or state[1] > SOFT_MAX:
            return Result(False, "outside_soft_limits", None, None, time.perf_counter() - started)
        linear = np.zeros(self.size)
        lx = linear[:self.nx].reshape(-1, 6)
        with np.errstate(over="ignore", invalid="ignore"):
            lx[:] = -2 * self.heading_weight * target[:, None] * HEADING
            lx[:, 0] -= 2 * self.big_hold_weight * big_anchor
        linear[self.nx:self.nx + 2] = -2 * self.slew_weight * previous
        if not np.isfinite(linear).all():
            return Result(False, "cost_overflow", None, None, time.perf_counter() - started)
        self.lower[:6] = self.upper[:6] = -state
        self.solver.update(q=linear, l=self.lower, u=self.upper)
        self.solver.update_settings(time_limit=deadline_s)
        solution = self.solver.solve(raise_error=False)
        elapsed = time.perf_counter() - started
        if solution.info.status != "solved" or solution.x is None:
            return Result(False, solution.info.status, None, None, elapsed)
        if elapsed > deadline_s:
            return Result(False, "host_deadline_exceeded", None, None, elapsed)
        if not np.isfinite(solution.x).all():
            return Result(False, "nonfinite_solution", None, None, elapsed)
        value = self.constraint @ solution.x
        violation = max(0., float(np.max(self.lower - value)), float(np.max(value - self.upper)))
        if violation > 2e-5:
            return Result(False, "constraint_residual", None, None, elapsed, violation)
        return Result(True, "solved", solution.x[self.nx:self.nx + 2].copy(),
                      solution.x[:self.nx].reshape(-1, 6).copy(), elapsed, violation)
