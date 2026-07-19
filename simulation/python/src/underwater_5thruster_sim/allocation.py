from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .robot_params import RobotParams


@dataclass
class AllocationResult:
    command: np.ndarray
    achieved_wrench: np.ndarray
    desired_wrench: np.ndarray
    allocation_error: np.ndarray


class ThrusterAllocator:
    """Weighted least-squares allocator for the 5 thrusters."""

    def __init__(self, params: RobotParams) -> None:
        self.params = params
        self.matrix = np.column_stack([t.wrench_per_unit_command() for t in params.thrusters])
        # Sway is not directly actuated; give it a low weight.  Other DOFs are controlled.
        self.weights = np.diag([1.00, 0.05, 1.15, 0.70, 0.75, 1.00])

    def allocate(self, desired_wrench: np.ndarray) -> AllocationResult:
        desired_wrench = np.asarray(desired_wrench, dtype=float).reshape(6)
        command = np.zeros(len(self.params.thrusters), dtype=float)
        active = np.ones(len(command), dtype=bool)
        residual = desired_wrench.copy()

        # Active-set clipping.  It is small and deterministic, good enough for a course simulator.
        for _ in range(4):
            if not np.any(active):
                break
            a = self.weights @ self.matrix[:, active]
            b = self.weights @ residual
            sol, *_ = np.linalg.lstsq(a, b, rcond=None)
            trial = command.copy()
            trial[active] += sol
            clipped = np.clip(trial, -1.0, 1.0)
            newly_saturated = active & (np.abs(trial - clipped) > 1e-8)
            command = clipped
            residual = desired_wrench - self.matrix @ command
            if not np.any(newly_saturated):
                break
            active[newly_saturated] = False

        achieved = self.matrix @ command
        return AllocationResult(command=command, achieved_wrench=achieved, desired_wrench=desired_wrench, allocation_error=desired_wrench - achieved)
