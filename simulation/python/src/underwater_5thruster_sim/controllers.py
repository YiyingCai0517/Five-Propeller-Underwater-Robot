from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .allocation import ThrusterAllocator, AllocationResult
from .math_utils import wrap_angle
from .robot_params import RobotParams, make_default_robot


@dataclass
class ControllerGains:
    depth_kp: float = 7.0
    depth_kd: float = 5.5
    surge_kp: float = 5.0
    surge_drag_ff: float = 0.55
    roll_kp: float = 6.0
    roll_kd: float = 2.2
    pitch_kp: float = 6.0
    pitch_kd: float = 2.2
    yaw_kp: float = 3.5
    yaw_kd: float = 1.5
    yaw_rate_kp: float = 1.2
    max_surge_force: float = 15.0
    max_heave_force: float = 30.0
    max_roll_moment: float = 2.2
    max_pitch_moment: float = 2.5
    max_yaw_moment: float = 2.4


@dataclass
class ControllerOutput:
    command: np.ndarray
    desired_wrench: np.ndarray
    allocation: AllocationResult
    reference: dict


class FiveThrusterController:
    """Cascaded task-space controller plus 5-thruster allocation."""

    def __init__(self, params: RobotParams | None = None, gains: ControllerGains | None = None) -> None:
        self.params = params or make_default_robot()
        self.gains = gains or ControllerGains()
        self.allocator = ThrusterAllocator(self.params)

    def command(self, eta: np.ndarray, nu: np.ndarray, reference: dict) -> ControllerOutput:
        g = self.gains
        m_diag = self.params.mass_matrix_diag
        roll, pitch, yaw = eta[3:6]
        u, _, _, p, q, r = nu

        target_depth = float(reference.get("target_depth", 1.0))
        target_z = -target_depth
        z_error = target_z - eta[2]
        # Approximate world vertical speed by body w for small roll/pitch, adequate here.
        z_dot_error = 0.0 - nu[2]
        heave_acc_cmd = g.depth_kp * z_error + g.depth_kd * z_dot_error
        # Feed-forward balances residual weight/buoyancy; positive heave is upward.
        heave_force = (self.params.weight - self.params.buoyancy) + m_diag[2] * heave_acc_cmd
        heave_force = float(np.clip(heave_force, -g.max_heave_force, g.max_heave_force))

        target_speed = float(reference.get("target_speed", 0.0))
        surge_force = m_diag[0] * g.surge_kp * (target_speed - u)
        if g.surge_drag_ff > 0.0:
            surge_force += g.surge_drag_ff * self.params.quadratic_damping[0] * abs(target_speed) * target_speed
        surge_force = float(np.clip(surge_force, -g.max_surge_force, g.max_surge_force))

        roll_moment = m_diag[3] * (g.roll_kp * (0.0 - roll) + g.roll_kd * (0.0 - p))
        pitch_moment = m_diag[4] * (g.pitch_kp * (0.0 - pitch) + g.pitch_kd * (0.0 - q))
        roll_moment = float(np.clip(roll_moment, -g.max_roll_moment, g.max_roll_moment))
        pitch_moment = float(np.clip(pitch_moment, -g.max_pitch_moment, g.max_pitch_moment))

        if "target_yaw" in reference:
            yaw_error = float(wrap_angle(float(reference["target_yaw"]) - yaw))
            yaw_rate_ref = float(reference.get("target_yaw_rate", 0.0))
            yaw_moment = m_diag[5] * (g.yaw_kp * yaw_error + g.yaw_kd * (yaw_rate_ref - r))
        else:
            yaw_rate_ref = float(reference.get("target_yaw_rate", 0.0))
            yaw_moment = m_diag[5] * g.yaw_rate_kp * (yaw_rate_ref - r)
        yaw_moment = float(np.clip(yaw_moment, -g.max_yaw_moment, g.max_yaw_moment))

        desired_wrench = np.array(
            [surge_force, 0.0, heave_force, roll_moment, pitch_moment, yaw_moment], dtype=float
        )
        allocation = self.allocator.allocate(desired_wrench)
        return ControllerOutput(
            command=allocation.command,
            desired_wrench=desired_wrench,
            allocation=allocation,
            reference=reference.copy(),
        )
