from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import rotation_matrix_body_to_world, euler_rates_from_body_rates, smooth_step, wrap_angle
from .robot_params import RobotParams, make_default_robot


@dataclass
class StepInfo:
    wrench_thruster: np.ndarray
    wrench_damping: np.ndarray
    wrench_restoring: np.ndarray
    wrench_total: np.ndarray
    actuator_command: np.ndarray


class UnderwaterRobot6DOF:
    """A compact 6-DOF underwater rigid-body simulator.

    Coordinates:
      world frame: X/Y horizontal, Z upward;
      body frame: X forward, Y left, Z upward;
      depth = -world_z.
    """

    def __init__(self, params: RobotParams | None = None, dt: float = 0.02) -> None:
        self.params = params or make_default_robot()
        self.dt = float(dt)
        self.eta = np.zeros(6, dtype=float)  # [x, y, z, roll, pitch, yaw]
        self.nu = np.zeros(6, dtype=float)   # [u, v, w, p, q, r] in body frame
        self.actuator_state = np.zeros(len(self.params.thrusters), dtype=float)
        self.last_info: StepInfo | None = None

    def reset(
        self,
        position: tuple[float, float, float] = (0.0, 0.0, -1.0),
        attitude: tuple[float, float, float] = (0.0, 0.0, 0.0),
        velocity: tuple[float, float, float, float, float, float] | None = None,
    ) -> np.ndarray:
        self.eta[:] = np.array([*position, *attitude], dtype=float)
        self.nu[:] = np.zeros(6, dtype=float) if velocity is None else np.array(velocity, dtype=float)
        self.actuator_state[:] = 0.0
        self.last_info = None
        return self.state.copy()

    @property
    def state(self) -> np.ndarray:
        return np.concatenate([self.eta, self.nu])

    @property
    def depth(self) -> float:
        return -float(self.eta[2])

    @property
    def body_to_world(self) -> np.ndarray:
        return rotation_matrix_body_to_world(*self.eta[3:6])

    @property
    def world_linear_velocity(self) -> np.ndarray:
        return self.body_to_world @ self.nu[:3]

    def _thruster_wrench(self, command: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        command = np.clip(np.asarray(command, dtype=float), -1.0, 1.0)
        self.actuator_state = smooth_step(
            self.actuator_state, command, self.dt, self.params.actuator_time_constant
        )
        wrench = np.zeros(6, dtype=float)
        for value, thruster in zip(self.actuator_state, self.params.thrusters):
            force = value * thruster.max_force * thruster.direction
            torque = np.cross(thruster.position, force)
            wrench[:3] += force
            wrench[3:] += torque
        return wrench, self.actuator_state.copy()

    def _damping_wrench(self) -> np.ndarray:
        r = self.body_to_world
        current_body = r.T @ self.params.current_world
        relative = self.nu.copy()
        relative[:3] -= current_body
        return -self.params.linear_damping * relative - self.params.quadratic_damping * np.abs(relative) * relative

    def _restoring_wrench(self) -> np.ndarray:
        # Net vertical force from weight + buoyancy, represented in body frame.
        net_world = np.array([0.0, 0.0, self.params.buoyancy - self.params.weight], dtype=float)
        force_body = self.body_to_world.T @ net_world

        roll, pitch, _ = self.eta[3:6]
        p, q, _ = self.nu[3:6]
        torque_body = np.array([
            -self.params.roll_stiffness * roll - self.params.roll_damping_extra * p,
            -self.params.pitch_stiffness * pitch - self.params.pitch_damping_extra * q,
            0.0,
        ])
        return np.concatenate([force_body, torque_body])

    def step(self, command: np.ndarray) -> tuple[np.ndarray, StepInfo]:
        wrench_thruster, actuator_command = self._thruster_wrench(command)
        wrench_damping = self._damping_wrench()
        wrench_restoring = self._restoring_wrench()
        wrench_total = wrench_thruster + wrench_damping + wrench_restoring

        nu_dot = wrench_total / self.params.mass_matrix_diag
        self.nu += self.dt * nu_dot

        # Keep the explicit integrator well-behaved if a user gives extreme commands.
        self.nu[:3] = np.clip(self.nu[:3], -2.5, 2.5)
        self.nu[3:] = np.clip(self.nu[3:], -4.0, 4.0)

        r_bw = self.body_to_world
        pos_dot = r_bw @ self.nu[:3]
        euler_dot = euler_rates_from_body_rates(self.eta[3], self.eta[4], self.nu[3:])
        self.eta[:3] += self.dt * pos_dot
        self.eta[3:6] += self.dt * euler_dot
        self.eta[3] = float(np.clip(self.eta[3], -1.2, 1.2))
        self.eta[4] = float(np.clip(self.eta[4], -1.2, 1.2))
        self.eta[5] = float(wrap_angle(self.eta[5]))

        info = StepInfo(
            wrench_thruster=wrench_thruster,
            wrench_damping=wrench_damping,
            wrench_restoring=wrench_restoring,
            wrench_total=wrench_total,
            actuator_command=actuator_command,
        )
        self.last_info = info
        return self.state.copy(), info
