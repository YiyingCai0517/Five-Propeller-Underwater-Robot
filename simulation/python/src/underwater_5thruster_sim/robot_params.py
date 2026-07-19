from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass(frozen=True)
class Thruster:
    """Thruster geometry in the simulator body frame.

    Body frame used by this project:
      X: forward, Y: left, Z: upward.

    Positive command produces force = max_force * direction.
    """

    name: str
    joint_name: str
    position: np.ndarray
    direction: np.ndarray
    max_force: float
    kind: str

    def wrench_per_unit_command(self) -> np.ndarray:
        force = self.max_force * self.direction
        torque = np.cross(self.position, force)
        return np.concatenate([force, torque])


@dataclass
class RobotParams:
    mass: float
    inertia: np.ndarray
    added_mass: np.ndarray
    linear_damping: np.ndarray
    quadratic_damping: np.ndarray
    thrusters: list[Thruster]
    water_density: float = 1000.0
    gravity: float = 9.80665
    buoyancy_scale: float = 1.002
    actuator_time_constant: float = 0.08
    roll_stiffness: float = 0.90
    pitch_stiffness: float = 1.10
    roll_damping_extra: float = 0.10
    pitch_damping_extra: float = 0.12
    current_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

    @property
    def volume(self) -> float:
        return self.mass * self.buoyancy_scale / self.water_density

    @property
    def weight(self) -> float:
        return self.mass * self.gravity

    @property
    def buoyancy(self) -> float:
        return self.water_density * self.gravity * self.volume

    @property
    def mass_matrix_diag(self) -> np.ndarray:
        rigid = np.array([self.mass, self.mass, self.mass, *self.inertia], dtype=float)
        return rigid + self.added_mass


def _cad_to_sim_position(cad_xyz: tuple[float, float, float]) -> np.ndarray:
    """Map SolidWorks/URDF coordinates to the simulation body frame.

    The uploaded base mesh spans roughly x=[-0.187,0.156], y=[-1.556,-1.0], z=[-0.081,0.089].
    The exported base inertial origin appears to have Y/Z swapped.  We use the
    physically plausible CAD center [-0.015387, -1.2703, 0.0053548] as body origin.
    """
    cad_center = np.array([-0.015387, -1.2703, 0.0053548], dtype=float)
    x_cad, y_cad, z_cad = np.array(cad_xyz, dtype=float) - cad_center
    # Simulator X is original +Y; simulator Y is original +X; simulator Z is original +Z.
    return np.array([y_cad, x_cad, z_cad], dtype=float)


def make_default_robot() -> RobotParams:
    """Create the default robot model derived from simulatedrobot.zip.

    Three vertical thrusters are mapped from frontwheeljoint/rbj/lbj.
    Two horizontal thrusters are mapped from lfj/rfj.
    """
    thrusters = [
        Thruster(
            name="front_vertical",
            joint_name="frontwheeljoint",
            position=_cad_to_sim_position((-0.015431, -1.065, 0.0)),
            direction=np.array([0.0, 0.0, 1.0]),
            max_force=12.0,
            kind="vertical",
        ),
        Thruster(
            name="rear_right_vertical",
            joint_name="rbj",
            position=_cad_to_sim_position((-0.10685, -1.514, 0.0)),
            direction=np.array([0.0, 0.0, 1.0]),
            max_force=12.0,
            kind="vertical",
        ),
        Thruster(
            name="rear_left_vertical",
            joint_name="lbj",
            position=_cad_to_sim_position((0.076668, -1.520, 0.0023601)),
            direction=np.array([0.0, 0.0, 1.0]),
            max_force=12.0,
            kind="vertical",
        ),
        Thruster(
            name="left_horizontal",
            joint_name="lfj",
            position=_cad_to_sim_position((0.12044, -1.2421, 0.0014413)),
            direction=np.array([1.0, 0.0, 0.0]),
            max_force=8.0,
            kind="horizontal",
        ),
        Thruster(
            name="right_horizontal",
            joint_name="rfj",
            position=_cad_to_sim_position((-0.1513, -1.2421, 0.0074378)),
            direction=np.array([1.0, 0.0, 0.0]),
            max_force=8.0,
            kind="horizontal",
        ),
    ]

    # Mass from the URDF after fixing rfw mass to match the other small thrusters.
    mass = 2.1846 + 5.0 * 0.0046375

    # Approximate CAD bounding-box inertia transformed to simulator axes.
    # This is intentionally conservative to avoid unrealistically fast rotation.
    inertia = np.array([0.030, 0.062, 0.080], dtype=float)  # roll, pitch, yaw

    return RobotParams(
        mass=mass,
        inertia=inertia,
        added_mass=np.array([0.75, 1.10, 1.60, 0.015, 0.025, 0.035], dtype=float),
        linear_damping=np.array([2.8, 4.5, 7.0, 0.12, 0.16, 0.20], dtype=float),
        quadratic_damping=np.array([9.0, 14.0, 18.0, 0.35, 0.45, 0.55], dtype=float),
        thrusters=thrusters,
        buoyancy_scale=1.002,
    )
