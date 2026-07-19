"""Five-thruster underwater robot dynamics and task simulation."""

from .robot_params import RobotParams, Thruster, make_default_robot
from .dynamics import UnderwaterRobot6DOF
from .controllers import FiveThrusterController, ControllerGains
from .tasks import run_task, run_all_tasks

__all__ = [
    "RobotParams",
    "Thruster",
    "make_default_robot",
    "UnderwaterRobot6DOF",
    "FiveThrusterController",
    "ControllerGains",
    "run_task",
    "run_all_tasks",
]
