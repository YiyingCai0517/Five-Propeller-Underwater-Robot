from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math
import time
from typing import Callable

import numpy as np

from .controllers import FiveThrusterController
from .dynamics import UnderwaterRobot6DOF
from .math_utils import wrap_angle
from .robot_params import make_default_robot


@dataclass
class TaskResult:
    task_name: str
    history: dict[str, np.ndarray]
    metrics: dict[str, float | int | str]
    output_dir: Path | None = None


class ReferenceGenerator:
    name = "base"
    duration = 20.0
    initial_depth = 0.25
    initial_yaw = 0.0

    def reference(self, t: float, eta: np.ndarray, nu: np.ndarray) -> dict:
        raise NotImplementedError

    def extra_log(self) -> dict[str, float]:
        return {}

    def metrics(self, hist: dict[str, np.ndarray]) -> dict[str, float | int | str]:
        depth = hist["depth"]
        target_depth = hist["target_depth"]
        yaw = hist["yaw"]
        target_yaw = hist["target_yaw"]
        # Ignore the first 20% transient for RMSE-like metrics.
        start = max(1, int(0.2 * len(depth)))
        return {
            "depth_rmse_m": float(np.sqrt(np.mean((depth[start:] - target_depth[start:]) ** 2))),
            "depth_final_error_m": float(depth[-1] - target_depth[-1]),
            "yaw_final_error_deg": float(np.rad2deg(wrap_angle(target_yaw[-1] - yaw[-1]))),
            "final_x_m": float(hist["x"][-1]),
            "final_y_m": float(hist["y"][-1]),
        }


class DepthForwardTask(ReferenceGenerator):
    name = "depth_forward"
    duration = 28.0
    initial_depth = 0.25

    def __init__(self, target_depth: float = 1.0, speed: float = 0.35, yaw: float = 0.0) -> None:
        self.target_depth = target_depth
        self.speed = speed
        self.yaw = yaw

    def reference(self, t: float, eta: np.ndarray, nu: np.ndarray) -> dict:
        # Smooth speed command avoids a large impulse at t=0.
        speed = self.speed * min(1.0, t / 4.0)
        return {"target_depth": self.target_depth, "target_speed": speed, "target_yaw": self.yaw}

    def metrics(self, hist: dict[str, np.ndarray]) -> dict[str, float | int | str]:
        m = super().metrics(hist)
        m["mean_speed_mps"] = float((hist["x"][-1] - hist["x"][0]) / (hist["time"][-1] - hist["time"][0]))
        return m


class TurnTask(ReferenceGenerator):
    name = "turn_90deg"
    duration = 34.0
    initial_depth = 0.25

    def __init__(self, target_depth: float = 1.0, speed: float = 0.25, turn_deg: float = 90.0) -> None:
        self.target_depth = target_depth
        self.speed = speed
        self.turn_rad = math.radians(turn_deg)
        self.turn_start = 5.0
        self.turn_rate_ref = math.radians(16.0)

    def reference(self, t: float, eta: np.ndarray, nu: np.ndarray) -> dict:
        speed = self.speed * min(1.0, t / 4.0)
        if t < self.turn_start:
            yaw_ref = 0.0
            yaw_rate_ref = 0.0
        else:
            yaw_ref = min(self.turn_rad, self.turn_rate_ref * (t - self.turn_start))
            yaw_rate_ref = self.turn_rate_ref if yaw_ref < self.turn_rad - 1e-6 else 0.0
        return {
            "target_depth": self.target_depth,
            "target_speed": speed,
            "target_yaw": yaw_ref,
            "target_yaw_rate": yaw_rate_ref,
        }

    def metrics(self, hist: dict[str, np.ndarray]) -> dict[str, float | int | str]:
        m = super().metrics(hist)
        m["final_yaw_deg"] = float(np.rad2deg(hist["yaw"][-1]))
        m["turn_radius_est_m"] = float(np.nanmax(np.abs(hist["y"])))
        return m


class RectangleTask(ReferenceGenerator):
    name = "rectangle_track"
    duration = 78.0
    initial_depth = 0.25

    def __init__(
        self,
        target_depth: float = 1.0,
        speed: float = 0.32,
        length: float = 2.0,
        width: float = 1.2,
        switch_radius: float = 0.18,
    ) -> None:
        self.target_depth = target_depth
        self.speed = speed
        self.switch_radius = switch_radius
        self.waypoints = np.array(
            [[0.0, 0.0], [length, 0.0], [length, width], [0.0, width], [0.0, 0.0]], dtype=float
        )
        self.current_index = 1
        self.completed = False
        self.closest_distance = math.inf

    def reference(self, t: float, eta: np.ndarray, nu: np.ndarray) -> dict:
        pos = eta[:2]
        if not self.completed:
            target = self.waypoints[self.current_index]
            dist = float(np.linalg.norm(target - pos))
            self.closest_distance = min(self.closest_distance, dist)
            if dist < self.switch_radius and self.current_index < len(self.waypoints) - 1:
                self.current_index += 1
                target = self.waypoints[self.current_index]
                dist = float(np.linalg.norm(target - pos))
            elif dist < self.switch_radius and self.current_index == len(self.waypoints) - 1:
                self.completed = True

        target = self.waypoints[self.current_index]
        delta = target - pos
        desired_yaw = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 1e-9 else eta[5]
        dist = float(np.linalg.norm(delta))
        speed = 0.0 if self.completed else self.speed * np.clip(dist / 0.5, 0.35, 1.0)
        return {
            "target_depth": self.target_depth,
            "target_speed": float(speed),
            "target_yaw": desired_yaw,
            "waypoint_index": float(self.current_index),
            "target_x": float(target[0]),
            "target_y": float(target[1]),
        }

    def extra_log(self) -> dict[str, float]:
        return {"waypoint_index": float(self.current_index)}

    def _path_distance(self, points: np.ndarray) -> np.ndarray:
        # Distance from each point to the closest rectangle segment.
        distances = []
        for p in points:
            best = math.inf
            for a, b in zip(self.waypoints[:-1], self.waypoints[1:]):
                ab = b - a
                lam = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0)
                proj = a + lam * ab
                best = min(best, float(np.linalg.norm(p - proj)))
            distances.append(best)
        return np.array(distances)

    def metrics(self, hist: dict[str, np.ndarray]) -> dict[str, float | int | str]:
        m = super().metrics(hist)
        pts = np.column_stack([hist["x"], hist["y"]])
        start = max(1, int(0.1 * len(pts)))
        d = self._path_distance(pts[start:])
        m["rectangle_cross_track_rmse_m"] = float(np.sqrt(np.mean(d**2)))
        m["rectangle_cross_track_max_m"] = float(np.max(d))
        m["completed_laps"] = int(self.completed)
        m["final_waypoint_index"] = int(self.current_index)
        return m


def make_task(task_name: str) -> ReferenceGenerator:
    table: dict[str, Callable[[], ReferenceGenerator]] = {
        "depth_forward": DepthForwardTask,
        "forward": DepthForwardTask,
        "turn": TurnTask,
        "turn_90deg": TurnTask,
        "rectangle": RectangleTask,
        "rectangle_track": RectangleTask,
    }
    if task_name not in table:
        raise ValueError(f"Unknown task '{task_name}'. Choose from: {', '.join(sorted(table))}")
    return table[task_name]()


def _initial_history(thruster_names: list[str]) -> dict[str, list[float]]:
    hist: dict[str, list[float]] = {
        "time": [], "x": [], "y": [], "z": [], "depth": [],
        "roll": [], "pitch": [], "yaw": [],
        "u": [], "v": [], "w": [], "p": [], "q": [], "r": [],
        "target_depth": [], "target_speed": [], "target_yaw": [], "target_yaw_rate": [],
        "target_x": [], "target_y": [], "waypoint_index": [],
        "desired_fx": [], "desired_fz": [], "desired_mx": [], "desired_my": [], "desired_mz": [],
        "alloc_error_norm": [],
    }
    for name in thruster_names:
        hist[f"cmd_{name}"] = []
    return hist


def run_task(
    task_name: str,
    output_dir: str | Path | None = None,
    dt: float = 0.02,
    duration: float | None = None,
    make_plots: bool = True,
) -> TaskResult:
    task = make_task(task_name)
    params = make_default_robot()
    robot = UnderwaterRobot6DOF(params=params, dt=dt)
    controller = FiveThrusterController(params=params)
    robot.reset(position=(0.0, 0.0, -task.initial_depth), attitude=(0.0, 0.0, task.initial_yaw))

    total_time = float(duration if duration is not None else task.duration)
    steps = int(total_time / dt)
    hist = _initial_history([t.name for t in params.thrusters])

    for k in range(steps + 1):
        t = k * dt
        eta = robot.eta.copy()
        nu = robot.nu.copy()
        ref = task.reference(t, eta, nu)
        out = controller.command(eta, nu, ref)

        hist["time"].append(t)
        for key, val in zip(["x", "y", "z", "roll", "pitch", "yaw"], eta):
            hist[key].append(float(val))
        hist["depth"].append(float(robot.depth))
        for key, val in zip(["u", "v", "w", "p", "q", "r"], nu):
            hist[key].append(float(val))
        hist["target_depth"].append(float(ref.get("target_depth", np.nan)))
        hist["target_speed"].append(float(ref.get("target_speed", np.nan)))
        hist["target_yaw"].append(float(ref.get("target_yaw", eta[5])))
        hist["target_yaw_rate"].append(float(ref.get("target_yaw_rate", 0.0)))
        hist["target_x"].append(float(ref.get("target_x", np.nan)))
        hist["target_y"].append(float(ref.get("target_y", np.nan)))
        hist["waypoint_index"].append(float(ref.get("waypoint_index", task.extra_log().get("waypoint_index", 0.0))))
        hist["desired_fx"].append(float(out.desired_wrench[0]))
        hist["desired_fz"].append(float(out.desired_wrench[2]))
        hist["desired_mx"].append(float(out.desired_wrench[3]))
        hist["desired_my"].append(float(out.desired_wrench[4]))
        hist["desired_mz"].append(float(out.desired_wrench[5]))
        hist["alloc_error_norm"].append(float(np.linalg.norm(out.allocation.allocation_error)))
        for name, cmd in zip([t.name for t in params.thrusters], out.command):
            hist[f"cmd_{name}"].append(float(cmd))

        robot.step(out.command)

    hist_np = {key: np.asarray(val, dtype=float) for key, val in hist.items()}
    metrics = task.metrics(hist_np)
    result = TaskResult(task_name=task.name, history=hist_np, metrics=metrics)

    if output_dir is not None:
        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        _write_csv(out_dir / f"{task.name}.csv", hist_np)
        (out_dir / f"{task.name}_metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
        result.output_dir = out_dir
        if make_plots:
            from .plotting import plot_task_result
            plot_task_result(result, out_dir)

    return result


def run_all_tasks(output_dir: str | Path = "outputs/latest", dt: float = 0.02, make_plots: bool = True) -> list[TaskResult]:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = Path(output_dir)
    if str(output_dir).endswith("latest"):
        base = Path(output_dir)
    else:
        base = Path(output_dir) / stamp
    base.mkdir(parents=True, exist_ok=True)
    results = []
    for name in ["depth_forward", "turn_90deg", "rectangle_track"]:
        results.append(run_task(name, output_dir=base, dt=dt, make_plots=make_plots))
    summary = {r.task_name: r.metrics for r in results}
    (base / "summary_metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return results


def _write_csv(path: Path, hist: dict[str, np.ndarray]) -> None:
    keys = list(hist.keys())
    rows = len(hist[keys[0]])
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        for i in range(rows):
            writer.writerow([hist[k][i] for k in keys])
