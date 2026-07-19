from __future__ import annotations

import math
import numpy as np


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap radians to [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rotation_matrix_body_to_world(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler rotation matrix from body frame to world ENU frame."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def euler_rates_from_body_rates(roll: float, pitch: float, rates_body: np.ndarray) -> np.ndarray:
    """Convert body angular velocity [p, q, r] to ZYX Euler angle rates."""
    p, q, r = rates_body
    cr, sr = math.cos(roll), math.sin(roll)
    cp = max(1e-6, math.cos(pitch))
    tp = math.sin(pitch) / cp
    return np.array([
        p + sr * tp * q + cr * tp * r,
        cr * q - sr * r,
        sr / cp * q + cr / cp * r,
    ])


def clamp_vector(x: np.ndarray, low: float | np.ndarray, high: float | np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(x, low), high)


def smooth_step(current: np.ndarray, target: np.ndarray, dt: float, time_constant: float) -> np.ndarray:
    if time_constant <= 1e-9:
        return target.copy()
    alpha = np.clip(dt / time_constant, 0.0, 1.0)
    return current + alpha * (target - current)
