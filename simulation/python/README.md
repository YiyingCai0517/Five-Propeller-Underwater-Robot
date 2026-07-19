# Python 6-DOF simulation

This module provides a standalone numerical model for algorithm study and repeatable demonstrations. It includes five thrust vectors, simplified 6-DOF dynamics, linear/quadratic damping, restoring terms, actuator lag, task controllers, and constrained weighted least-squares allocation.

## Install and test

From this directory:

```bash
python -m venv .venv
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python scripts/smoke_test.py
```

## Run all examples

```bash
python run_all_tasks.py
```

Skip plots for a faster headless run:

```bash
python run_all_tasks.py --no-plots --output outputs/latest
```

Run a single task:

```bash
python scripts/run_task.py depth_forward
python scripts/run_task.py turn_90deg
python scripts/run_task.py rectangle_track
```

Generated CSV, JSON, and PNG files are written under `outputs/` and ignored by Git.

## Source map

| File | Purpose |
| --- | --- |
| `src/underwater_5thruster_sim/dynamics.py` | Rigid-body integration, damping, restoring force, actuator lag |
| `controllers.py` | Depth, surge, roll, pitch, and yaw control laws |
| `allocation.py` | Weighted least-squares actuator allocation with clipping |
| `robot_params.py` | Coordinate transform, mass assumptions, damping, thruster geometry |
| `tasks.py` | Forward, turn, and rectangular waypoint tasks |
| `plotting.py` | Result figures |

## Model limitations

- Mass, inertia, added mass, damping, restoring stiffness, and actuator time constant are provisional values derived from CAD interpretation and engineering assumptions.
- Thruster output is a symmetric command-to-force model, not a measured PWM/RPM/thrust curve.
- The model omits free-surface effects, tether forces, radio behavior, waves, turbulence, leakage, battery voltage sag, detailed sensor noise, and controller timing jitter.
- Very small simulated depth RMSE values demonstrate consistency of the assumed model and controller; they are not physical accuracy claims.
- The simulator control law and allocator are related to, but not identical to, the STM32 PID and analytic mixer.

Calibrate the model with tank data before using it to select real-world gains or safety limits.
