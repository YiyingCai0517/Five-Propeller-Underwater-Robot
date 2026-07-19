#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from underwater_5thruster_sim.tasks import run_task


def main() -> None:
    checks = []
    for task_name, duration in [("depth_forward", 8.0), ("turn_90deg", 10.0), ("rectangle_track", 12.0)]:
        result = run_task(task_name, output_dir=None, duration=duration, make_plots=False)
        state_keys = ["x", "y", "z", "roll", "pitch", "yaw", "u", "v", "w", "p", "q", "r"]
        finite = all(np.isfinite(result.history[k]).all() for k in state_keys)
        checks.append(finite)
        print(f"{task_name}: finite={finite}, depth_rmse={result.metrics.get('depth_rmse_m'):.4f}")
    if not all(checks):
        raise SystemExit("Smoke test failed: non-finite state detected")
    print("[OK] smoke test finished")


if __name__ == "__main__":
    main()
