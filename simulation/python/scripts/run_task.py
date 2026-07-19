#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from underwater_5thruster_sim.tasks import run_task


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one five-thruster underwater robot task.")
    parser.add_argument("task", choices=["depth_forward", "turn_90deg", "rectangle_track", "forward", "turn", "rectangle"])
    parser.add_argument("--output", default="outputs/latest")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    result = run_task(args.task, output_dir=args.output, dt=args.dt, duration=args.duration, make_plots=not args.no_plots)
    print(f"[{result.task_name}] metrics")
    for k, v in result.metrics.items():
        print(f"  {k}: {v}")
    print(f"Outputs saved to: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
