#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from underwater_5thruster_sim.tasks import run_all_tasks


def main() -> None:
    parser = argparse.ArgumentParser(description="Run all three underwater robot demo tasks.")
    parser.add_argument("--output", default="outputs/latest", help="Output directory for CSV/PNG/JSON files.")
    parser.add_argument("--dt", type=float, default=0.02, help="Simulation step in seconds.")
    parser.add_argument("--no-plots", action="store_true", help="Skip matplotlib figures.")
    args = parser.parse_args()

    results = run_all_tasks(output_dir=args.output, dt=args.dt, make_plots=not args.no_plots)
    print("\nSimulation finished. Metrics:")
    for result in results:
        print(f"\n[{result.task_name}]")
        for key, value in result.metrics.items():
            print(f"  {key}: {value}")
    print(f"\nOutputs saved to: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
