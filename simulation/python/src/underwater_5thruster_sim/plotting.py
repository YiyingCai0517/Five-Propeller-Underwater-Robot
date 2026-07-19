from __future__ import annotations

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from .tasks import TaskResult


def plot_task_result(result: TaskResult, output_dir: str | Path) -> None:
    output_dir = Path(output_dir)
    h = result.history
    name = result.task_name

    plt.figure(figsize=(7, 5))
    plt.plot(h["x"], h["y"], label="actual")
    if np.isfinite(h["target_x"]).any():
        mask = np.isfinite(h["target_x"]) & np.isfinite(h["target_y"])
        plt.plot(h["target_x"][mask], h["target_y"][mask], linestyle="--", label="active waypoint")
    plt.axis("equal")
    plt.xlabel("world X / m")
    plt.ylabel("world Y / m")
    plt.title(f"{name}: horizontal trajectory")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / f"{name}_xy.png", dpi=160)
    plt.close()

    plt.figure(figsize=(7, 4))
    plt.plot(h["time"], h["depth"], label="depth")
    plt.plot(h["time"], h["target_depth"], linestyle="--", label="target")
    plt.xlabel("time / s")
    plt.ylabel("depth / m")
    plt.title(f"{name}: depth tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / f"{name}_depth.png", dpi=160)
    plt.close()

    plt.figure(figsize=(7, 4))
    plt.plot(h["time"], np.rad2deg(h["yaw"]), label="yaw")
    plt.plot(h["time"], np.rad2deg(h["target_yaw"]), linestyle="--", label="target")
    plt.xlabel("time / s")
    plt.ylabel("yaw / deg")
    plt.title(f"{name}: yaw tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / f"{name}_yaw.png", dpi=160)
    plt.close()

    plt.figure(figsize=(8, 5))
    for key in sorted(k for k in h if k.startswith("cmd_")):
        plt.plot(h["time"], h[key], label=key.replace("cmd_", ""))
    plt.xlabel("time / s")
    plt.ylabel("normalized command")
    plt.title(f"{name}: thruster commands")
    plt.ylim(-1.1, 1.1)
    plt.grid(True)
    plt.legend(ncol=2, fontsize=8)
    plt.tight_layout()
    plt.savefig(output_dir / f"{name}_thrusters.png", dpi=160)
    plt.close()
