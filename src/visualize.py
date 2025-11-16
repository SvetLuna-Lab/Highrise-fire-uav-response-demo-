from __future__ import annotations
from pathlib import Path
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt

def plot_paths(paths: Dict[str, List[Tuple[int, int]]], fires: List[Tuple[int, int]], out_png: str) -> None:
    """
    Simple scatter plot of UAV paths and fire locations.
    """
    plt.figure()
    for uid, p in paths.items():
        xs = [x for x, _ in p]
        ys = [y for _, y in p]
        plt.plot(xs, ys, label=uid)
    if fires:
        fx = [x for x, _ in fires]
        fy = [y for _, y in fires]
        plt.scatter(fx, fy, marker="x")
    plt.xlabel("x (cells)")
    plt.ylabel("y (floors)")
    plt.legend()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close()
