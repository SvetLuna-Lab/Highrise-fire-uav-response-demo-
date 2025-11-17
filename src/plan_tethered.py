"""
Computes desired nozzle target near a facade seat with safe standoff and altitude.
"""
from __future__ import annotations
import numpy as np

def plan_nozzle_target(seat_pos: np.ndarray, facade_normal: np.ndarray,
                       standoff_m: float, base_alt_m: float) -> np.ndarray:
    n = facade_normal / (np.linalg.norm(facade_normal) + 1e-9)
    target = seat_pos - n * standoff_m
    # keep above minimum to avoid lintels / sills
    target[2] = max(target[2], base_alt_m + 1.5)
    return target
