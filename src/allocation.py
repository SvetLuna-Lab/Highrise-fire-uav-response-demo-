from __future__ import annotations
from typing import List, Tuple
import numpy as np
from scipy.optimize import linear_sum_assignment

def assign_tasks(etamat: List[List[float]]) -> List[Tuple[int, int]]:
    """
    Hungarian assignment on ETA matrix (UAV x Fire). Returns list of (uav_idx, fire_idx).
    Any inf rows/cols will be skipped by the solver.
    """
    C = np.array(etamat, dtype=float)
    row_ind, col_ind = linear_sum_assignment(C)
    return list(zip(row_ind.tolist(), col_ind.tolist()))
