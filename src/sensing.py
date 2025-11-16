from __future__ import annotations
from typing import List, Tuple

def detect_fires_in_range(uav_xy: Tuple[int, int], fires_xy: List[Tuple[int, int]], radius_cells: int = 1) -> List[int]:
    """Return indices of fires within Chebyshev distance 'radius_cells' (very simple stub)."""
    ux, uy = uav_xy
    hits = []
    for idx, (fx, fy) in enumerate(fires_xy):
        if max(abs(ux - fx), abs(uy - fy)) <= radius_cells:
            hits.append(idx)
    return hits
