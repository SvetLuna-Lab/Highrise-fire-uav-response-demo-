from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple
from .dynamics import Kinematics

@dataclass
class UAV:
    uid: str
    kin: Kinematics
    batt_pct: float = 100.0
    tank_liters: float = 0.0
    path: List[Tuple[int, int]] = field(default_factory=list)
    path_idx: int = 0
    state: str = "idle"  # idle|enroute|suppression|rtl

    def step_path(self, dt: float, v_cruise: float, v_max: float) -> None:
        # Path following is handled in sim_loop (controller) for clarity.
        pass
