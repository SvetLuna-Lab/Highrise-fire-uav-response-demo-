from __future__ import annotations
from typing import Tuple
from .dynamics import Kinematics, integrate_step

def track_waypoint(kin: Kinematics, wp: Tuple[float, float], v_cruise: float, dt: float, v_max: float) -> Kinematics:
    """Point mass controller: accelerate toward waypoint until cruise speed."""
    wx, wy = wp
    dx = wx - kin.x; dy = wy - kin.y
    dist = (dx*dx + dy*dy) ** 0.5 + 1e-9
    ux, uy = dx / dist, dy / dist
    # crude acceleration to approach cruise velocity
    desired_vx, desired_vy = ux * v_cruise, uy * v_cruise
    ax = (desired_vx - kin.vx) / max(dt, 1e-3)
    ay = (desired_vy - kin.vy) / max(dt, 1e-3)
    return integrate_step(kin, ax, ay, dt, v_max)
