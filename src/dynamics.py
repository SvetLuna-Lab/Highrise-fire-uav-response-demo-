# src/dynamics.py
from __future__ import annotations

from dataclasses import dataclass
from math import hypot

@dataclass
class Kinematics:
    """Minimal 2D kinematics state used across the simulator."""
    x: float
    y: float
    vx: float
    vy: float

def clamp_speed(vx: float, vy: float, v_max: float) -> tuple[float, float]:
    """Clamp velocity vector to the max speed while preserving direction."""
    speed = hypot(vx, vy)
    if v_max <= 0.0 or speed <= v_max:
        return vx, vy
    k = v_max / max(speed, 1e-9)
    return vx * k, vy * k

def integrate_euler(kin: Kinematics, ax: float, ay: float, dt: float, v_max: float) -> None:
    """One Euler step with simple speed clamp."""
    # update velocity
    kin.vx += ax * dt
    kin.vy += ay * dt
    kin.vx, kin.vy = clamp_speed(kin.vx, kin.vy, v_max)

    # update position
    kin.x += kin.vx * dt
    kin.y += kin.vy * dt
