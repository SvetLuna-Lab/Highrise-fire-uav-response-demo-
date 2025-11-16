from __future__ import annotations
from dataclasses import dataclass

@dataclass
class Kinematics:
    x: float
    y: float
    vx: float
    vy: float

def integrate_step(kin: Kinematics, ax: float, ay: float, dt: float, v_max: float) -> Kinematics:
    """Simple Euler integration with speed clamp."""
    vx = kin.vx + ax * dt
    vy = kin.vy + ay * dt
    speed = (vx * vx + vy * vy) ** 0.5
    if speed > v_max:
        scale = v_max / (speed + 1e-9)
        vx *= scale
        vy *= scale
    x = kin.x + vx * dt
    y = kin.y + vy * dt
    return Kinematics(x=x, y=y, vx=vx, vy=vy)
