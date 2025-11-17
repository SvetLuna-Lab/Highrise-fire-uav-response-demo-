"""
Tethered nozzle model held by a UAV; water is supplied from a roof pump via hose.
Computes jet efficiency, suppression proxy, and IR exposure flags.
"""
from __future__ import annotations
import math
import numpy as np

def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / max(n, 1e-9)

class TetheredNozzle:
    def __init__(self, flow_lps: float, pressure_bar: float, standoff_m: float,
                 ir_flux_max_wpm2: float, therm_shield: bool = True) -> None:
        self.flow_lps = flow_lps
        self.pressure_bar = pressure_bar
        self.standoff_m = standoff_m
        self.ir_limit = ir_flux_max_wpm2
        self.therm_shield = therm_shield
        self.time_on_target = 0.0
        self.time_over_ir = 0.0

    @staticmethod
    def _jet_efficiency(standoff: float, wind_mps: float, plume_w_m: float) -> float:
        # Bell curve around ~2.2 m; penalty for wind & plume width.
        opt, w = 2.2, 1.2
        geom = math.exp(-((standoff - opt) ** 2) / (2 * w * w))
        pen = 1.0 / (1.0 + 0.08 * (wind_mps + 0.7 * plume_w_m) ** 2)
        return float(max(0.0, min(1.0, geom * pen)))

    def step(self, dt: float, nozzle_pos: np.ndarray, seat_pos: np.ndarray,
             wind_mps: float, plume_w_m: float, ir_flux_wpm2: float) -> dict:
        dist = np.linalg.norm(nozzle_pos - seat_pos)
        eff  = self._jet_efficiency(self.standoff_m, wind_mps, plume_w_m)

        # Angle penalty: favor spraying roughly along facade normal
        aim = _unit(seat_pos - nozzle_pos)
        facade_n = np.array([1.0, 0.0, 0.0])          # simplify: +X normal
        ang_pen = abs(float(np.dot(aim, facade_n))) ** 0.5  # [0..1]

        base = (self.flow_lps / 8.0) * (self.pressure_bar / 6.0)
        suppression = base * eff * ang_pen * (1.0 / (1.0 + 0.05 * (dist - self.standoff_m) ** 2))
        suppression = float(max(0.0, min(1.0, suppression)))

        limit = 1.5 * self.ir_limit if self.therm_shield else self.ir_limit
        over_ir = ir_flux_wpm2 > limit

        if suppression > 0.25:
            self.time_on_target += dt
        if over_ir:
            self.time_over_ir += dt

        return {"eff": eff, "suppression": suppression, "over_ir": over_ir}
