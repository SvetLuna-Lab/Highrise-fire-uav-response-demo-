# src/sim_loop.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

# NOTE: import your own modules here (env, agents, safety, etc.)
# from .env import HighriseEnv
# from .agents import UAV
# from .safety import SafetyLimits
# …


@dataclass
class SimConfig:
    """Minimal simulation configuration."""

    dt: float                  # integration step [s]
    t_max: float               # horizon [s]
    v_cruise: float            # nominal speed [m/s]
    v_max: float               # absolute speed limit [m/s]
    flow_lps: float            # water flow [L/s]
    limits: Any                # safety limits object (opaque here)
    wind_penalty: float        # planning penalty for wind


class Simulator:
    """Lightweight simulator shell.

    - Keeps environment, UAV list and active fires.
    - Exposes a `step()` loop.
    - Accumulates a simple log and path traces per UAV.
    - Contains an *optional* tethered-nozzle / hose hook that can be enabled
      at runtime (does nothing unless explicitly turned on).
    """

    def __init__(
        self,
        env,
        uavs,
        fires: Dict[str, Tuple[int, int, float]],
        cfg: SimConfig,
    ) -> None:
        self.env = env
        self.uavs = uavs
        self.fires = fires
        self.cfg = cfg

        self.t: float = 0.0
        self.log: List[Dict[str, Any]] = []  # per-step events (filled by your logic)

        # --- Tethered mode state (disabled by default) -----------------------
        self._tether_enabled: bool = False
        self._tether_base_xy: Tuple[float, float] = (0.0, 0.0)
        self._hose_length_m: float = 0.0

        # Accumulators for tethered metrics (very coarse proxies for now)
        self._time_on_target_s: float = 0.0        # total time when stream hits target area
        self._time_over_ir_s: float = 0.0          # “IR over limit” proxy (fire intensity present)
        self._peak_tension_N: float = 0.0          # peak hose tension
        self._min_bend_radius_m: float = float("+inf")  # minimum bend radius along the path

        # Path traces per UAV (for plotting/analysis)
        self._paths: Dict[str, List[Tuple[float, float]]] = {u.uid: [] for u in self.uavs}

    # --------------------------------------------------------------------- #
    # Public hook to enable tethered mode from run_scenario.py
    def enable_tether(self, hose_length_m: float, base_xy: Tuple[float, float]) -> None:
        """Enable tethered-nozzle mode and set base point + hose length."""
        self._tether_enabled = True
        self._hose_length_m = float(hose_length_m)
        self._tether_base_xy = (float(base_xy[0]), float(base_xy[1]))

    # --------------------------------------------------------------------- #
    def plan_initial_paths(self) -> None:
        """Initial planning stub. Keep your existing logic here."""
        pass

    # --------------------------------------------------------------------- #
    def _near_fire(self, x: float, y: float, tol_m: float = 2.0) -> bool:
        """Return True if (x, y) is within tol_m of any fire centroid."""
        for _, (fx, fy, _intensity) in self.fires.items():
            dx = x - fx
            dy = y - fy
            if dx * dx + dy * dy <= tol_m * tol_m:
                return True
        return False

    # --------------------------------------------------------------------- #
    def _update_tether_metrics(self, dt: float) -> None:
        """Very simple surrogate metrics for the tethered mode.

        Replace with physically grounded models when they are ready:
        - time on target is detected by proximity to a fire,
        - IR over limit is proxied by “any fire with intensity > 0.5”,
        - hose tension ~ a*speed + b*distance to base (clamped),
        - bend radius is approximated from discrete path curvature.
        """
        # “IR over limit” proxy: any active/intense fire present
        ir_over_limit = any(intensity > 0.5 for _, (_, _, intensity) in self.fires.items())

        on_target = False
        max_tension = 0.0
        min_bend = float("+inf")

        for u in self.uavs:
            x, y = u.kin.x, u.kin.y
            self._paths[u.uid].append((x, y))

            if self._near_fire(x, y, tol_m=2.0):
                on_target = True

            # Very coarse tension estimate:
            # T ≈ a * |v| + b * dist(base), clamped from above
            dist_base = ((x - self._tether_base_xy[0]) ** 2 + (y - self._tether_base_xy[1]) ** 2) ** 0.5
            v_mod = (u.kin.vx ** 2 + u.kin.vy ** 2) ** 0.5
            T = min(2000.0, 12.0 * v_mod + 3.0 * dist_base)
            max_tension = max(max_tension, T)

            # Bend radius proxy from discrete curvature (angle change)
            if len(self._paths[u.uid]) >= 3:
                import math

                x0, y0 = self._paths[u.uid][-3]
                x1, y1 = self._paths[u.uid][-2]
                x2, y2 = self._paths[u.uid][-1]

                def angle(a, b, c) -> float:
                    ax, ay = a[0] - b[0], a[1] - b[1]
                    cx, cy = c[0] - b[0], c[1] - b[1]
                    na, nc = math.hypot(ax, ay), math.hypot(cx, cy)
                    if na == 0 or nc == 0:
                        return 0.0
                    cosv = max(-1.0, min(1.0, (ax * cx + ay * cy) / (na * nc)))
                    return math.acos(cosv)

                phi = angle((x0, y0), (x1, y1), (x2, y2))
                # R ~ v / ω; here v ~ |v|, ω ~ φ/dt  →  R ~ |v| * dt / max(φ, ε)
                R = float("+inf") if phi < 1e-3 else (v_mod * self.cfg.dt / phi)
                min_bend = min(min_bend, R)

        if on_target:
            self._time_on_target_s += dt
        if ir_over_limit:
            self._time_over_ir_s += dt

        self._peak_tension_N = max(self._peak_tension_N, max_tension)
        if min_bend < self._min_bend_radius_m:
            self._min_bend_radius_m = min_bend

    # --------------------------------------------------------------------- #
    def step(self) -> bool:
        """Advance the simulation by one time step.

        Returns:
            bool: True if we are still within horizon, False otherwise.
        """
        if self.t >= self.cfg.t_max:
            return False

        dt = self.cfg.dt

        # >>> Your core dynamics/planning/safety logic goes here <<<
        # - integrate UAV kinematics
        # - update environment effects
        # - perform planning/re-planning
        # - append per-step events into `self.log`
        # -----------------------------------------------------------

        # Tethered-mode hook
        if self._tether_enabled:
            self._update_tether_metrics(dt)

        self.t += dt
        return self.t < self.cfg.t_max

    # --------------------------------------------------------------------- #
    def collect_paths(self) -> Dict[str, List[Tuple[float, float]]]:
        """Return recorded paths per UAV."""
        return self._paths

    # --------------------------------------------------------------------- #
    def metrics_summary(self) -> Dict[str, float]:
        """Expose a stable set of tethered metrics for summary.json.

        Always return the same keys; when the mode is disabled,
        provide neutral values.
        """
        if not self._tether_enabled:
            return {
                "time_on_target_s": 0.0,
                "ir_over_limit_s": 0.0,
                "tension_N_peak": 0.0,
                "min_bend_radius_m": float("+inf"),
            }

        return {
            "time_on_target_s": round(self._time_on_target_s, 3),
            "ir_over_limit_s": round(self._time_over_ir_s, 3),
            "tension_N_peak": round(self._peak_tension_N, 2),
            "min_bend_radius_m": (
                float("+inf")
                if self._min_bend_radius_m == float("+inf")
                else round(self._min_bend_radius_m, 3)
            ),
        }

