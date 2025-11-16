from __future__ import annotations
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from .env import HighriseEnv
from .agents import UAV
from .dynamics import Kinematics
from .planner import astar
from .controller import track_waypoint
from .safety import SafetyLimits, check_geofence, should_rtl, gust_exceeded
from .sensing import detect_fires_in_range
from .suppression import suppression_effect

@dataclass
class SimConfig:
    dt: float
    t_max: float
    v_cruise: float
    v_max: float
    flow_lps: float
    limits: SafetyLimits
    wind_penalty: float

class Simulator:
    """
    Single-threaded discrete-time loop:
    - plan once at t=0 (demo) and follow
    - apply basic safety checks and suppression
    - log per-step data
    """
    def __init__(self, env: HighriseEnv, uavs: List[UAV], fires: Dict[str, Tuple[int,int,float]], cfg: SimConfig) -> None:
        self.env = env
        self.uavs = uavs
        self.fires = fires  # fid -> (x,y,intensity)
        self.cfg = cfg
        self.t = 0.0
        self.log: List[Dict] = []
        self._planned_paths: Dict[str, List[Tuple[int,int]]] = {}

    def _wind_cost(self, x: int, y: int) -> float:
        w = self.env.wind_ms_at_cell(x, y)
        return self.cfg.wind_penalty * w

    def plan_initial_paths(self) -> None:
        W, H = self.env.grid.width, self.env.grid.height
        # naive: assign fires in order to UAVs by closest start (could call Hungarian on ETA matrix)
        fire_targets = list(self.fires.items())
        for i, u in enumerate(self.uavs):
            if i < len(fire_targets):
                fid, (fx, fy, _) = fire_targets[i]
                path = astar((int(u.kin.x), int(u.kin.y)), (fx, fy), self.env.is_blocked, self._wind_cost, W, H)
                self._planned_paths[u.uid] = path
                u.path = path
                u.state = "enroute"

    def step(self) -> bool:
        dt = self.cfg.dt
        self.t += dt

        # simple gust model
        gust = random.gauss(0.0, 1.0) * 0.0  # set to 0 for reproducibility; or cfg.gust_sigma

        # update each UAV
        for u in self.uavs:
            # safety checks
            outside = check_geofence(u.kin.x, u.kin.y, self.env.grid.width, self.env.grid.height, self.cfg.limits.geofence_margin_m)
            if outside or should_rtl(u.batt_pct, self.cfg.limits) or gust_exceeded(abs(gust), self.cfg.limits):
                u.state = "rtl"

            # follow path or RTL to base (0,y=0)
            if u.state in ("enroute", "suppression"):
                if u.path and u.path_idx < len(u.path):
                    wx, wy = u.path[u.path_idx]
                    u.kin = track_waypoint(u.kin, (wx, wy), self.cfg.v_cruise, dt, self.cfg.v_max)
                    if abs(u.kin.x - wx) < 0.5 and abs(u.kin.y - wy) < 0.5:
                        u.path_idx += 1
                else:
                    u.state = "suppression"

            elif u.state == "rtl":
                if not u.path or u.path_idx >= len(u.path):
                    u.path = [(int(u.kin.x), int(u.kin.y))]
                    u.path += [(int(u.kin.x), 0)]
                    u.path_idx = 0
                wx, wy = u.path[u.path_idx]
                u.kin = track_waypoint(u.kin, (wx, wy), self.cfg.v_cruise, dt, self.cfg.v_max)
                if abs(u.kin.x - wx) < 0.5 and abs(u.kin.y - wy) < 0.5:
                    u.path_idx += 1

            # battery drain (demo)
            u.batt_pct = max(0.0, u.batt_pct - 0.02)

        # suppression step (if in range)
        fire_list = [(fx, fy) for (_, (fx, fy, _)) in self.fires.items()]
        for u in self.uavs:
            if u.state == "suppression":
                hits = detect_fires_in_range((int(u.kin.x), int(u.kin.y)), fire_list, radius_cells=0)
                for idx in hits:
                    fid = list(self.fires.keys())[idx]
                    fx, fy, intensity = self.fires[fid]
                    drop = suppression_effect(self.cfg.flow_lps, dt, intensity)
                    self.fires[fid] = (fx, fy, max(0.0, intensity - drop))
                    self.log.append({"t": self.t, "event": "suppression", "uav_id": u.uid, "fire_id": fid,
                                     "temp_drop": drop, "fire_intensity": self.fires[fid][2]})

        # first arrival event
        for u in self.uavs:
            if u.state == "suppression":
                self.log.append({"t": self.t, "event": "first_arrival", "uav_id": u.uid})
                break

        # termination: all fires out or time limit
        if all(inten <= 0.0 for (_, (_, _, inten)) in self.fires.items()):
            return False
        return self.t < self.cfg.t_max

    def collect_paths(self) -> Dict[str, List[tuple]]:
        return {u.uid: u.path for u in self.uavs}
