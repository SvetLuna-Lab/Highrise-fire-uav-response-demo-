from __future__ import annotations
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

@dataclass
class Fire:
    fid: str
    x: int
    y: int
    intensity: float  # 0..1 proxy

@dataclass
class BuildingGrid:
    width: int
    height: int
    cell_size_m: float
    no_fly_mask: np.ndarray  # shape (H, W), True = forbidden

class WindField:
    """Altitude-dependent wind magnitude (m/s). Direction is assumed horizontal, +x for demo."""
    def __init__(self, profile: List[Tuple[float, float]]) -> None:
        self.profile = sorted(profile, key=lambda p: p[0])

    def at_alt(self, alt_m: float) -> float:
        # Piecewise-linear interpolation
        xs, ys = zip(*self.profile)
        return float(np.interp(alt_m, xs, ys, left=ys[0], right=ys[-1]))

class HighriseEnv:
    """
    2.5D facade environment:
    - Discrete grid: x (columns), y (rows = floors).
    - Fires are cells needing suppression.
    - Wind is altitude-dependent magnitude along +x (demo).
    """
    def __init__(self, building_json: str, wind_yaml: str) -> None:
        self.grid = self._load_building(building_json)
        self.wind = self._load_wind(wind_yaml)
        self.fires: Dict[str, Fire] = {}

    def _load_building(self, path: str) -> BuildingGrid:
        cfg = json.loads(Path(path).read_text(encoding="utf-8"))
        W = cfg["grid"]["width_cells"]
        H = cfg["grid"]["height_cells"]
        cell = cfg["grid"]["cell_size_m"]
        mask = np.zeros((H, W), dtype=bool)
        for z in cfg.get("no_fly_zones", []):
            x0, y0, x1, y1 = z["rect"]
            mask[y0:y1, x0:x1] = True
        return BuildingGrid(width=W, height=H, cell_size_m=cell, no_fly_mask=mask)

    def _load_wind(self, path: str) -> WindField:
        yml = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        prof = [(float(a), float(v)) for a, v in yml["profile"]]
        return WindField(prof)

    def set_fires(self, fires: List[Dict]) -> None:
        self.fires = {
            f["id"]: Fire(fid=f["id"], x=int(f["x"]), y=int(f["y"]), intensity=float(f.get("intensity", 1.0)))
            for f in fires
        }

    def wind_ms_at_cell(self, x: int, y: int) -> float:
        alt = y * self.grid.cell_size_m
        return self.wind.at_alt(alt)

    def is_blocked(self, x: int, y: int) -> bool:
        if x < 0 or y < 0 or x >= self.grid.width or y >= self.grid.height:
            return True
        return bool(self.grid.no_fly_mask[y, x])
