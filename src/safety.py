from __future__ import annotations
from dataclasses import dataclass

@dataclass
class SafetyLimits:
    geofence_margin_m: float
    gust_limit_ms: float
    min_batt_pct: float

def check_geofence(x: float, y: float, W: int, H: int, margin: float) -> bool:
    """True if outside safe bounds."""
    return not (margin <= x <= W - 1 - margin and margin <= y <= H - 1 - margin)

def should_rtl(batt_pct: float, limits: SafetyLimits) -> bool:
    return batt_pct <= limits.min_batt_pct

def gust_exceeded(gust_ms: float, limits: SafetyLimits) -> bool:
    return gust_ms > limits.gust_limit_ms
