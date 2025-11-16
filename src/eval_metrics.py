from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List

@dataclass
class Summary:
    response_time_s: float
    coverage_pct: float
    temp_drop_proxy: float
    safety_score: float
    mission_score: float

def mission_score(coverage: float, temp_drop: float, safety_penalty: float) -> float:
    """
    Weighted score in [0,1] approx:
    - coverage, temp_drop: higher is better
    - safety_penalty: accumulated normalized violations (0 best)
    """
    cov = max(0.0, min(1.0, coverage))
    tmp = max(0.0, min(1.0, temp_drop))
    pen = max(0.0, min(1.0, safety_penalty))
    return 0.4 * cov + 0.3 * tmp - 0.3 * pen

def summarize(log: List[Dict]) -> Summary:
    if not log:
        return Summary(0.0, 0.0, 0.0, 0.0, 0.0)

    t_first = min((row["t"] for row in log if row.get("event") == "first_arrival"), default=None)
    response_time = float(t_first) if t_first is not None else 0.0

    # Coverage proxy: fraction of fires that reached intensity 0 at least once
    fires = {}
    for row in log:
        if "fire_id" in row and "fire_intensity" in row:
            fid = row["fire_id"]
            fires.setdefault(fid, 1.0)
            fires[fid] = min(fires[fid], row["fire_intensity"])
    covered = sum(1 for v in fires.values() if v <= 0.0)
    coverage = (covered / len(fires)) if fires else 0.0

    temp_drop = sum(row.get("temp_drop", 0.0) for row in log)
    safety_penalty = sum(row.get("safety_violation", 0.0) for row in log)

    score = mission_score(coverage, temp_drop, safety_penalty)
    return Summary(response_time, coverage, temp_drop, safety_penalty, score)
