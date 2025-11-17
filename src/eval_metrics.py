# src/eval_metrics.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, List, Set

@dataclass
class Summary:
    """Aggregate mission metrics used for README/CI smoke."""
    response_time_s: float
    coverage_pct: float
    temp_drop_proxy: float
    safety_score: float
    mission_score: float

def _get_t(row: Dict[str, Any]) -> float:
    t = row.get("t")
    try:
        return float(t) if t is not None else 0.0
    except Exception:
        return 0.0

def summarize(log_rows: List[Dict[str, Any]]) -> Summary:
    """
    Minimal, robust aggregator over per-step logs.
    Assumes rows may contain:
      - 't' (time, seconds)
      - 'on_target' (bool): stream hits a fire cell
      - 'hit_fire_id' (str): which fire is affected
      - safety flags: 'geofence_violation', 'gust_violation', 'low_batt' (bool)
      - 'spray_on' (bool): water is being sprayed
    All keys are optional; missing keys are treated as False/0.
    """
    if not log_rows:
        return Summary(0.0, 0.0, 0.0, 0.0, 0.0)

    # timeline
    t_first = min(_get_t(r) for r in log_rows)
    t_last  = max(_get_t(r) for r in log_rows)

    # response time: first moment we are "on target"; fallback to total time
    on_target_ts = [ _get_t(r) for r in log_rows if bool(r.get("on_target")) ]
    response_time_s = (min(on_target_ts) - t_first) if on_target_ts else (t_last - t_first)

    # coverage: fraction of distinct fires that were hit at least once
    hit_ids: Set[str] = set()
    for r in log_rows:
        fid = r.get("hit_fire_id")
        if isinstance(fid, str) and fid:
            hit_ids.add(fid)

    # If the simulator recorded a list of known fire ids in any row, use it.
    # Otherwise, approximate by the set we've actually hit (avoids division by zero).
    known_fires: Set[str] = set()
    for r in log_rows:
        kf = r.get("known_fire_ids")
        if isinstance(kf, list):
            known_fires |= {str(x) for x in kf if isinstance(x, (str, int))}
    denom = len(known_fires) if known_fires else max(1, len(hit_ids))
    coverage_pct = len(hit_ids) / denom if denom > 0 else 0.0

    # temperature drop proxy: number of steps with spray_on (acts as rough cooling surrogate)
    # You can replace with physical cooling integral later.
    temp_drop_proxy = float(sum(1 for r in log_rows if bool(r.get("spray_on"))))

    # safety score: normalize count of safety violations by number of rows (lower is better)
    n = max(1, len(log_rows))
    violations = 0
    for r in log_rows:
        violations += int(bool(r.get("geofence_violation")))
        violations += int(bool(r.get("gust_violation")))
        violations += int(bool(r.get("low_batt")))
    # convert to [0..1], where 1 means perfect safety (no violations)
    safety_score = max(0.0, 1.0 - violations / float(n))

    # overall mission score: simple weighted blend (tweak later if needed)
    # faster response, higher coverage, better safety, more cooling → higher score.
    # Response component uses a decay; clamp to [0,1].
    import math
    resp_comp = math.exp(-max(0.0, response_time_s) / 60.0)  # ~1 if <1min; decays afterwards
    cov_comp  = coverage_pct
    safe_comp = safety_score
    cool_comp = min(1.0, temp_drop_proxy / float(n))  # fraction of steps with spray

    mission_score = 0.35 * resp_comp + 0.35 * cov_comp + 0.2 * safe_comp + 0.1 * cool_comp
    return Summary(
        response_time_s=max(0.0, float(response_time_s)),
        coverage_pct=float(cov_comp),
        temp_drop_proxy=float(temp_drop_proxy),
        safety_score=float(safe_comp),
        mission_score=float(mission_score),
    )
