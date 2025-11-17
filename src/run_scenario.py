from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Tuple, Optional

import yaml

from .env import HighriseEnv
from .agents import UAV
from .dynamics import Kinematics
from .sim_loop import Simulator, SimConfig
from .safety import SafetyLimits
from .eval_metrics import summarize
from .visualize import plot_paths


# ---------------------------
# I/O helpers
# ---------------------------

def _load_yaml(path: str | Path) -> Dict[str, Any]:
    """Load YAML file as dict (UTF-8)."""
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))

def _write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write list of dicts to CSV (creates parents)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


# ---------------------------
# Main entry
# ---------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Highrise UAV fire response simulator (facade ops)."
    )
    ap.add_argument("--config", required=True, type=str,
                    help="Path to YAML config (e.g., configs/default.yaml).")
    ap.add_argument("--scenario", default=None, type=str,
                    help="Override scenario YAML (e.g., data/scenarios/case_small.yaml).")
    args = ap.parse_args()

    # Load high-level config
    cfg = _load_yaml(args.config)
    sim_cfg = cfg["sim"]
    building = cfg["building"]["spec"]
    wind = cfg["env"]["wind_profile"]
    report_dir = Path(cfg["report"]["out_dir"])

    # Scenario (default or override)
    scen_path = args.scenario or "data/scenarios/case_small.yaml"
    scenario = _load_yaml(scen_path)

    # Environment
    env = HighriseEnv(building_json=building, wind_yaml=wind)
    env.set_fires(scenario["fires"])

    # UAVs
    uavs: List[UAV] = []
    for i, ustart in enumerate(scenario["uav_starts"]):
        kin = Kinematics(x=float(ustart["x"]), y=float(ustart["y"]), vx=0.0, vy=0.0)
        uavs.append(
            UAV(
                uid=ustart["id"],
                kin=kin,
                batt_pct=100.0,
                tank_liters=float(cfg["uav"]["tank_liters"]),
            )
        )

    # Safety limits
    limits = SafetyLimits(
        geofence_margin_m=float(cfg["safety"]["geofence_margin_m"]),
        gust_limit_ms=float(cfg["safety"]["gust_limit_ms"]),
        min_batt_pct=float(cfg["safety"]["min_batt_pct"]),
    )

    # Simulator config
    sim = Simulator(
        env=env,
        uavs=uavs,
        fires={f["id"]: (int(f["x"]), int(f["y"]), float(f.get("intensity", 1.0)))
               for f in scenario["fires"]},
        cfg=SimConfig(
            dt=float(sim_cfg["dt"]),
            t_max=float(sim_cfg["t_max"]),
            v_cruise=float(cfg["uav"]["v_cruise"]),
            v_max=float(cfg["uav"]["v_max"]),
            flow_lps=float(cfg["uav"]["flow_lps"]),
            limits=limits,
            wind_penalty=float(cfg["planner"]["wind_penalty"]),
        ),
    )

    # ---------------------------
    # Tethered nozzle / hose hook (safe no-op if not implemented yet)
    # Expected config keys (optional):
    #   tethered:
    #     enabled: true
    #     hose_length_m: 120.0
    #     base_xy: [x0, y0]      # anchor/valve location on the roof/facade
    # ---------------------------
    tether_cfg = cfg.get("tethered", {})
    if tether_cfg and bool(tether_cfg.get("enabled", False)):
        hose_len = float(tether_cfg.get("hose_length_m", 0.0))
        base_xy: Tuple[float, float] = tuple(tether_cfg.get("base_xy", [0.0, 0.0]))  # type: ignore
        try:
            # Your Simulator may provide an integration point like this:
            sim.enable_tether(hose_length_m=hose_len, base_xy=base_xy)
        except AttributeError:
            # Not wired yet — continue without failing.
            pass

    # Initial planning
    sim.plan_initial_paths()

    # Main loop
    while sim.step():
        pass

    # ---------------------------
    # Artifacts
    # ---------------------------

    # Mission log
    log_rows: List[Dict[str, Any]] = sim.log
    _write_csv(report_dir / "mission_log.csv", log_rows)

    # Summary JSON
    summ = summarize(log_rows)
    (report_dir / "summary.json").write_text(
        json.dumps(
            {
                "response_time_s": summ.response_time_s,
                "coverage_pct": summ.coverage_pct,
                "temp_drop_proxy": summ.temp_drop_proxy,
                "safety_score": summ.safety_score,
                "mission_score": summ.mission_score,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    # Paths plot
    fires_xy = [(f["x"], f["y"]) for f in scenario["fires"]]
    plot_paths(sim.collect_paths(), fires_xy, str(report_dir / "paths.png"))

    print("=== Highrise UAV fire response demo complete ===")
    print(f"Logs:     {report_dir / 'mission_log.csv'}")
    print(f"Summary:  {report_dir / 'summary.json'}")
    print(f"Plot:     {report_dir / 'paths.png'}")


if __name__ == "__main__":
    main()
