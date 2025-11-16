from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List

import yaml
from .env import HighriseEnv
from .agents import UAV
from .dynamics import Kinematics
from .sim_loop import Simulator, SimConfig
from .safety import SafetyLimits
from .eval_metrics import summarize
from .visualize import plot_paths

def _load_yaml(path: str) -> Dict[str, Any]:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))

def _write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8"); return
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, type=str)
    ap.add_argument("--scenario", default=None, type=str, help="Override scenario file (yaml).")
    args = ap.parse_args()

    cfg = _load_yaml(args.config)
    sim_cfg = cfg["sim"]
    building = cfg["building"]["spec"]
    wind = cfg["env"]["wind_profile"]
    report_dir = Path(cfg["report"]["out_dir"])

    # scenario
    scen_path = args.scenario or "data/scenarios/case_small.yaml"
    scenario = _load_yaml(scen_path)

    env = HighriseEnv(building_json=building, wind_yaml=wind)
    env.set_fires(scenario["fires"])

    # UAV init
    uavs = []
    for i, ustart in enumerate(scenario["uav_starts"]):
        kin = Kinematics(x=float(ustart["x"]), y=float(ustart["y"]), vx=0.0, vy=0.0)
        uavs.append(UAV(uid=ustart["id"], kin=kin, batt_pct=100.0, tank_liters=float(cfg["uav"]["tank_liters"])))

    limits = SafetyLimits(
        geofence_margin_m=float(cfg["safety"]["geofence_margin_m"]),
        gust_limit_ms=float(cfg["safety"]["gust_limit_ms"]),
        min_batt_pct=float(cfg["safety"]["min_batt_pct"])
    )

    sim = Simulator(
        env=env,
        uavs=uavs,
        fires={f["id"]: (int(f["x"]), int(f["y"]), float(f.get("intensity", 1.0))) for f in scenario["fires"]},
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

    sim.plan_initial_paths()

    # loop
    while sim.step():
        pass

    # logs → files
    # mission log: minimal fields we used
    log_rows: List[Dict[str, Any]] = sim.log
    _write_csv(report_dir / "mission_log.csv", log_rows)

    # summary
    summ = summarize(log_rows)
    (report_dir / "summary.json").write_text(
        json.dumps({
            "response_time_s": summ.response_time_s,
            "coverage_pct": summ.coverage_pct,
            "temp_drop_proxy": summ.temp_drop_proxy,
            "safety_score": summ.safety_score,
            "mission_score": summ.mission_score
        }, indent=2),
        encoding="utf-8"
    )

    # paths plot
    fires_xy = [(f["x"], f["y"]) for f in scenario["fires"]]
    plot_paths(sim.collect_paths(), fires_xy, str(report_dir / "paths.png"))

    print("=== Highrise UAV fire response demo complete ===")
    print(f"Logs:     {report_dir / 'mission_log.csv'}")
    print(f"Summary:  {report_dir / 'summary.json'}")
    print(f"Plot:     {report_dir / 'paths.png'}")

if __name__ == "__main__":
    main()
