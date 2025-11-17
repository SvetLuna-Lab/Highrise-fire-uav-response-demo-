import json
import os
import sys
import subprocess

def test_run_tethered_smoke():
   
    cmd = [
        sys.executable, "-m", "src.run_scenario",
        "--config", "configs/tethered_case.yaml",
        "--scenario", "data/scenarios/case_small.yaml",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    assert res.returncode == 0, res.stderr

    assert os.path.exists("reports/summary.json")
    with open("reports/summary.json", "r", encoding="utf-8") as f:
        data = json.load(f)

    
    for k in ["time_on_target_s", "ir_over_limit_s", "tension_N_peak", "min_bend_radius_m"]:
        assert k in data
