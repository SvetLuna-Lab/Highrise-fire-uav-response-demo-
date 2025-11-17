import json
import os
from pathlib import Path
import pytest

from src.sim_loop import Simulator, SimConfig

class DummyEnv:
    pass

class DummyUAV:
    def __init__(self, uid):
       
        from types import SimpleNamespace
        self.uid = uid
        self.kin = SimpleNamespace(x=0.0, y=0.0, vx=0.0, vy=0.0)

@pytest.fixture
def sim_basic(tmp_path):
    cfg = SimConfig(
        dt=0.1, t_max=0.5,
        v_cruise=5.0, v_max=10.0,
        flow_lps=1.0, limits=None,
        wind_penalty=0.0
    )
    fires = {"f1": (0, 0, 1.0)}
    sim = Simulator(env=DummyEnv(), uavs=[DummyUAV("u1")], fires=fires, cfg=cfg)
    return sim

def test_metrics_summary_disabled(sim_basic):
   
    m = sim_basic.metrics_summary()
    assert set(m.keys()) == {"time_on_target_s","ir_over_limit_s","tension_N_peak","min_bend_radius_m"}
    assert m["time_on_target_s"] == 0.0

def test_metrics_summary_enabled(sim_basic):
    sim_basic.enable_tether(hose_length_m=120.0, base_xy=(0.0, 0.0))
 
    while sim_basic.step():
        pass
    m = sim_basic.metrics_summary()
    assert m["tension_N_peak"] >= 0.0
  
    assert ("min_bend_radius_m" in m)
