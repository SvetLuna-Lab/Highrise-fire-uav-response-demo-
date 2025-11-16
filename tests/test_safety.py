from src.safety import SafetyLimits, check_geofence, should_rtl, gust_exceeded

def test_safety_flags():
    limits = SafetyLimits(geofence_margin_m=1.0, gust_limit_ms=5.0, min_batt_pct=20.0)
    assert should_rtl(15.0, limits) is True
    assert gust_exceeded(6.0, limits) is True
    assert check_geofence(-1, 5, 10, 10, 1.0) is True
