from __future__ import annotations

def suppression_effect(flow_lps: float, dt: float, intensity: float) -> float:
    """
    Proxy temperature drop contribution for a single step.
    More flow and time → more effect; clamp by current intensity.
    """
    base = flow_lps * dt * 0.02  # scaling factor (demo)
    return min(base, intensity)
