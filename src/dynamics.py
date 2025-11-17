"""
Quasi-static segmented hose under gravity and wind drag.
Returns hose points, peak tension, and minimal bend radius.
"""
from __future__ import annotations
import numpy as np

def integrate_hose(p_anchor: np.ndarray, p_nozzle: np.ndarray, wind_vec: np.ndarray,
                   length_m: float, linear_mass_kgpm: float, diameter_m: float,
                   damping: float, segs: int = 8, iters: int = 20) -> dict:
    g = 9.80665
    n = max(2, int(segs))
    pts = np.linspace(p_anchor, p_nozzle, n + 1)   # initial guess

    seg_len = length_m / n
    mass_seg = linear_mass_kgpm * seg_len
    A = np.pi * (0.5 * diameter_m) ** 2
    rho_air, Cd = 1.225, 1.1

    for _ in range(iters):
        # internal relaxation with gravity and wind drag
        for i in range(1, n):
            Fg = np.array([0.0, 0.0, -mass_seg * g])
            v_rel = wind_vec
            Fd = 0.5 * rho_air * Cd * A * np.linalg.norm(v_rel) * v_rel
            F = Fg + Fd
            pts[i] += 0.35 * (F / (mass_seg * g))

        # enforce segment length
        for i in range(n):
            d = pts[i + 1] - pts[i]
            Ld = np.linalg.norm(d) + 1e-9
            corr = (Ld - seg_len) / Ld * 0.5
            pts[i]     +=  corr * d
            pts[i + 1] -=  corr * d

        pts[0]  = p_anchor
        pts[-1] = p_nozzle

    # peak tension (weight + distributed drag proxy)
    tensions, cumulative = [], 0.0
    for _ in range(n, 0, -1):
        cumulative += mass_seg * g
        cumulative += 0.5 * rho_air * Cd * A * np.linalg.norm(wind_vec) ** 2 * (seg_len / length_m)
        tensions.append(cumulative)
    peak_tension = max(tensions) if tensions else 0.0

    # curvature-based bend radius proxy
    radii = []
    for i in range(1, n):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        ab = b - a; cb = b - c
        cosang = np.clip(np.dot(ab, cb) / ((np.linalg.norm(ab) * np.linalg.norm(cb)) + 1e-9), -1.0, 1.0)
        ang = np.arccos(cosang)
        kappa = abs(ang) / (np.linalg.norm(ab) + 1e-9)
        r = 1.0 / max(kappa, 1e-6)
        radii.append(r)
    min_r = float(min(radii)) if radii else 1e9

    return {"pts": pts, "peak_tension_N": float(peak_tension), "min_bend_radius_m": float(min_r)}
