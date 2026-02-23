#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml


def load_params(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def sgn(v: float, eps: float = 1e-9) -> float:
    if v > eps:
        return 1.0
    if v < -eps:
        return -1.0
    return 0.0


def _sorted_strict(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(x)
    xs = x[order]
    ys = y[order]
    if np.any(np.diff(xs) <= 0.0):
        raise ValueError("LUT x-grid must be strictly monotonic.")
    return xs, ys


def _require_strict_increasing(x: np.ndarray, name: str) -> np.ndarray:
    xs = np.asarray(x, dtype=float)
    if np.any(np.diff(xs) <= 0.0):
        raise ValueError(f"{name} grid must be strictly increasing.")
    return xs


def _interp_monotonic(xq: float, xp: np.ndarray, yp: np.ndarray) -> float:
    xs, ys = _sorted_strict(np.asarray(xp, dtype=float), np.asarray(yp, dtype=float))
    return float(np.interp(xq, xs, ys, left=ys[0], right=ys[-1]))


def _lut_value_d1_d2(xq: float, xp: np.ndarray, yp: np.ndarray) -> tuple[float, float, float]:
    xs, ys = _sorted_strict(np.asarray(xp, dtype=float), np.asarray(yp, dtype=float))
    d1_grid = np.gradient(ys, xs, edge_order=1)
    d2_grid = np.gradient(d1_grid, xs, edge_order=1)
    y = float(np.interp(xq, xs, ys, left=ys[0], right=ys[-1]))
    d1 = float(np.interp(xq, xs, d1_grid, left=d1_grid[0], right=d1_grid[-1]))
    d2 = float(np.interp(xq, xs, d2_grid, left=d2_grid[0], right=d2_grid[-1]))
    return y, d1, d2


def _invert_lut(yq: float, xp: np.ndarray, yp: np.ndarray) -> float:
    ys, xs = _sorted_strict(np.asarray(yp, dtype=float), np.asarray(xp, dtype=float))
    return float(np.interp(yq, ys, xs, left=xs[0], right=xs[-1]))


def _poly_eval_and_derivatives(x: float, coeffs_ascending: list[float]) -> tuple[float, float, float]:
    coeffs = np.asarray(coeffs_ascending, dtype=float)
    y = float(np.sum(coeffs * np.power(x, np.arange(coeffs.size))))
    if coeffs.size <= 1:
        return y, 0.0, 0.0
    d1 = float(np.sum(np.arange(1, coeffs.size) * coeffs[1:] * np.power(x, np.arange(coeffs.size - 1))))
    if coeffs.size <= 2:
        return y, d1, 0.0
    idx = np.arange(2, coeffs.size)
    d2 = float(np.sum(idx * (idx - 1.0) * coeffs[2:] * np.power(x, np.arange(coeffs.size - 2))))
    return y, d1, d2


def _ball_inertia_kgm2(params: dict[str, Any]) -> float:
    plant = params["plant"]
    m = float(plant["ball_mass_kg"])
    r = float(plant["ball_radius_m"])

    if "ball_inertia_kgm2" in plant:
        return float(plant["ball_inertia_kgm2"])
    if "ball_inertia_ratio" in plant:
        return float(plant["ball_inertia_ratio"]) * m * r * r
    if "rolling_factor" in plant:
        alpha = float(plant["rolling_factor"])
        if not (0.0 < alpha < 1.0):
            raise ValueError("rolling_factor must satisfy 0 < alpha < 1")
        return (m / alpha - m) * r * r
    return (2.0 / 5.0) * m * r * r


def effective_mass_kg(params: dict[str, Any]) -> float:
    plant = params["plant"]
    m = float(plant["ball_mass_kg"])
    r = float(plant["ball_radius_m"])
    return m + (_ball_inertia_kgm2(params) / (r * r))


def outer_model_coeffs(params: dict[str, Any]) -> tuple[float, float]:
    plant = params["plant"]
    m = float(plant["ball_mass_kg"])
    m_e = effective_mass_kg(params)

    if "rolling_damping_ns_per_m" in plant:
        b_x = float(plant["rolling_damping_ns_per_m"])
    else:
        c_legacy = float(plant.get("viscous_damping_1ps", 0.0))
        b_x = c_legacy * m_e

    alpha = m / m_e
    beta = b_x / m_e
    return alpha, beta


def theta_from_phi(
    phi_rad: float,
    theta_map_cfg: dict[str, Any],
    direction: float = 0.0,
) -> tuple[float, float, float]:
    mode = str(theta_map_cfg.get("mode", "linear")).lower()
    hcfg = theta_map_cfg.get("hysteresis", {})
    hysteresis_enabled = bool(hcfg.get("enabled", False))

    def eval_base() -> tuple[float, float, float]:
        if mode == "linear":
            lcfg = theta_map_cfg.get("linear", {})
            phi0 = float(lcfg.get("phi0_rad", 0.0))
            theta0 = float(lcfg.get("theta0_rad", 0.0))
            gain = float(lcfg.get("gain_theta_per_phi", lcfg.get("gain", 1.0)))
            return theta0 + gain * (phi_rad - phi0), gain, 0.0

        if mode == "poly":
            pcfg = theta_map_cfg.get("poly", {})
            coeffs = pcfg.get("coeffs", [0.0, 1.0])
            return _poly_eval_and_derivatives(phi_rad, list(coeffs))

        if mode == "lut":
            lcfg = theta_map_cfg.get("lut", {})
            return _lut_value_d1_d2(phi_rad, np.asarray(lcfg["phi_rad"]), np.asarray(lcfg["theta_rad"]))

        raise ValueError(f"Unsupported theta_from_phi mode: {mode}")

    def eval_hyst_branch(branch: str) -> tuple[float, float, float]:
        bcfg = hcfg.get(branch, {})
        return _lut_value_d1_d2(
            phi_rad,
            np.asarray(bcfg["phi_rad"], dtype=float),
            np.asarray(bcfg["theta_rad"], dtype=float),
        )

    if hysteresis_enabled:
        has_up = isinstance(hcfg.get("up"), dict) and "phi_rad" in hcfg["up"] and "theta_rad" in hcfg["up"]
        has_dn = isinstance(hcfg.get("down"), dict) and "phi_rad" in hcfg["down"] and "theta_rad" in hcfg["down"]

        if has_up and has_dn:
            if direction > 0.0:
                return eval_hyst_branch("up")
            if direction < 0.0:
                return eval_hyst_branch("down")

            up = eval_hyst_branch("up")
            dn = eval_hyst_branch("down")
            return ((up[0] + dn[0]) * 0.5, (up[1] + dn[1]) * 0.5, (up[2] + dn[2]) * 0.5)

    return eval_base()


def phi_from_theta(
    theta_rad: float,
    theta_map_cfg: dict[str, Any],
    direction: float = 0.0,
) -> float:
    mode = str(theta_map_cfg.get("mode", "linear")).lower()
    hcfg = theta_map_cfg.get("hysteresis", {})
    hysteresis_enabled = bool(hcfg.get("enabled", False))

    if hysteresis_enabled:
        has_up = isinstance(hcfg.get("up"), dict) and "phi_rad" in hcfg["up"] and "theta_rad" in hcfg["up"]
        has_dn = isinstance(hcfg.get("down"), dict) and "phi_rad" in hcfg["down"] and "theta_rad" in hcfg["down"]
        if has_up and has_dn:
            branch = "up" if direction >= 0.0 else "down"
            bcfg = hcfg[branch]
            return _invert_lut(
                theta_rad,
                np.asarray(bcfg["phi_rad"], dtype=float),
                np.asarray(bcfg["theta_rad"], dtype=float),
            )

    if mode == "linear":
        lcfg = theta_map_cfg.get("linear", {})
        phi0 = float(lcfg.get("phi0_rad", 0.0))
        theta0 = float(lcfg.get("theta0_rad", 0.0))
        gain = float(lcfg.get("gain_theta_per_phi", lcfg.get("gain", 1.0)))
        if abs(gain) < 1e-12:
            raise ValueError("theta_from_phi linear gain too small for inversion.")
        return phi0 + (theta_rad - theta0) / gain

    if mode == "lut":
        lcfg = theta_map_cfg.get("lut", {})
        return _invert_lut(
            theta_rad,
            np.asarray(lcfg["phi_rad"], dtype=float),
            np.asarray(lcfg["theta_rad"], dtype=float),
        )

    if mode == "poly":
        pcfg = theta_map_cfg.get("poly", {})
        coeffs = list(pcfg.get("coeffs", [0.0, 1.0]))
        phi_guess = float(theta_rad)
        for _ in range(20):
            y, d1, _ = _poly_eval_and_derivatives(phi_guess, coeffs)
            err = y - theta_rad
            if abs(err) < 1e-10:
                return phi_guess
            if abs(d1) < 1e-10:
                break
            phi_guess -= err / d1
        raise ValueError("Failed to invert theta_from_phi poly map near requested theta.")

    raise ValueError(f"Unsupported theta_from_phi mode for inversion: {mode}")


def x_from_d_theta(d_m: float, theta_rad: float, x_map_cfg: dict[str, Any]) -> float:
    mode = str(x_map_cfg.get("mode", "linear")).lower()

    if mode == "linear":
        lcfg = x_map_cfg.get("linear", {})
        center_m = float(lcfg.get("center_m", 0.0))
        sign = float(lcfg.get("sign", 1.0))
        gain = float(lcfg.get("gain_x_per_d", 1.0))
        return sign * gain * (d_m - center_m)

    if mode == "affine_theta":
        acfg = x_map_cfg.get("affine_theta", {})
        c0 = float(acfg.get("c0", 0.0))
        c_d = float(acfg.get("c_d", 1.0))
        c_th = float(acfg.get("c_theta", 0.0))
        c_dt = float(acfg.get("c_dtheta", 0.0))
        return c0 + c_d * d_m + c_th * theta_rad + c_dt * d_m * theta_rad

    if mode == "lut1d":
        lcfg = x_map_cfg.get("lut1d", {})
        theta_gain = float(lcfg.get("theta_gain", 0.0))
        x0 = _interp_monotonic(d_m, np.asarray(lcfg["d_m"], dtype=float), np.asarray(lcfg["x_m"], dtype=float))
        return x0 + theta_gain * theta_rad

    if mode == "lut2d":
        lcfg = x_map_cfg.get("lut2d", {})
        d_grid = _require_strict_increasing(np.asarray(lcfg["d_m"], dtype=float), "lut2d d_m")
        t_grid = _require_strict_increasing(np.asarray(lcfg["theta_rad"], dtype=float), "lut2d theta_rad")
        x_grid = np.asarray(lcfg["x_m"], dtype=float)
        if x_grid.shape != (t_grid.size, d_grid.size):
            raise ValueError("lut2d x_m must have shape [len(theta_rad), len(d_m)]")

        d = float(np.clip(d_m, d_grid[0], d_grid[-1]))
        t = float(np.clip(theta_rad, t_grid[0], t_grid[-1]))

        i = int(np.clip(np.searchsorted(d_grid, d) - 1, 0, d_grid.size - 2))
        j = int(np.clip(np.searchsorted(t_grid, t) - 1, 0, t_grid.size - 2))

        d0, d1 = d_grid[i], d_grid[i + 1]
        t0, t1 = t_grid[j], t_grid[j + 1]
        xd = 0.0 if d1 == d0 else (d - d0) / (d1 - d0)
        xt = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)

        q11 = x_grid[j, i]
        q21 = x_grid[j, i + 1]
        q12 = x_grid[j + 1, i]
        q22 = x_grid[j + 1, i + 1]
        return float(
            (1.0 - xd) * (1.0 - xt) * q11
            + xd * (1.0 - xt) * q21
            + (1.0 - xd) * xt * q12
            + xd * xt * q22
        )

    raise ValueError(f"Unsupported x_from_d_theta mode: {mode}")


def d_from_x_theta(x_m: float, theta_rad: float, x_map_cfg: dict[str, Any]) -> float:
    mode = str(x_map_cfg.get("mode", "linear")).lower()

    if mode == "linear":
        lcfg = x_map_cfg.get("linear", {})
        center_m = float(lcfg.get("center_m", 0.0))
        sign = float(lcfg.get("sign", 1.0))
        gain = float(lcfg.get("gain_x_per_d", 1.0))
        denom = sign * gain
        if abs(denom) < 1e-12:
            raise ValueError("linear x_from_d_theta gain too small for inversion.")
        return center_m + (x_m / denom)

    if mode == "affine_theta":
        acfg = x_map_cfg.get("affine_theta", {})
        c0 = float(acfg.get("c0", 0.0))
        c_d = float(acfg.get("c_d", 1.0))
        c_th = float(acfg.get("c_theta", 0.0))
        c_dt = float(acfg.get("c_dtheta", 0.0))
        denom = c_d + c_dt * theta_rad
        if abs(denom) < 1e-12:
            raise ValueError("affine_theta denominator too small for inversion.")
        return (x_m - c0 - c_th * theta_rad) / denom

    if mode == "lut1d":
        lcfg = x_map_cfg.get("lut1d", {})
        theta_gain = float(lcfg.get("theta_gain", 0.0))
        x0 = x_m - theta_gain * theta_rad
        return _invert_lut(
            x0,
            np.asarray(lcfg["d_m"], dtype=float),
            np.asarray(lcfg["x_m"], dtype=float),
        )

    raise ValueError(f"Inverse not implemented for x_from_d_theta mode: {mode}")


def actuator_defaults(params: dict[str, Any]) -> dict[str, float]:
    plant = params["plant"]
    design = params.get("design", {})
    act = params.get("actuator", {})

    j = float(plant.get("beam_inertia_kgm2", 0.005))
    r = float(plant["ball_radius_m"])
    m_e = effective_mass_kg(params)
    j_eq0 = j + m_e * r * r

    inner_bw_hz = float(design.get("inner_bw_hz", 4.0))
    inner_zeta = float(design.get("inner_zeta", 0.9))
    wn = 2.0 * np.pi * inner_bw_hz

    kp_default = j_eq0 * wn * wn
    kd_default = 2.0 * inner_zeta * wn * j_eq0

    return {
        "phi_tau_s": float(act.get("phi_tau_s", plant.get("actuator_tau_s", 0.08))),
        "theta_kp_nm_per_rad": float(act.get("theta_kp_nm_per_rad", kp_default)),
        "theta_kd_nms_per_rad": float(act.get("theta_kd_nms_per_rad", kd_default)),
        "torque_limit_nm": float(act.get("torque_limit_nm", 0.35)),
    }


def theta_tracking_torque(
    theta_ref_rad: float,
    theta_rad: float,
    theta_dot_radps: float,
    params: dict[str, Any],
) -> float:
    ac = actuator_defaults(params)
    tau = ac["theta_kp_nm_per_rad"] * (theta_ref_rad - theta_rad) - ac["theta_kd_nms_per_rad"] * theta_dot_radps
    return clamp(tau, -ac["torque_limit_nm"], ac["torque_limit_nm"])


def full_coupled_accels(
    x_m: float,
    x_dot_mps: float,
    theta_rad: float,
    theta_dot_radps: float,
    tau_act_nm: float,
    params: dict[str, Any],
) -> tuple[float, float]:
    plant = params["plant"]
    g = float(plant["gravity_mps2"])
    m = float(plant["ball_mass_kg"])
    r = float(plant["ball_radius_m"])

    m_e = effective_mass_kg(params)

    j = float(plant.get("beam_inertia_kgm2", 0.005))
    m_b = float(plant.get("beam_mass_kg", 0.0))
    l_b = float(plant.get("beam_com_l_m", 0.0))
    h_b = float(plant.get("beam_com_h_m", 0.0))

    if "rolling_damping_ns_per_m" in plant:
        b_x = float(plant["rolling_damping_ns_per_m"])
    else:
        b_x = float(plant.get("viscous_damping_1ps", 0.0)) * m_e
    f_cx = float(plant.get("rolling_coulomb_n", 0.0))
    c_theta = float(plant.get("pivot_damping_nms", 0.0))
    tau_c_theta = float(plant.get("pivot_coulomb_nm", 0.0))

    qx = -b_x * x_dot_mps - f_cx * sgn(x_dot_mps)
    qtheta = tau_act_nm - c_theta * theta_dot_radps - tau_c_theta * sgn(theta_dot_radps)

    a11 = m_e
    a12 = m_e * r
    a21 = m_e * r
    a22 = j + m_e * r * r + m * x_m * x_m

    rhs_x = m * x_m * theta_dot_radps * theta_dot_radps + m * g * np.sin(theta_rad) + qx
    rhs_theta = (
        qtheta
        - 2.0 * m * x_m * x_dot_mps * theta_dot_radps
        + m * g * (x_m * np.cos(theta_rad) + r * np.sin(theta_rad))
        + m_b * g * (l_b * np.cos(theta_rad) + h_b * np.sin(theta_rad))
    )

    mat = np.array([[a11, a12], [a21, a22]], dtype=float)
    rhs = np.array([rhs_x, rhs_theta], dtype=float)
    x_ddot, theta_ddot = np.linalg.solve(mat, rhs)
    return float(x_ddot), float(theta_ddot)
