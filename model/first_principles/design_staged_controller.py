#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from math import ceil, exp, floor, log, pi
from pathlib import Path
from typing import Any

import numpy as np
from scipy import linalg, signal

from first_principles_core import load_params, outer_model_coeffs


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _float(value: Any) -> float:
    return float(value)


def design_staged_controller(params: dict[str, Any]) -> dict[str, Any]:
    plant = params["plant"]
    limits = params["limits"]
    actuator = params["actuator"]
    calibration = params["calibration"]
    sign_audit = params["sign_audit"]
    design = params["design"]

    dist_cfg = calibration["d_from_voltage"]
    theta_cfg = calibration["theta_from_as5600_deg"]
    theta_linear = theta_cfg["linear"]
    theta_safe = theta_cfg["safe_range"]
    x_from_d_linear = calibration["x_from_d_theta"]["linear"]
    inner_cfg = design["staged_inner"]
    outer_cfg = design["staged_outer_lqi"]

    dt_s = _float(inner_cfg["dt_s"])
    _require(dt_s > 0.0, "staged_inner.dt_s must be positive")
    loop_ms = int(round(1000.0 * dt_s))

    d_min_cm = _float(dist_cfg["d_min_cm"])
    d_max_cm = _float(dist_cfg["d_max_cm"])
    d_setpoint_midpoint_cm = 0.5 * (d_min_cm + d_max_cm)
    d_setpoint_trim_default_cm = _float(dist_cfg["setpoint_trim_default_cm"])
    d_setpoint_default_cm = d_setpoint_midpoint_cm + d_setpoint_trim_default_cm
    x_scale_cm = 0.5 * (d_max_cm - d_min_cm)
    _require(x_scale_cm > 0.0, "distance window must be positive")
    center_from_legacy_x_map_cm = 100.0 * _float(x_from_d_linear["center_m"])
    _require(
        abs(center_from_legacy_x_map_cm - d_setpoint_midpoint_cm) < 1e-9,
        "calibration.x_from_d_theta.linear.center_m must match the Sharp midpoint",
    )
    _require(
        _float(x_from_d_linear["sign"]) > 0.0,
        "calibration.x_from_d_theta.linear.sign must remain positive for the pivot-mounted Sharp",
    )

    theta_slope_deg_per_as_deg = _float(theta_linear["slope_deg_per_as_deg"])
    theta_offset_deg = _float(theta_linear["offset_deg"])
    theta_cal_min_deg = _float(theta_safe["theta_cal_min_deg"])
    theta_cal_max_deg = _float(theta_safe["theta_cal_max_deg"])
    theta_cal_margin_deg = _float(theta_safe["theta_cal_margin_deg"])
    theta_cal_extrapolate_deg = _float(theta_safe["theta_cal_extrapolate_deg"])
    theta_cal_soft_min_deg = (
        theta_cal_min_deg + theta_cal_margin_deg - theta_cal_extrapolate_deg
    )
    theta_cal_soft_max_deg = (
        theta_cal_max_deg - theta_cal_margin_deg + theta_cal_extrapolate_deg
    )
    theta_safe_span_deg = theta_cal_soft_max_deg - theta_cal_soft_min_deg
    model_theta_envelope_deg = _float(limits["theta_cmd_deg"])
    theta_envelope_deg = min(model_theta_envelope_deg, theta_safe_span_deg)
    theta_cmd_limit_default_deg = (
        _float(outer_cfg["theta_limit_fraction_of_envelope"]) * theta_envelope_deg
    )
    _require(theta_cmd_limit_default_deg > 0.0, "theta command limit must be positive")

    as5600_raw_to_deg = 360.0 / 4096.0
    steps_per_rev = _float(actuator["stepper_steps_per_rev"])
    steps_per_beam_deg = (steps_per_rev / 360.0) / theta_slope_deg_per_as_deg
    beam_deg_per_step = 1.0 / steps_per_beam_deg
    as5600_lsb_beam_deg = theta_slope_deg_per_as_deg * as5600_raw_to_deg
    dominant_angle_quantization_deg = max(beam_deg_per_step, as5600_lsb_beam_deg)

    inner_discrete_pole = _float(inner_cfg["inner_discrete_pole"])
    _require(0.0 < inner_discrete_pole < 1.0, "inner discrete pole must be in (0, 1)")
    inner_bw_radps = -log(inner_discrete_pole) / dt_s
    inner_bw_hz = inner_bw_radps / (2.0 * pi)
    outer_bw_hz = _float(outer_cfg["outer_bw_hz"])
    outer_bw_radps = 2.0 * pi * outer_bw_hz
    _require(
        inner_bw_radps > outer_bw_radps,
        "inner loop must be faster than outer loop",
    )

    inner_kp_theta = 1.0 - inner_discrete_pole
    inner_kd_theta = _float(inner_cfg["inner_kd_theta"])
    inner_theta_deadband_deg = (
        _float(inner_cfg["deadband_quantization_multiple"])
        * dominant_angle_quantization_deg
    )
    inner_theta_rate_deadband_deg_s = inner_theta_deadband_deg / (
        _float(inner_cfg["rate_deadband_sample_factor"]) * dt_s
    )
    step_budget_per_sample = _float(limits["step_rate_sps"]) * dt_s
    theta_limit_steps = theta_cmd_limit_default_deg * steps_per_beam_deg
    inner_max_step_delta = int(
        floor(
            _float(inner_cfg["max_step_delta_budget_fraction"])
            * min(step_budget_per_sample, theta_limit_steps)
        )
    )
    _require(inner_max_step_delta >= 1, "inner_max_step_delta must be at least 1")

    alpha, beta_1ps = outer_model_coeffs(params)
    g_mps2 = _float(plant["gravity_mps2"])
    k_theta_cmps2_per_deg = alpha * g_mps2 * (pi / 180.0) * 100.0

    inner_zeta = _float(outer_cfg["actuator_model_inner_damping_ratio"])
    a_cont = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, -beta_1ps, -k_theta_cmps2_per_deg, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, -(inner_bw_radps**2), -2.0 * inner_zeta * inner_bw_radps],
        ],
        dtype=float,
    )
    b_cont = np.array([[0.0], [0.0], [0.0], [inner_bw_radps**2]], dtype=float)
    a_disc, b_disc, _, _, _ = signal.cont2discrete(
        (a_cont, b_cont, np.eye(4), np.zeros((4, 1))),
        dt_s,
    )

    a_aug = np.eye(5, dtype=float)
    a_aug[:4, :4] = a_disc
    a_aug[4, :4] = np.array([dt_s, 0.0, 0.0, 0.0], dtype=float)
    b_aug = np.zeros((5, 1), dtype=float)
    b_aug[:4, :] = b_disc

    theta_scale_deg = theta_cmd_limit_default_deg
    x_dot_scale_cm_s = outer_bw_radps * x_scale_cm
    theta_dot_state_scale_deg_s = inner_bw_radps * theta_scale_deg
    xi_scale_cm_s = x_scale_cm / outer_bw_radps

    q_weights = outer_cfg["q_weights"]
    r_weights = outer_cfg["r_weights"]
    q = np.diag(
        [
            _float(q_weights["wx"]) / (x_scale_cm**2),
            _float(q_weights["wv"]) / (x_dot_scale_cm_s**2),
            _float(q_weights["wt"]) / (theta_scale_deg**2),
            _float(q_weights["ww"]) / (theta_dot_state_scale_deg_s**2),
            _float(q_weights["wi"]) / (xi_scale_cm_s**2),
        ]
    )
    r = np.array([[_float(r_weights["wu"]) / (theta_scale_deg**2)]], dtype=float)

    p = linalg.solve_discrete_are(a_aug, b_aug, q, r)
    k_lqi = np.linalg.solve(b_aug.T @ p @ b_aug + r, b_aug.T @ p @ a_aug)
    closed_loop_eigs = np.linalg.eigvals(a_aug - b_aug @ k_lqi)
    closed_loop_pole_abs = np.abs(closed_loop_eigs)

    x_dot_limit_cm_s = (
        _float(outer_cfg["xdot_limit_scale_fraction"]) * x_dot_scale_cm_s
    )
    theta_dot_limit_deg_s = (
        _float(outer_cfg["theta_dot_limit_outer_bw_multiple"])
        * outer_bw_radps
        * theta_scale_deg
    )
    integral_clamp_deg = (
        _float(outer_cfg["integral_clamp_fraction_of_theta_limit"])
        * theta_scale_deg
    )
    center_band_cm = _float(outer_cfg["center_band_x_fraction_of_half_window"]) * x_scale_cm
    center_band_x_dot_cm_s = (
        _float(outer_cfg["center_band_xdot_fraction_of_xdot_limit"])
        * x_dot_limit_cm_s
    )
    integral_capture_cm = (
        _float(outer_cfg["integral_capture_x_fraction_of_half_window"]) * x_scale_cm
    )
    integral_capture_x_dot_cm_s = (
        _float(outer_cfg["integral_capture_xdot_fraction_of_xdot_limit"])
        * x_dot_limit_cm_s
    )

    recovery_enter_x_cm = (
        _float(outer_cfg["recovery_enter_x_fraction_of_half_window"]) * x_scale_cm
    )
    recovery_enter_x_dot_cm_s = (
        _float(outer_cfg["recovery_enter_xdot_fraction_of_xdot_limit"])
        * x_dot_limit_cm_s
    )
    recovery_enter_count = int(ceil(_float(outer_cfg["recovery_enter_time_s"]) / dt_s))
    recovery_exit_x_cm = (
        _float(outer_cfg["recovery_exit_x_fraction_of_half_window"]) * x_scale_cm
    )
    recovery_exit_handoff_x_cm = (
        _float(outer_cfg["recovery_exit_handoff_x_fraction_of_half_window"]) * x_scale_cm
    )
    recovery_exit_inward_x_dot_cm_s = (
        _float(outer_cfg["recovery_exit_inward_xdot_fraction_of_xdot_limit"])
        * x_dot_limit_cm_s
    )
    recovery_floor_default_deg = (
        _float(outer_cfg["recovery_floor_fraction_of_theta_limit"]) * theta_scale_deg
    )
    recovery_floor_min_deg = 0.0
    recovery_floor_max_deg = (
        _float(outer_cfg["recovery_floor_max_fraction_of_theta_limit"]) * theta_scale_deg
    )
    recovery_floor_gain_deg_per_cm = (
        _float(outer_cfg["recovery_floor_gain_fraction_of_theta_limit_per_half_window"])
        * theta_scale_deg
        / x_scale_cm
    )

    zero_trim_x_cm = (
        _float(outer_cfg["zero_trim_x_fraction_of_half_window"]) * x_scale_cm
    )
    zero_trim_x_dot_cm_s = (
        _float(outer_cfg["zero_trim_xdot_fraction_of_xdot_limit"]) * x_dot_limit_cm_s
    )
    zero_trim_theta_track_err_deg = (
        _float(outer_cfg["zero_trim_theta_track_fraction_of_theta_limit"])
        * theta_scale_deg
    )
    zero_trim_theta_dot_deg_s = (
        _float(outer_cfg["zero_trim_theta_dot_fraction_of_theta_dot_limit"])
        * theta_dot_limit_deg_s
    )
    zero_trim_alpha = 1.0 - exp(
        -dt_s / _float(outer_cfg["zero_trim_estimator_tau_s"])
    )

    constants: dict[str, Any] = {
        "LOOP_MS": loop_ms,
        "DT": dt_s,
        "SHARP_FIT_K_V_CM": _float(dist_cfg["reciprocal_affine"]["k_v_cm"]),
        "SHARP_FIT_OFFSET_CM": _float(dist_cfg["reciprocal_affine"]["offset_cm"]),
        "SHARP_MIN_VALID_V": _float(dist_cfg["min_valid_v"]),
        "D_MIN_CM": d_min_cm,
        "D_MAX_CM": d_max_cm,
        "D_SETPOINT_MIDPOINT_CM": d_setpoint_midpoint_cm,
        "D_SETPOINT_TRIM_DEFAULT_CM": d_setpoint_trim_default_cm,
        "D_SETPOINT_DEFAULT_CM": d_setpoint_default_cm,
        "AS5600_RAW_TO_DEG": as5600_raw_to_deg,
        "THETA_SLOPE_DEG_PER_AS_DEG": theta_slope_deg_per_as_deg,
        "THETA_OFFSET_DEG": theta_offset_deg,
        "THETA_CAL_MIN_DEG": theta_cal_min_deg,
        "THETA_CAL_MAX_DEG": theta_cal_max_deg,
        "THETA_CAL_MARGIN_DEG": theta_cal_margin_deg,
        "THETA_CAL_EXTRAPOLATE_DEG": theta_cal_extrapolate_deg,
        "STEPS_PER_BEAM_DEG": steps_per_beam_deg,
        "DIR_SIGN": int(sign_audit["dir_sign"]),
        "INNER_STEP_SIGN": int(sign_audit["inner_step_sign"]),
        "OUTER_SIGN_DEFAULT": int(sign_audit["outer_sign"]),
        "INNER_KP_THETA": inner_kp_theta,
        "INNER_KD_THETA": inner_kd_theta,
        "INNER_THETA_DEADBAND_DEG": inner_theta_deadband_deg,
        "INNER_THETA_RATE_DEADBAND_DEG_S": inner_theta_rate_deadband_deg_s,
        "INNER_MAX_STEP_DELTA": inner_max_step_delta,
        "THETA_CMD_LIMIT_DEFAULT_DEG": theta_cmd_limit_default_deg,
        "OUTER_KX_DEFAULT": float(k_lqi[0, 0]),
        "OUTER_KV_DEFAULT": float(k_lqi[0, 1]),
        "OUTER_KT_DEFAULT": float(k_lqi[0, 2]),
        "OUTER_KW_DEFAULT": float(k_lqi[0, 3]),
        "OUTER_KI_DEFAULT": float(k_lqi[0, 4]),
        "OUTER_INTEGRAL_CLAMP_DEG": integral_clamp_deg,
        "OUTER_INTEGRAL_CAPTURE_CM": integral_capture_cm,
        "OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S": integral_capture_x_dot_cm_s,
        "OUTER_INTEGRAL_BLEED_OUTSIDE": _float(outer_cfg["integral_bleed_outside"]),
        "OUTER_INTEGRAL_BLEED_RECOVERY": _float(outer_cfg["integral_bleed_recovery"]),
        "OUTER_INTEGRAL_BLEED_CENTER": _float(outer_cfg["integral_bleed_center"]),
        "OUTER_CENTER_BAND_CM": center_band_cm,
        "OUTER_CENTER_BAND_X_DOT_CM_S": center_band_x_dot_cm_s,
        "OUTER_X_DOT_LIMIT_CM_S": x_dot_limit_cm_s,
        "OUTER_THETA_DOT_LIMIT_DEG_S": theta_dot_limit_deg_s,
        "OUTER_GAIN_SCALE_MIN": _float(outer_cfg["gain_scale_min"]),
        "OUTER_GAIN_SCALE_MAX": _float(outer_cfg["gain_scale_max"]),
        "RECOVERY_ENTER_X_CM": recovery_enter_x_cm,
        "RECOVERY_ENTER_X_DOT_CM_S": recovery_enter_x_dot_cm_s,
        "RECOVERY_ENTER_COUNT": recovery_enter_count,
        "RECOVERY_EXIT_X_CM": recovery_exit_x_cm,
        "RECOVERY_EXIT_HANDOFF_X_CM": recovery_exit_handoff_x_cm,
        "RECOVERY_EXIT_INWARD_X_DOT_CM_S": recovery_exit_inward_x_dot_cm_s,
        "RECOVERY_FLOOR_DEFAULT_DEG": recovery_floor_default_deg,
        "RECOVERY_FLOOR_MIN_DEG": recovery_floor_min_deg,
        "RECOVERY_FLOOR_MAX_DEG": recovery_floor_max_deg,
        "RECOVERY_FLOOR_GAIN_DEG_PER_CM": recovery_floor_gain_deg_per_cm,
        "ZERO_TRIM_EST_X_CM": zero_trim_x_cm,
        "ZERO_TRIM_EST_X_DOT_CM_S": zero_trim_x_dot_cm_s,
        "ZERO_TRIM_EST_THETA_TRACK_ERR_DEG": zero_trim_theta_track_err_deg,
        "ZERO_TRIM_EST_THETA_DOT_DEG_S": zero_trim_theta_dot_deg_s,
        "ZERO_TRIM_EST_ALPHA": zero_trim_alpha,
    }

    units = {
        "LOOP_MS": "ms",
        "DT": "s",
        "SHARP_FIT_K_V_CM": "cm*V",
        "SHARP_FIT_OFFSET_CM": "cm",
        "SHARP_MIN_VALID_V": "V",
        "D_MIN_CM": "cm",
        "D_MAX_CM": "cm",
        "D_SETPOINT_MIDPOINT_CM": "cm",
        "D_SETPOINT_TRIM_DEFAULT_CM": "cm",
        "D_SETPOINT_DEFAULT_CM": "cm",
        "AS5600_RAW_TO_DEG": "as-deg/count",
        "THETA_SLOPE_DEG_PER_AS_DEG": "beam-deg/as-deg",
        "THETA_OFFSET_DEG": "beam-deg",
        "THETA_CAL_MIN_DEG": "deg",
        "THETA_CAL_MAX_DEG": "deg",
        "THETA_CAL_MARGIN_DEG": "deg",
        "THETA_CAL_EXTRAPOLATE_DEG": "deg",
        "STEPS_PER_BEAM_DEG": "steps/deg",
        "DIR_SIGN": "sign",
        "INNER_STEP_SIGN": "sign",
        "OUTER_SIGN_DEFAULT": "sign",
        "INNER_KP_THETA": "ratio",
        "INNER_KD_THETA": "s",
        "INNER_THETA_DEADBAND_DEG": "deg",
        "INNER_THETA_RATE_DEADBAND_DEG_S": "deg/s",
        "INNER_MAX_STEP_DELTA": "steps/sample",
        "THETA_CMD_LIMIT_DEFAULT_DEG": "deg",
        "OUTER_KX_DEFAULT": "deg/cm",
        "OUTER_KV_DEFAULT": "deg*s/cm",
        "OUTER_KT_DEFAULT": "deg/deg",
        "OUTER_KW_DEFAULT": "s",
        "OUTER_KI_DEFAULT": "deg/(cm*s)",
        "OUTER_INTEGRAL_CLAMP_DEG": "deg",
        "OUTER_INTEGRAL_CAPTURE_CM": "cm",
        "OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S": "cm/s",
        "OUTER_INTEGRAL_BLEED_OUTSIDE": "ratio/sample",
        "OUTER_INTEGRAL_BLEED_RECOVERY": "ratio/sample",
        "OUTER_INTEGRAL_BLEED_CENTER": "ratio/sample",
        "OUTER_CENTER_BAND_CM": "cm",
        "OUTER_CENTER_BAND_X_DOT_CM_S": "cm/s",
        "OUTER_X_DOT_LIMIT_CM_S": "cm/s",
        "OUTER_THETA_DOT_LIMIT_DEG_S": "deg/s",
        "OUTER_GAIN_SCALE_MIN": "ratio",
        "OUTER_GAIN_SCALE_MAX": "ratio",
        "RECOVERY_ENTER_X_CM": "cm",
        "RECOVERY_ENTER_X_DOT_CM_S": "cm/s",
        "RECOVERY_ENTER_COUNT": "samples",
        "RECOVERY_EXIT_X_CM": "cm",
        "RECOVERY_EXIT_HANDOFF_X_CM": "cm",
        "RECOVERY_EXIT_INWARD_X_DOT_CM_S": "cm/s",
        "RECOVERY_FLOOR_DEFAULT_DEG": "deg",
        "RECOVERY_FLOOR_MIN_DEG": "deg",
        "RECOVERY_FLOOR_MAX_DEG": "deg",
        "RECOVERY_FLOOR_GAIN_DEG_PER_CM": "deg/cm",
        "ZERO_TRIM_EST_X_CM": "cm",
        "ZERO_TRIM_EST_X_DOT_CM_S": "cm/s",
        "ZERO_TRIM_EST_THETA_TRACK_ERR_DEG": "deg",
        "ZERO_TRIM_EST_THETA_DOT_DEG_S": "deg/s",
        "ZERO_TRIM_EST_ALPHA": "ratio/sample",
    }

    verification = {
        "closed_loop_stable": bool(np.all(closed_loop_pole_abs < 1.0)),
        "closed_loop_max_abs_pole": float(np.max(closed_loop_pole_abs)),
        "closed_loop_poles": [complex(v).real if abs(complex(v).imag) < 1e-12 else [float(complex(v).real), float(complex(v).imag)] for v in closed_loop_eigs],
        "inner_outer_bandwidth_ratio": inner_bw_radps / outer_bw_radps,
        "ordering_ok": bool(
            0.0 < center_band_cm
            < integral_capture_cm
            < recovery_enter_x_cm
            < x_scale_cm
            and 0.0 < recovery_floor_default_deg < theta_cmd_limit_default_deg
        ),
    }

    _require(verification["closed_loop_stable"], "staged LQI closed loop is unstable")
    _require(verification["ordering_ok"], "derived guard bands violate required ordering")

    return {
        "meta": {
            "method": "staged-sharp-as5600-discrete-lqi",
            "controller_architecture": "staged-cascaded-sharp-as5600",
            "controller_state_order": [
                "x_cm",
                "x_dot_cm_s",
                "theta_rel_deg",
                "theta_dot_deg_s",
                "xi_cm_s",
            ],
            "controller_command": "theta_cmd_rel_deg",
            "notes": [
                "The active design is the staged Sharp/AS5600 controller documented in docs/modeling.md.",
                "DIR_SIGN, INNER_STEP_SIGN, and OUTER_SIGN_DEFAULT stay explicit so sign bookkeeping is not hidden inside runtime wiring assumptions.",
                "The runtime integral bleed factors intentionally remain 1.0 because this design uses gating and clamping, not active bleed-down.",
            ],
        },
        "inputs": {
            "distance_calibration": dist_cfg,
            "angle_calibration": theta_cfg,
            "x_from_d_theta": calibration["x_from_d_theta"],
            "sign_audit": sign_audit,
            "staged_inner": inner_cfg,
            "staged_outer_lqi": outer_cfg,
        },
        "plant_model": {
            "alpha": alpha,
            "beta_1ps": beta_1ps,
            "k_theta_cmps2_per_deg": k_theta_cmps2_per_deg,
            "a_continuous": a_cont.tolist(),
            "b_continuous": b_cont.tolist(),
            "a_discrete": a_disc.tolist(),
            "b_discrete": b_disc.tolist(),
        },
        "design_targets": {
            "loop_ms": loop_ms,
            "dt_s": dt_s,
            "inner_discrete_pole": inner_discrete_pole,
            "inner_bw_hz": inner_bw_hz,
            "inner_bw_radps": inner_bw_radps,
            "outer_bw_hz": outer_bw_hz,
            "outer_bw_radps": outer_bw_radps,
            "theta_safe_span_deg": theta_safe_span_deg,
            "theta_envelope_deg": theta_envelope_deg,
            "theta_cmd_limit_default_deg": theta_cmd_limit_default_deg,
        },
        "scales": {
            "x_scale_cm": x_scale_cm,
            "x_dot_scale_cm_s": x_dot_scale_cm_s,
            "theta_scale_deg": theta_scale_deg,
            "theta_dot_state_scale_deg_s": theta_dot_state_scale_deg_s,
            "xi_scale_cm_s": xi_scale_cm_s,
            "beam_deg_per_step": beam_deg_per_step,
            "as5600_lsb_beam_deg": as5600_lsb_beam_deg,
            "dominant_angle_quantization_deg": dominant_angle_quantization_deg,
            "step_budget_per_sample": step_budget_per_sample,
            "theta_limit_steps": theta_limit_steps,
        },
        "weights": {
            "q": q.tolist(),
            "r": r.tolist(),
            "q_weights": q_weights,
            "r_weights": r_weights,
        },
        "constants": constants,
        "units": units,
        "verification": verification,
    }


def _cpp_literal(name: str, value: Any) -> tuple[str, str]:
    if isinstance(value, bool):
        return "bool", "true" if value else "false"
    if isinstance(value, int):
        if name.endswith("_MS"):
            return "uint32_t", str(value)
        if "SIGN" in name:
            return "int8_t", str(value)
        if name.endswith("_COUNT") or name.endswith("_DELTA"):
            return "int32_t", str(value)
        return "int32_t", str(value)
    return "float", f"{float(value):.9f}f"


def render_header(payload: dict[str, Any]) -> str:
    constants = payload["constants"]
    units = payload["units"]
    notes = payload["meta"]["notes"]
    lines = []
    for name, value in constants.items():
        ctype, literal = _cpp_literal(name, value)
        unit = units.get(name, "n/a")
        lines.append(f"constexpr {ctype} {name} = {literal};  // {unit}")

    return """#pragma once

#include <stdint.h>

namespace bb {
namespace generated {
namespace staged {

// Auto-generated by model/first_principles/design_staged_controller.py
// Runtime state units: x_cm, x_dot_cm_s, theta_rel_deg, theta_dot_deg_s, xi_cm_s
// Runtime command: theta_cmd_rel_deg
// Notes:
""" + "\n".join(f"// - {note}" for note in notes) + "\n\n" + "\n".join(lines) + """

}  // namespace staged
}  // namespace generated
}  // namespace bb
"""


def print_summary(payload: dict[str, Any], json_path: Path, header_path: Path) -> None:
    constants = payload["constants"]
    verification = payload["verification"]
    targets = payload["design_targets"]

    print("Staged Sharp/AS5600 controller design")
    print("-------------------------------------")
    print(f"DT = {constants['DT']:.6f} s ({constants['LOOP_MS']} ms)")
    print(
        f"Inner bandwidth = {targets['inner_bw_hz']:.6f} Hz, "
        f"outer bandwidth = {targets['outer_bw_hz']:.6f} Hz"
    )
    print(
        "Outer gains = "
        f"[{constants['OUTER_KX_DEFAULT']:.9f}, "
        f"{constants['OUTER_KV_DEFAULT']:.9f}, "
        f"{constants['OUTER_KT_DEFAULT']:.9f}, "
        f"{constants['OUTER_KW_DEFAULT']:.9f}, "
        f"{constants['OUTER_KI_DEFAULT']:.9f}]"
    )
    print(
        f"Closed-loop max |z| = {verification['closed_loop_max_abs_pole']:.9f}, "
        f"stable = {verification['closed_loop_stable']}"
    )
    print("Saved:", json_path)
    print("Saved:", header_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Design the staged Sharp/AS5600 controller from the measured-model YAML"
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=Path(__file__).with_name("params_measured.yaml"),
        help="Path to parameter yaml",
    )
    parser.add_argument(
        "--json-output",
        type=Path,
        default=Path(__file__).with_name("staged_controller_constants.json"),
        help="Path to staged controller JSON output",
    )
    parser.add_argument(
        "--header-output",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "firmware"
        / "include"
        / "generated"
        / "staged_controller_constants.h",
        help="Path to staged controller firmware header",
    )
    args = parser.parse_args()

    params = load_params(args.params)
    payload = design_staged_controller(params)

    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.header_output.parent.mkdir(parents=True, exist_ok=True)
    with args.json_output.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    args.header_output.write_text(render_header(payload), encoding="utf-8")

    print_summary(payload, args.json_output, args.header_output)


if __name__ == "__main__":
    main()
