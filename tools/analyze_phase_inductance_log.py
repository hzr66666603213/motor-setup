#!/usr/bin/env python3
"""Parse ODrive v3.6 phase-inductance bring-up logs.

The firmware log currently contains merged per-sample means plus rank A/B
summary peaks.  It does not contain per-repeat or per-rank waveforms, so this
tool exports the recoverable merged curves and explicitly reports fields that
cannot be reconstructed offline.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from statistics import median, pstdev

RISE_SAMPLES = 30
FALL_SAMPLES = 80
TS_S = 50.0e-6
R_PHASE = 3.2
AMP_PER_COUNT = 0.0201416


def parse_kv(line: str) -> dict[str, str]:
    return {m.group(1): m.group(2) for m in re.finditer(r"(\w+)=([^\s]+)", line)}


def to_float(kv: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(kv.get(key, default))
    except ValueError:
        return default


def robust_sigma(values: list[float]) -> float:
    if not values:
        return 0.0
    med = median(values)
    return 1.4826 * median([abs(v - med) for v in values])


def r2_and_residual(y: list[float], yfit: list[float]) -> tuple[float, float, float]:
    if not y:
        return 0.0, 0.0, 0.0
    mean = sum(y) / len(y)
    ss_tot = sum((v - mean) ** 2 for v in y)
    ss_res = sum((a - b) ** 2 for a, b in zip(y, yfit))
    rms = math.sqrt(ss_res / len(y))
    max_res = max(abs(a - b) for a, b in zip(y, yfit))
    return (1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else 0.0, rms, max_res


def fit_rise(samples: list[dict[str, float]], resistance: float) -> dict[str, float]:
    y = [s["delta_i_alpha_mean"] for s in samples]
    tail = y[-10:] if len(y) >= 10 else y
    tail_med = median(tail)
    noise = max(robust_sigma(tail), 0.5 * AMP_PER_COUNT)
    offset_limit = max(3.0 * noise, abs(tail_med) * 0.05, 0.0005)
    best = None
    for dstep in range(5):
        delay = TS_S * dstep * 0.25
        for tstep in range(120):
            tau = 20.0e-6 * math.exp(math.log(500.0) * tstep / 119.0)
            z = []
            for i in range(len(y)):
                t = (i + 1) * TS_S - delay
                z.append(0.0 if t <= 0 else 1.0 - math.exp(-t / tau))
            s00 = float(len(y))
            s01 = sum(z)
            s11 = sum(v * v for v in z)
            sy0 = sum(y)
            sy1 = sum(a * b for a, b in zip(z, y))
            det = s00 * s11 - s01 * s01
            if abs(det) < 1e-12:
                continue
            offset = (sy0 * s11 - sy1 * s01) / det
            amp = (s00 * sy1 - s01 * sy0) / det
            if amp <= 0 or abs(offset) > offset_limit:
                continue
            fit = [offset + amp * zi for zi in z]
            sse = sum((a - b) ** 2 for a, b in zip(y, fit))
            if best is None or sse < best[0]:
                r2, rms, max_res = r2_and_residual(y, fit)
                best = (sse, offset, amp, tau, delay, r2, rms, max_res)
    if best is None:
        return {"valid": 0.0}
    _, offset, amp, tau, delay, r2, rms, max_res = best
    return {
        "valid": 1.0,
        "tail_median": tail_med,
        "peak": max(y),
        "peak_minus_tail": max(y) - tail_med,
        "offset": offset,
        "amplitude": amp,
        "tau_us": tau * 1e6,
        "delay_us": delay * 1e6,
        "L_uH": resistance * tau * 1e6,
        "r2": r2,
        "residual_rms": rms,
        "residual_max": max_res,
        "noise_sigma": noise,
    }


def fit_fall(samples: list[dict[str, float]], resistance: float) -> dict[str, float]:
    y = [s["delta_i_alpha_mean"] for s in samples]
    std = [s["delta_i_alpha_std"] for s in samples]
    tail = y[-10:] if len(y) >= 10 else y
    offset_est = median(tail)
    noise = max(robust_sigma(tail), pstdev(tail) * 0.5 if len(tail) > 1 else 0.0, 0.5 * AMP_PER_COUNT)
    initial = y[0] - offset_est
    if initial <= 0:
        return {"valid": 0.0}
    best = None

    for start in (0, 1):
        if start + 4 >= len(y):
            continue
        start_initial = y[start] - offset_est
        if start_initial <= 0:
            continue
        max_end = len(y) - 1
        floor = 3.0 * noise
        ratio_floor = 0.15 * abs(start_initial)
        tail_candidate_run = 0
        for i in range(start + 1, len(y)):
            ay = abs(y[i] - offset_est)
            if ay < floor or ay < ratio_floor:
                tail_candidate_run += 1
            else:
                tail_candidate_run = 0
            if tail_candidate_run >= 3:
                tail_start_index = i - tail_candidate_run + 1
                max_end = max(start + 4, tail_start_index - 1)
                break
        max_end = min(max_end, len(y) - 1)
        for end in range(start + 4, max_end + 1):
            point_count = end - start + 1
            ywin = y[start : end + 1]
            swin = std[start : end + 1]
            for ostep in range(-8, 9):
                offset = offset_est + ostep * noise * 0.25
                for tstep in range(160):
                    tau = 20.0e-6 * math.exp(math.log(500.0) * tstep / 159.0)
                    z = [math.exp(-((i - start) * TS_S) / tau) for i in range(start, end + 1)]
                    weights = [1.0 / (si * si + noise * noise) for si in swin]
                    sz2 = sum(w * zi * zi for w, zi in zip(weights, z))
                    if sz2 <= 1e-12:
                        continue
                    amp = sum(w * zi * (yi - offset) for w, zi, yi in zip(weights, z, ywin)) / sz2
                    if amp <= 0:
                        continue
                    fit = [offset + amp * zi for zi in z]
                    weighted_sse = sum(w * (yi - fi) ** 2 for w, yi, fi in zip(weights, ywin, fit))
                    weighted_rms = math.sqrt(weighted_sse / sum(weights))
                    r2, rms, max_res = r2_and_residual(ywin, fit)
                    normalized = weighted_rms / noise if noise > 0 else 999.0
                    r2_penalty = max(0.0, 0.95 - r2) * 4.0
                    point_penalty = 0.03 * (10 - point_count) if point_count < 10 else 0.0
                    score = normalized + r2_penalty + point_penalty
                    if best is None or score < best[0]:
                        best = (
                            score,
                            start,
                            end,
                            offset,
                            amp,
                            tau,
                            r2,
                            rms,
                            max_res,
                            weighted_rms,
                            normalized,
                            fit,
                        )
    if best is None:
        return {"valid": 0.0}
    score, start, end, offset, amp, tau, r2, rms, max_res, weighted_rms, normalized, fit = best
    diag_rows = []
    residuals = []
    max_idx = start
    same_run = 0
    max_same_run = 0
    last_sign = 0
    for pos, i in enumerate(range(start, end + 1)):
        fitted = fit[pos]
        residual = y[i] - fitted
        residuals.append(residual)
        if abs(residual) >= abs(y[max_idx] - fit[max_idx - start]):
            max_idx = i
        sign = 1 if residual > 0 else (-1 if residual < 0 else 0)
        if sign and sign == last_sign:
            same_run += 1
        else:
            same_run = 1 if sign else 0
        max_same_run = max(max_same_run, same_run)
        last_sign = sign
        diag_rows.append(
            {
                "sample_index": i,
                "t_us": samples[i]["t_us"],
                "mean_current": y[i],
                "std_current": std[i],
                "fitted_current": fitted,
                "residual": residual,
                "residual_adc_counts": residual / AMP_PER_COUNT,
                "in_3sigma_noise": 1 if abs(y[i] - offset_est) < 3.0 * noise else 0,
            }
        )
    curvature_vals = [
        residuals[i] - 2.0 * residuals[i - 1] + residuals[i - 2]
        for i in range(2, len(residuals))
    ]
    curvature = math.sqrt(sum(v * v for v in curvature_vals) / len(curvature_vals)) if curvature_vals else 0.0
    return {
        "valid": 1.0,
        "start_index": start,
        "end_index": end,
        "point_count": end - start + 1,
        "initial_current": y[start] - offset_est,
        "noise_floor": noise,
        "offset": offset,
        "amplitude": amp,
        "tau_us": tau * 1e6,
        "L_uH": resistance * tau * 1e6,
        "r2": r2,
        "residual_rms": rms,
        "residual_max": max_res,
        "weighted_residual_rms": weighted_rms,
        "normalized_residual_rms": normalized,
        "window_score": score,
        "residual_max_index": max_idx,
        "residual_max_counts": max_res / AMP_PER_COUNT,
        "residual_same_sign_run": max_same_run,
        "residual_curvature": curvature,
        "drop_one_sensitivity": "NOT_USED_FOR_FORMAL_PASS",
        "diag_rows": diag_rows,
        "rank_a_fit": "UNAVAILABLE_NO_PER_RANK_WAVEFORM_IN_LOG",
        "rank_b_fit": "UNAVAILABLE_NO_PER_RANK_WAVEFORM_IN_LOG",
    }


def fit_fall_fixed_window(samples: list[dict[str, float]],
                          resistance: float,
                          start: int,
                          end: int) -> dict[str, float]:
    y = [s["delta_i_alpha_mean"] for s in samples]
    std = [s["delta_i_alpha_std"] for s in samples]
    if not y:
        return {"valid": 0}
    start = max(0, min(start, len(y) - 1))
    end = max(start + 4, min(end, len(y) - 1))
    if end - start + 1 < 5:
        return {"valid": 0}
    tail = y[-10:] if len(y) >= 10 else y
    offset_est = median(tail)
    noise = max(robust_sigma(tail), pstdev(tail) * 0.5 if len(tail) > 1 else 0.0, 0.5 * AMP_PER_COUNT)
    ywin = y[start : end + 1]
    swin = std[start : end + 1]
    best = None
    for ostep in range(-8, 9):
        offset = offset_est + ostep * noise * 0.25
        for tstep in range(160):
            tau = 20.0e-6 * math.exp(math.log(500.0) * tstep / 159.0)
            z = [math.exp(-((i - start) * TS_S) / tau) for i in range(start, end + 1)]
            weights = [1.0 / (si * si + noise * noise) for si in swin]
            sz2 = sum(w * zi * zi for w, zi in zip(weights, z))
            if sz2 <= 1e-12:
                continue
            amp = sum(w * zi * (yi - offset) for w, zi, yi in zip(weights, z, ywin)) / sz2
            if amp <= 0:
                continue
            fit = [offset + amp * zi for zi in z]
            weighted_sse = sum(w * (yi - fi) ** 2 for w, yi, fi in zip(weights, ywin, fit))
            weighted_rms = math.sqrt(weighted_sse / sum(weights))
            r2, rms, max_res = r2_and_residual(ywin, fit)
            if best is None or weighted_rms < best[0]:
                best = (weighted_rms, offset, amp, tau, r2, rms, max_res)
    if best is None:
        return {"valid": 0}
    weighted_rms, offset, amp, tau, r2, rms, max_res = best
    return {
        "valid": 1,
        "start_index": start,
        "end_index": end,
        "point_count": end - start + 1,
        "L_uH": resistance * tau * 1e6,
        "r2": r2,
        "residual_rms": rms,
        "weighted_residual_rms": weighted_rms,
        "residual_max": max_res,
        "residual_max_counts": max_res / AMP_PER_COUNT,
        "offset": offset,
        "amplitude": amp,
    }


def rise_diag_rows(samples: list[dict[str, float]], rise_fit: dict[str, float]) -> list[dict[str, object]]:
    if rise_fit.get("valid", 0.0) < 1.0:
        return []
    tau = rise_fit["tau_us"] * 1e-6
    delay = rise_fit["delay_us"] * 1e-6
    rows = []
    for row in samples:
        t = row["t_us"] * 1e-6 - delay
        z = 0.0 if t <= 0 else 1.0 - math.exp(-t / tau)
        fitted = rise_fit["offset"] + rise_fit["amplitude"] * z
        residual = row["delta_i_alpha_mean"] - fitted
        rows.append({
            "sample_index": row["sample_index"],
            "t_us": row["t_us"],
            "raw_pc0": row.get("raw_pc0", ""),
            "raw_pc1": row.get("raw_pc1", ""),
            "mean_current": row["delta_i_alpha_mean"],
            "std_current": row["delta_i_alpha_std"],
            "fitted_current": fitted,
            "residual": residual,
            "residual_adc_counts": residual / AMP_PER_COUNT,
        })
    return rows


def fall_window_sensitivity(samples: list[dict[str, float]], fall_fit: dict[str, float]) -> list[dict[str, object]]:
    if fall_fit.get("valid", 0.0) < 1.0:
        return []
    base_start = int(fall_fit["start_index"])
    base_end = int(fall_fit["end_index"])
    rows = []
    for ds in (-1, 0, 1):
        for de in (-1, 0, 1):
            start = base_start + ds
            end = base_end + de
            fit = fit_fall_fixed_window(samples, R_PHASE, start, end)
            fit["start_delta"] = ds
            fit["end_delta"] = de
            rows.append(fit)
    return rows


def linear_slope(values: list[float], start: int, end: int) -> float:
    if end <= start:
        return 0.0
    xs = [float(i - start) for i in range(start, end + 1)]
    ys = values[start : end + 1]
    n = float(len(xs))
    sx = sum(xs)
    sy = sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    det = n * sxx - sx * sx
    return (n * sxy - sx * sy) / det if abs(det) > 1e-12 else 0.0


def rise_monotonic_end_before_90(rise: list[dict[str, float]], rise_fit: dict[str, float]) -> int:
    if not rise or rise_fit.get("valid", 0.0) < 1.0:
        return max(0, len(rise) - 1)
    steady = rise_fit.get("offset", 0.0) + rise_fit.get("amplitude", 0.0)
    threshold = 0.90 * steady
    for i, row in enumerate(rise):
        if row["delta_i_alpha_mean"] >= threshold:
            return i - 1 if i > 1 else min(1, len(rise) - 1)
    return len(rise) - 1


def monotonic_stats(samples: list[dict[str, float]], start: int, end: int, rise: bool) -> dict[str, float]:
    y = [row["delta_i_alpha_mean"] for row in samples]
    std = [row["delta_i_alpha_std"] for row in samples]
    if len(y) < 2:
        return {
            "monotonic_window_start": 0,
            "monotonic_window_end": 0,
            "comparison_count": 0,
            "violation_count": 0,
            "violation_ratio": 1.0,
            "max_violation_a": 0.0,
            "max_violation_counts": 0.0,
            "tolerance_max_a": 0.0,
            "global_trend_slope": 0.0,
            "ok": 0,
        }
    start = max(0, min(start, len(y) - 1))
    end = max(start + 1, min(end, len(y) - 1))
    comparison_count = 0
    violation_count = 0
    max_violation = 0.0
    tolerance_max = 0.0
    large_violation = False
    for k in range(start + 1, end + 1):
        sigma_delta = math.sqrt(std[k] * std[k] + std[k - 1] * std[k - 1])
        tolerance = max(3.0 * sigma_delta, 0.5 * AMP_PER_COUNT)
        local_limit = max(4.0 * sigma_delta, 1.0 * AMP_PER_COUNT)
        reversal = (y[k - 1] - y[k]) if rise else (y[k] - y[k - 1])
        tolerance_max = max(tolerance_max, tolerance)
        if reversal > tolerance:
            violation_count += 1
            max_violation = max(max_violation, reversal)
            if reversal > local_limit:
                large_violation = True
        comparison_count += 1
    ratio = violation_count / comparison_count if comparison_count else 1.0
    slope = linear_slope(y, start, end)
    trend_ok = slope > 0.0 if rise else slope < 0.0
    return {
        "monotonic_window_start": start,
        "monotonic_window_end": end,
        "comparison_count": comparison_count,
        "violation_count": violation_count,
        "violation_ratio": ratio,
        "max_violation_a": max_violation,
        "max_violation_counts": max_violation / AMP_PER_COUNT if AMP_PER_COUNT > 0 else 0.0,
        "tolerance_max_a": tolerance_max,
        "global_trend_slope": slope,
        "ok": 1 if comparison_count > 0 and ratio <= 0.15 and not large_violation and trend_ok else 0,
    }


def solve_3x3(m, y):
    det = (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )
    if abs(det) < 1e-12:
        return None
    out = []
    for col in range(3):
        mm = [row[:] for row in m]
        for row in range(3):
            mm[row][col] = y[row]
        d = (
            mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1])
            - mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0])
            + mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0])
        )
        out.append(d / det)
    return out


def fit_arx(rise, fall, resistance, voltage_step, fixed_r=False, smooth=False):
    r = [s["delta_i_alpha_mean"] for s in rise]
    f = [s["delta_i_alpha_mean"] for s in fall]
    if smooth:
        r = [(r[max(0, i - 1)] + r[i] + r[min(len(r) - 1, i + 1)]) / 3.0 for i in range(len(r))]
        f = [(f[max(0, i - 1)] + f[i] + f[min(len(f) - 1, i + 1)]) / 3.0 for i in range(len(f))]
    if fixed_r:
        xs, ys = [], []
        for arr, v in ((r, voltage_step), (f, 0.0)):
            for i in range(len(arr) - 1):
                xs.append(arr[i] - v / resistance)
                ys.append(arr[i + 1] - v / resistance)
        n = len(xs)
        sx, sy = sum(xs), sum(ys)
        sxx = sum(x * x for x in xs)
        sxy = sum(x * y for x, y in zip(xs, ys))
        det = n * sxx - sx * sx
        if abs(det) < 1e-12:
            return {"valid": 0}
        a = (n * sxy - sx * sy) / det
        c = (sy - a * sx) / n
        b = (1.0 - a) / resistance
    else:
        m = [[0.0] * 3 for _ in range(3)]
        yy = [0.0] * 3
        for arr, v in ((r, voltage_step), (f, 0.0)):
            for i in range(len(arr) - 1):
                x = [arr[i], v, 1.0]
                for row in range(3):
                    yy[row] += x[row] * arr[i + 1]
                    for col in range(3):
                        m[row][col] += x[row] * x[col]
        sol = solve_3x3(m, yy)
        if sol is None:
            return {"valid": 0}
        a, b, c = sol
    if not (0 < a < 1 and b > 0):
        return {"valid": 0, "a": a, "b": b, "c": c}
    tau = -TS_S / math.log(a)
    r_est = resistance if fixed_r else (1.0 - a) / b
    return {
        "valid": 1,
        "a": a,
        "b": b,
        "c": c,
        "R_ohm": r_est,
        "tau_us": tau * 1e6,
        "L_uH": r_est * tau * 1e6,
    }


def parse_log(path: Path):
    levels: dict[int, dict[str, object]] = {}
    samples = []
    active_windows = []
    rank_summaries = []
    for line in path.read_text(errors="replace").splitlines():
        lm = re.search(r"inductance_level(\d+):", line)
        if lm:
            level = int(lm.group(1))
            levels.setdefault(level, {})["summary"] = parse_kv(line)
            continue
        rbm = re.search(r"inductance_level(\d+)_robust:", line)
        if rbm:
            level = int(rbm.group(1))
            levels.setdefault(level, {})["robust"] = parse_kv(line)
            continue
        wm = re.search(r"inductance_level(\d+)_active_sample_window:", line)
        if wm:
            level = int(wm.group(1))
            kv = parse_kv(line)
            row = {
                "level": level,
                "tag": kv.get("tag", ""),
                "tim1_cnt": to_float(kv, "TIM1_CNT"),
                "direction": to_float(kv, "direction"),
                "active_command": to_float(kv, "active_command"),
                "ccr1": to_float(kv, "CCR1"),
                "ccr2": to_float(kv, "CCR2"),
                "ccr3": to_float(kv, "CCR3"),
                "ccr4": to_float(kv, "CCR4"),
                "high_side_mask": kv.get("high_side_mask", ""),
                "low_side_mask": kv.get("low_side_mask", ""),
                "vw_low_side_sampleable": to_float(kv, "vw_low_side_sampleable"),
                "distance_to_nearest_edge_us": to_float(kv, "distance_to_nearest_edge_us"),
                "sample_window_valid": to_float(kv, "sample_window_valid"),
            }
            levels.setdefault(level, {}).setdefault("active_windows", []).append(row)
            active_windows.append(row)
            continue
        rm = re.search(r"inductance_level(\d+)_rank:", line)
        if rm:
            level = int(rm.group(1))
            kv = parse_kv(line)
            row = {"level": level}
            row.update(kv)
            levels.setdefault(level, {})["rank_summary"] = row
            rank_summaries.append(row)
            continue
        rfitm = re.search(r"inductance_level(\d+)_rank([AB])_fit:", line)
        if rfitm:
            level = int(rfitm.group(1))
            rank = rfitm.group(2)
            levels.setdefault(level, {}).setdefault("rank_fits", {})[rank] = parse_kv(line)
            continue
        rsm = re.search(r"inductance_level(\d+)_rank([AB])_(rise|fall)_sample(\d+):", line)
        if rsm:
            level = int(rsm.group(1))
            rank = rsm.group(2)
            kind = rsm.group(3)
            idx = int(rsm.group(4))
            kv = parse_kv(line)
            row = {
                "level": level,
                "rank": rank,
                "kind": kind,
                "sample_index": idx,
                "mean": to_float(kv, "mean"),
                "std": to_float(kv, "std"),
                "valid_count": to_float(kv, "valid_count"),
            }
            levels.setdefault(level, {}).setdefault(f"rank_{rank.lower()}_{kind}", []).append(row)
            continue
        ram = re.search(r"inductance_level(\d+)_repeat_arx_fixed_R:", line)
        if ram:
            level = int(ram.group(1))
            levels.setdefault(level, {})["repeat_arx_fixed_R"] = parse_kv(line)
            continue
        rdm = re.search(r"inductance_level(\d+)_repeat(\d+)_diag:", line)
        if rdm:
            level = int(rdm.group(1))
            repeat = int(rdm.group(2))
            kv = parse_kv(line)
            row = {"level": level, "repeat": repeat}
            row.update(kv)
            levels.setdefault(level, {}).setdefault("repeat_diag", []).append(row)
            continue
        sm = re.search(r"inductance_level(\d+)_(rise|fall)_sample(\d+):", line)
        if sm:
            level = int(sm.group(1))
            kind = sm.group(2)
            idx = int(sm.group(3))
            kv = parse_kv(line)
            row = {
                "level": level,
                "kind": kind,
                "sample_index": idx,
                "t_us": to_float(kv, "t_us"),
                "adc_seq": to_float(kv, "adc_seq"),
                "command_seq": to_float(kv, "voltage_command_seq"),
                "raw_pc0": kv.get("raw_pc0", ""),
                "raw_pc1": kv.get("raw_pc1", ""),
                "voltage_for_this_sample": to_float(kv, "voltage_for_this_sample"),
                "commanded_next_voltage": to_float(kv, "commanded_next_voltage"),
                "commanded_voltage": to_float(kv, "commanded_voltage"),
                "ccr1": to_float(kv, "CCR1"),
                "ccr2": to_float(kv, "CCR2"),
                "ccr3": to_float(kv, "CCR3"),
                "ccr4": to_float(kv, "CCR4"),
                "delta_i_alpha_mean": to_float(kv, "delta_i_alpha_mean"),
                "delta_i_alpha_std": to_float(kv, "delta_i_alpha_std"),
                "delta_i_beta_mean": to_float(kv, "delta_i_beta_mean"),
                "valid_repeat_count": to_float(kv, "valid_repeat_count"),
                "rank_order": "",
            }
            levels.setdefault(level, {}).setdefault(kind, []).append(row)
            samples.append(row)
    return levels, samples, active_windows, rank_summaries


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def analyze_one_log(log_path: Path, out_dir: Path) -> str:
    out_dir.mkdir(parents=True, exist_ok=True)

    levels, samples, active_windows, rank_summaries = parse_log(log_path)
    write_csv(out_dir / "phase_inductance_all_samples.csv", samples)
    write_csv(out_dir / "active_sample_windows.csv", active_windows)
    write_csv(out_dir / "rank_summaries.csv", rank_summaries)

    summary_lines = [f"log={log_path}"]
    missing = [
        "per-repeat baseline samples",
        "per-repeat rank A waveform",
        "per-repeat rank B waveform",
    ]
    if any("raw_pc0" in row and row["raw_pc0"] != "" for row in samples):
        summary_lines.append("raw_pc0_pc1_for_averaged_samples=available")
    else:
        missing.append("raw PC0/PC1 for each averaged sample")
    if not active_windows:
        missing.append("active-pulse sample_window_diag lines")
    summary_lines.append("missing_fields=" + ",".join(missing))
    active_confirmed = bool(active_windows) and all(
        (float(row["sample_window_valid"]) >= 1.0)
        and (float(row["vw_low_side_sampleable"]) >= 1.0)
        and (float(row["distance_to_nearest_edge_us"]) >= 2.0)
        for row in active_windows
    )
    summary_lines.append(
        "active_pulse_sample_window_confirmed=%d count=%d min_distance_us=%.3f"
        % (
            1 if active_confirmed else 0,
            len(active_windows),
            min([float(row["distance_to_nearest_edge_us"]) for row in active_windows], default=0.0),
        )
    )

    for level, data in sorted(levels.items()):
        rise = sorted(data.get("rise", []), key=lambda r: r["sample_index"])
        fall = sorted(data.get("fall", []), key=lambda r: r["sample_index"])
        write_csv(out_dir / f"level{level}_rise.csv", rise)
        write_csv(out_dir / f"level{level}_fall.csv", fall)
        write_csv(out_dir / f"level{level}_merged.csv", rise + fall)
        rank_a = sorted(data.get("rank_a_rise", []), key=lambda r: r["sample_index"]) + \
                 sorted(data.get("rank_a_fall", []), key=lambda r: r["sample_index"])
        rank_b = sorted(data.get("rank_b_rise", []), key=lambda r: r["sample_index"]) + \
                 sorted(data.get("rank_b_fall", []), key=lambda r: r["sample_index"])
        write_csv(out_dir / f"level{level}_rank_a.csv", rank_a)
        write_csv(out_dir / f"level{level}_rank_b.csv", rank_b)
        write_csv(out_dir / f"level{level}_repeat_diag.csv", data.get("repeat_diag", []))
        if "rank_summary" in data:
            summary_lines.append(f"level{level}_rank_summary={data['rank_summary']}")
        if "rank_fits" in data:
            summary_lines.append(f"level{level}_rank_fits={data['rank_fits']}")
        if "repeat_arx_fixed_R" in data:
            summary_lines.append(f"level{level}_repeat_arx_fixed_R={data['repeat_arx_fixed_R']}")
        if not rise or not fall:
            continue
        summary = data.get("summary", {})
        voltage_step = to_float(summary, "delta_v_applied", 0.0)
        old = {
            "L_rise_uH": to_float(summary, "L_rise_uH"),
            "L_fall_uH": to_float(summary, "L_fall_uH"),
            "L_initial_slope_uH": to_float(summary, "L_initial_slope_uH"),
            "L_discrete_uH": to_float(summary, "L_discrete_uH"),
            "rise_r2": to_float(summary, "rise_fit_r_squared"),
            "fall_r2": to_float(summary, "fall_fit_r_squared"),
        }
        rise_fit = fit_rise(rise, R_PHASE)
        fall_fit = fit_fall(fall, R_PHASE)
        rise_diag = rise_diag_rows(rise, rise_fit)
        if rise_diag:
            write_csv(out_dir / f"level{level}_rise_fit_diag.csv", rise_diag)
        fall_diag_rows = fall_fit.get("diag_rows", [])
        if isinstance(fall_diag_rows, list):
            write_csv(out_dir / f"level{level}_fall_window_diag.csv", fall_diag_rows)
        fall_sensitivity = fall_window_sensitivity(fall, fall_fit)
        write_csv(out_dir / f"level{level}_fall_window_sensitivity.csv", fall_sensitivity)
        fall_fit_summary = {k: v for k, v in fall_fit.items() if k != "diag_rows"}
        rise_mono = monotonic_stats(
            rise, 0, rise_monotonic_end_before_90(rise, rise_fit), True
        )
        fall_mono = monotonic_stats(
            fall, int(fall_fit.get("start_index", 0)), int(fall_fit.get("end_index", len(fall) - 1)), False
        )
        arx_free = fit_arx(rise, fall, R_PHASE, voltage_step, fixed_r=False, smooth=False)
        arx_fixed = fit_arx(rise, fall, R_PHASE, voltage_step, fixed_r=True, smooth=False)
        arx_smooth = fit_arx(rise, fall, R_PHASE, voltage_step, fixed_r=True, smooth=True)
        robust = data.get("robust", {})
        recomputed_l_rise_uh = to_float(summary, "L_rise_uH")
        recomputed_l_fall_uh = float(fall_fit.get("L_uH", 0.0))
        recomputed_fall_r2 = float(fall_fit.get("r2", 0.0))
        rise_fall_diff = (
            abs(recomputed_l_rise_uh - recomputed_l_fall_uh)
            / (0.5 * (abs(recomputed_l_rise_uh) + abs(recomputed_l_fall_uh)))
            * 100.0
            if recomputed_l_rise_uh > 0.0 and recomputed_l_fall_uh > 0.0
            else 999.0
        )
        exp_avg = 0.5 * (recomputed_l_rise_uh + recomputed_l_fall_uh)
        discrete_diff = (
            abs(to_float(summary, "L_discrete_uH") - exp_avg)
            / (0.5 * (abs(to_float(summary, "L_discrete_uH")) + abs(exp_avg)))
            * 100.0
            if to_float(summary, "L_discrete_uH") > 0.0 and exp_avg > 0.0
            else 999.0
        )
        peak_alpha = to_float(summary, "peak_delta_i_alpha")
        peak_beta = to_float(summary, "peak_delta_i_beta")
        recomputed_level_reliable = (
            to_float(summary, "valid_repeat_count") >= 48.0
            and to_float(robust, "active_sample_window_valid") >= 1.0
            and voltage_step > 0.0
            and (peak_alpha >= 0.15 or to_float(summary, "effective_adc_counts") >= 8.0)
            and peak_alpha > 0.001
            and abs(peak_beta / peak_alpha) < 0.20
            and rise_mono["ok"] >= 1
            and fall_mono["ok"] >= 1
            and to_float(summary, "rise_fit_r_squared") >= 0.95
            and recomputed_fall_r2 >= 0.95
            and recomputed_l_rise_uh > 0.0
            and recomputed_l_fall_uh > 0.0
            and to_float(summary, "L_discrete_uH") > 0.0
            and to_float(robust, "L_fused_uH") > 0.0
            and to_float(robust, "fused_method_count") >= 2.0
            and rise_fall_diff < 25.0
            and discrete_diff < 30.0
            and to_float(summary, "dynamics_too_fast") == 0.0
            and to_float(summary, "pulse_too_short") == 0.0
        )
        summary_lines.append(f"level{level}_old={old}")
        summary_lines.append(f"level{level}_rise_fit={rise_fit}")
        summary_lines.append(f"level{level}_fall_fit={fall_fit_summary}")
        summary_lines.append(f"level{level}_fall_window_sensitivity={fall_sensitivity}")
        summary_lines.append(
            f"level{level}_rank_a_fit=UNAVAILABLE_NO_PER_RANK_WAVEFORM_IN_LOG"
            if not rank_a else f"level{level}_rank_a_fit={data.get('rank_fits', {}).get('A', {})}"
        )
        summary_lines.append(
            f"level{level}_rank_b_fit=UNAVAILABLE_NO_PER_RANK_WAVEFORM_IN_LOG"
            if not rank_b else f"level{level}_rank_b_fit={data.get('rank_fits', {}).get('B', {})}"
        )
        summary_lines.append(f"level{level}_rise_monotonic_recomputed={rise_mono}")
        summary_lines.append(f"level{level}_fall_monotonic_recomputed={fall_mono}")
        summary_lines.append(f"level{level}_arx_free={arx_free}")
        summary_lines.append(f"level{level}_arx_fixed_R={arx_fixed}")
        summary_lines.append(f"level{level}_arx_smoothed_fixed_R={arx_smooth}")
        summary_lines.append(
            "level%d_recomputed level_reliable=%d rise_fall_diff_percent=%.2f "
            "discrete_diff_percent=%.2f"
            % (
                level,
                1 if recomputed_level_reliable else 0,
                rise_fall_diff,
                discrete_diff,
            )
        )
        summary_lines.append(
            "level%d_noise baseline_sigma_a=UNAVAILABLE baseline_sigma_b=UNAVAILABLE "
            "fall_tail_sigma=%.6f effective_noise_counts=%.3f"
            % (
                level,
                robust_sigma([r["delta_i_alpha_mean"] for r in fall[-10:]]),
                max(robust_sigma([r["delta_i_alpha_mean"] for r in fall[-10:]]), 0.5 * AMP_PER_COUNT)
                / AMP_PER_COUNT,
            )
        )

    summary_path = out_dir / "phase_inductance_fit_summary.txt"
    summary_path.write_text("\n".join(summary_lines) + "\n")
    return f"wrote {out_dir}\n{summary_path.read_text()}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path, nargs="+")
    ap.add_argument("--out-dir", type=Path)
    args = ap.parse_args()
    outputs = []
    for log in args.log:
        if args.out_dir and len(args.log) == 1:
            out_dir = args.out_dir
        elif args.out_dir:
            out_dir = args.out_dir / log.stem
        else:
            out_dir = log.with_suffix("")
        outputs.append(analyze_one_log(log, out_dir))
    print("\n".join(outputs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
