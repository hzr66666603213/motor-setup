# -*- coding: utf-8 -*-
"""
SolidWorks 友好版：内/外摆线针齿包络齿廓生成

功能：
1. 外摆线齿廓：Zc = Zp - 1
2. 内摆线齿廓：Zc = Zp + 1
3. 生成针齿参考图
4. 输出 SolidWorks 更容易拉伸的闭合 LWPOLYLINE DXF
5. 输出参考用 DXF / CSV / PNG

单位：mm

SolidWorks 拉伸优先用：
- SW_outer_{OUTER_TEETH}teeth_closed_profile.dxf
- SW_inner_{INNER_TEETH}teeth_closed_profile.dxf

参考检查用：
- outer_with_pins_reference.dxf
- inner_with_pins_reference.dxf
- inner_outer_with_pins_reference.dxf
- pins_only_reference.dxf
"""

import math
import csv
import json
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# 1. 参数区
# ============================================================

# 针齿数量
PIN_COUNT = 32

# 外摆线齿数：Zc = Zp - 1
OUTER_TEETH = PIN_COUNT - 1

# 内摆线齿数：Zc = Zp + 1
INNER_TEETH = PIN_COUNT + 1

# 针齿中心分布圆直径
PIN_PCD = 14.6

# 针齿半径，例如针齿直径 0.8，则填 0.4
PIN_RADIUS = 0.3

# 偏心距 e
ECCENTRICITY = 0.15

# 等距修形量，正值表示多让开一点
OFFSET_MOD = 0.02

# 额外侧隙，3D打印可放大到 0.05~0.15
CLEARANCE = 0.01

# 齿廓内部计算点数，不要太大，否则慢
SAMPLE_NUM = 5000

# SolidWorks 导出点数
# 一般 800~1600 比较合适
SW_EXPORT_POINTS = 1200

# 针齿圆点数，仅参考图用
PIN_CIRCLE_POINTS = 96

# 外摆线齿廓相位
OUTER_PROFILE_PHASE_DEG = 0.0

# 内摆线齿廓相位
INNER_PROFILE_PHASE_DEG = 0.0

# 外摆线对应针齿圈相位
OUTER_PIN_PHASE_DEG = 0.0

# 内摆线对应针齿圈相位
INNER_PIN_PHASE_DEG = 0.0

# 是否显示图像
SHOW_PLOT = True


# ============================================================
# 2. 基础函数
# ============================================================

def rot2d(angle_rad: float) -> np.ndarray:
    """二维旋转矩阵"""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, -s], [s, c]], dtype=float)


def rotate_points(points: np.ndarray, angle_rad: float) -> np.ndarray:
    """批量旋转点"""
    r = rot2d(angle_rad)
    return points @ r.T


def compute_normals(points: np.ndarray) -> np.ndarray:
    """计算曲线单位法向"""
    dx = np.gradient(points[:, 0])
    dy = np.gradient(points[:, 1])

    nx = -dy
    ny = dx

    length = np.sqrt(nx * nx + ny * ny)
    length[length < 1e-12] = 1e-12

    return np.column_stack((nx / length, ny / length))


def choose_normal_side(points: np.ndarray, normals: np.ndarray, side: str) -> np.ndarray:
    """
    选择法向方向。

    towards_origin:
        法向大致指向原点。

    away_origin:
        法向大致背离原点。
    """
    radial_dot = np.sum(points * normals, axis=1)

    if side == "towards_origin":
        flip = radial_dot > 0
    elif side == "away_origin":
        flip = radial_dot < 0
    else:
        raise ValueError("side must be 'towards_origin' or 'away_origin'")

    normals[flip] *= -1.0
    return normals


def resample_closed_curve(points: np.ndarray, target_num: int) -> np.ndarray:
    """
    将闭合曲线按弧长均匀重采样，减少 SolidWorks 导入卡顿。

    points:
        原始闭合曲线点阵，不要求最后一点等于第一点。

    target_num:
        输出点数。
    """
    pts = np.asarray(points, dtype=float)

    if len(pts) < 3:
        raise ValueError("points must contain at least 3 points")

    # 去掉末尾重复点
    if np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
        pts = pts[:-1]

    closed = np.vstack([pts, pts[0]])

    seg = np.diff(closed, axis=0)
    seg_len = np.sqrt(np.sum(seg * seg, axis=1))

    # 去掉零长度段
    valid = seg_len > 1e-12
    if not np.all(valid):
        new_closed = [closed[0]]
        for i, ok in enumerate(valid):
            if ok:
                new_closed.append(closed[i + 1])
        closed = np.array(new_closed)
        seg = np.diff(closed, axis=0)
        seg_len = np.sqrt(np.sum(seg * seg, axis=1))

    cum_len = np.insert(np.cumsum(seg_len), 0, 0.0)
    total_len = cum_len[-1]

    if total_len < 1e-9:
        raise ValueError("curve length is too small")

    new_s = np.linspace(0.0, total_len, target_num, endpoint=False)
    new_pts = []

    for s in new_s:
        idx = np.searchsorted(cum_len, s, side="right") - 1
        idx = min(idx, len(seg_len) - 1)

        if seg_len[idx] < 1e-12:
            new_pts.append(closed[idx])
            continue

        t = (s - cum_len[idx]) / seg_len[idx]
        p = closed[idx] * (1.0 - t) + closed[idx + 1] * t
        new_pts.append(p)

    return np.array(new_pts)




# ============================================================
# 2.5 参数计算 / K1 系数 / 重要参数报告
# ============================================================

def to_builtin(value):
    """把 numpy 类型转换成普通 Python 类型，便于 JSON/CSV 输出。"""
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): to_builtin(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(v) for v in value]
    return value


def evaluate_k1(k1: float) -> str:
    """
    对短幅系数 K1 做经验性提示。

    这里采用常见定义：
        K1 = e * Zp / Rp
    其中：
        e  = 偏心距
        Zp = 针齿数量
        Rp = 针齿中心分布圆半径

    注意：不同资料也可能用摆线轮齿数 Zc 替代 Zp。
    所以后面的报告会同时给出 K1_by_pin_count 和 K1_by_profile_teeth。
    """
    if k1 < 0.45:
        return "偏小：齿形会比较尖，承载齿廓可能偏薄，需要重点查齿根和顶切。"
    if k1 < 0.55:
        return "略小：小型化有利，但齿根/顶切/加工误差要重点检查。"
    if k1 <= 0.80:
        return "常用区间附近：适合作为初版设计起点。"
    if k1 <= 0.95:
        return "偏大：齿形更饱满，但外径、滑动和干涉要重点检查。"
    return "过大风险：可能导致齿廓过钝、干涉或外径增大，建议降低 e 或增大 Rp。"


def polygon_area_closed(points: np.ndarray) -> float:
    """闭合多边形面积，点阵不要求首尾重复。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        return 0.0
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def curve_perimeter_closed(points: np.ndarray) -> float:
    """闭合曲线折线长度。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return 0.0
    seg = np.roll(pts, -1, axis=0) - pts
    return float(np.sum(np.sqrt(np.sum(seg * seg, axis=1))))


def estimate_curvature_radius(points: np.ndarray) -> dict:
    """
    估算闭合曲线曲率半径。

    用离散点中心差分估算，仅作为几何检查：
    - min_radius_of_curvature 越小，齿顶/齿根越尖，加工和应力越敏感。
    - 对真实接触强度，还需要基于实际啮合位置计算等效曲率半径。
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) < 5:
        return {
            "min_radius_of_curvature_mm": None,
            "max_curvature_1_per_mm": None,
            "min_radius_point_x_mm": None,
            "min_radius_point_y_mm": None,
        }

    prev_pts = np.roll(pts, 1, axis=0)
    next_pts = np.roll(pts, -1, axis=0)

    d1 = 0.5 * (next_pts - prev_pts)
    d2 = next_pts - 2.0 * pts + prev_pts

    numerator = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    denominator = (d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5
    denominator[denominator < 1e-18] = np.nan

    curvature = numerator / denominator
    finite = np.isfinite(curvature) & (curvature > 1e-12)

    if not np.any(finite):
        return {
            "min_radius_of_curvature_mm": None,
            "max_curvature_1_per_mm": None,
            "min_radius_point_x_mm": None,
            "min_radius_point_y_mm": None,
        }

    radius = np.full_like(curvature, np.inf)
    radius[finite] = 1.0 / curvature[finite]
    idx = int(np.nanargmin(radius))

    return {
        "min_radius_of_curvature_mm": float(radius[idx]),
        "max_curvature_1_per_mm": float(curvature[idx]),
        "min_radius_point_x_mm": float(pts[idx, 0]),
        "min_radius_point_y_mm": float(pts[idx, 1]),
    }


def minimum_clearance_to_pins(
    profile: np.ndarray,
    pin_centers: np.ndarray,
    pin_radius: float
) -> dict:
    """
    计算当前装配相位下，齿廓点到最近针齿圆的最小距离。

    gap > 0：当前相位存在几何间隙
    gap = 0：理论相切
    gap < 0：当前相位发生穿透/干涉

    注意：这里只检查 theta=0 当前装配相位，不等于完整相位扫描。
    """
    pts = np.asarray(profile, dtype=float)
    centers = np.asarray(pin_centers, dtype=float)

    if len(pts) == 0 or len(centers) == 0:
        return {
            "min_gap_to_reference_pins_mm": None,
            "nearest_pin_index": None,
            "profile_point_index": None,
        }

    diff = pts[:, None, :] - centers[None, :, :]
    dist = np.sqrt(np.sum(diff * diff, axis=2)) - pin_radius
    flat_idx = int(np.argmin(dist))
    point_idx, pin_idx = np.unravel_index(flat_idx, dist.shape)

    return {
        "min_gap_to_reference_pins_mm": float(dist[point_idx, pin_idx]),
        "nearest_pin_index": int(pin_idx),
        "profile_point_index": int(point_idx),
        "gap_point_x_mm": float(pts[point_idx, 0]),
        "gap_point_y_mm": float(pts[point_idx, 1]),
    }


def calculate_design_parameters(
    pin_count: int,
    profile_teeth: int,
    pin_pcd: float,
    pin_radius: float,
    eccentricity: float,
    offset_mod: float,
    clearance: float,
    mode: str,
) -> dict:
    """计算 K1、K2、针齿间隙、理论速比等基础设计参数。"""
    rp = pin_pcd / 2.0
    pin_diameter = 2.0 * pin_radius
    pin_pitch_arc = math.pi * pin_pcd / pin_count
    pin_pitch_chord = 2.0 * rp * math.sin(math.pi / pin_count)
    pin_chord_gap = pin_pitch_chord - pin_diameter
    pin_arc_gap = pin_pitch_arc - pin_diameter
    offset_distance = pin_radius + offset_mod + clearance

    k1_by_pin_count = eccentricity * pin_count / rp if rp > 0 else float("nan")
    k1_by_profile_teeth = eccentricity * profile_teeth / rp if rp > 0 else float("nan")

    # K2 在不同资料里定义不完全统一，这里同时输出半径/直径相对针齿分布半径的系数。
    k2_pin_radius_over_rp = pin_radius / rp if rp > 0 else float("nan")
    k2_pin_diameter_over_rp = pin_diameter / rp if rp > 0 else float("nan")

    # 常规一级摆线针轮，固定针齿壳、输出摆线轮自转时，理论减速比通常近似取 Zc:1。
    # 内/外摆线和 NN 结构的实际速比需要按具体约束链重新推导。
    conventional_single_stage_ratio_abs = abs(profile_teeth)

    return {
        "mode": mode,
        "pin_count_Zp": pin_count,
        "profile_teeth_Zc": profile_teeth,
        "pin_pcd_mm": pin_pcd,
        "pin_ring_radius_Rp_mm": rp,
        "pin_radius_rp_mm": pin_radius,
        "pin_diameter_dp_mm": pin_diameter,
        "eccentricity_e_mm": eccentricity,
        "offset_mod_mm": offset_mod,
        "clearance_mm": clearance,
        "offset_distance_pin_radius_plus_mod_plus_clearance_mm": offset_distance,
        "K1_short_amplitude_by_pin_count_eZp_over_Rp": k1_by_pin_count,
        "K1_short_amplitude_by_profile_teeth_eZc_over_Rp": k1_by_profile_teeth,
        "K1_evaluation": evaluate_k1(k1_by_pin_count),
        "K2_pin_radius_over_Rp": k2_pin_radius_over_rp,
        "K2_pin_diameter_over_Rp": k2_pin_diameter_over_rp,
        "eccentricity_ratio_e_over_Rp": eccentricity / rp if rp > 0 else float("nan"),
        "pin_pitch_arc_mm": pin_pitch_arc,
        "pin_pitch_chord_mm": pin_pitch_chord,
        "adjacent_pin_arc_gap_mm": pin_arc_gap,
        "adjacent_pin_chord_gap_mm": pin_chord_gap,
        "pin_pitch_to_pin_diameter_arc_ratio": pin_pitch_arc / pin_diameter if pin_diameter > 0 else float("nan"),
        "conventional_single_stage_ratio_abs_reference": conventional_single_stage_ratio_abs,
    }


def analyze_closed_profile(
    profile: np.ndarray,
    pin_centers: np.ndarray,
    pin_radius: float,
    name: str,
) -> dict:
    """计算齿廓外包络、面积、周长、曲率、当前相位针齿间隙等。"""
    pts = np.asarray(profile, dtype=float)
    if len(pts) < 3:
        raise ValueError("profile must contain at least 3 points")

    radii = np.sqrt(np.sum(pts * pts, axis=1))
    x_min, y_min = np.min(pts, axis=0)
    x_max, y_max = np.max(pts, axis=0)

    result = {
        "profile_name": name,
        "point_count": int(len(pts)),
        "min_radius_from_origin_mm": float(np.min(radii)),
        "max_radius_from_origin_mm": float(np.max(radii)),
        "radial_depth_max_minus_min_mm": float(np.max(radii) - np.min(radii)),
        "outer_envelope_diameter_by_radius_mm": float(2.0 * np.max(radii)),
        "inner_envelope_diameter_by_radius_mm": float(2.0 * np.min(radii)),
        "bbox_x_min_mm": float(x_min),
        "bbox_x_max_mm": float(x_max),
        "bbox_y_min_mm": float(y_min),
        "bbox_y_max_mm": float(y_max),
        "bbox_width_mm": float(x_max - x_min),
        "bbox_height_mm": float(y_max - y_min),
        "closed_area_mm2": float(polygon_area_closed(pts)),
        "closed_perimeter_mm": float(curve_perimeter_closed(pts)),
    }

    result.update(estimate_curvature_radius(pts))
    result.update(minimum_clearance_to_pins(pts, pin_centers, pin_radius))
    return result


def collect_report_warnings(report: dict) -> list[str]:
    """根据报告生成检查提示。"""
    warnings = []

    for key in ("outer_design", "inner_design"):
        d = report.get(key, {})
        mode = d.get("mode", key)
        k1 = d.get("K1_short_amplitude_by_pin_count_eZp_over_Rp")
        chord_gap = d.get("adjacent_pin_chord_gap_mm")
        if isinstance(k1, (int, float)):
            if k1 < 0.50:
                warnings.append(f"{mode}: K1={k1:.4f} 偏小，重点查顶切、齿根厚度、最小曲率半径。")
            elif k1 > 0.90:
                warnings.append(f"{mode}: K1={k1:.4f} 偏大，重点查外径、干涉、滑动和齿顶厚度。")
        if isinstance(chord_gap, (int, float)) and chord_gap <= 0:
            warnings.append(f"{mode}: 相邻针齿弦向间隙={chord_gap:.4f} mm，针齿圆已经互相重叠。")

    for key in ("outer_profile", "inner_profile"):
        p = report.get(key, {})
        name = p.get("profile_name", key)
        min_gap = p.get("min_gap_to_reference_pins_mm")
        min_rho = p.get("min_radius_of_curvature_mm")
        if isinstance(min_gap, (int, float)) and min_gap < -1e-6:
            warnings.append(f"{name}: 当前 theta=0 参考相位下，齿廓与针齿有 {min_gap:.4f} mm 穿透；需要调整相位或做完整相位扫描。")
        if isinstance(min_rho, (int, float)) and min_rho < 0.05:
            warnings.append(f"{name}: 最小曲率半径 {min_rho:.4f} mm 很小，加工/应力/导入 SolidWorks 都要小心。")

    if not warnings:
        warnings.append("未发现明显的基础几何报警；仍需做完整装配相位扫描和接触校核。")
    return warnings


def flatten_dict(data: dict, prefix: str = "") -> list[tuple[str, object]]:
    """将嵌套字典展平成 path, value。"""
    rows = []
    for key, value in data.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            rows.extend(flatten_dict(value, path))
        else:
            rows.append((path, value))
    return rows


def save_parameter_report_json(filename: str, report: dict) -> None:
    """保存参数报告 JSON。"""
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(to_builtin(report), f, ensure_ascii=False, indent=2)


def save_parameter_report_csv(filename: str, report: dict) -> None:
    """保存参数报告 CSV。"""
    with open(filename, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["parameter", "value"])
        for path, value in flatten_dict(to_builtin(report)):
            if isinstance(value, list):
                value = " | ".join(str(v) for v in value)
            writer.writerow([path, value])


def save_parameter_report_txt(filename: str, report: dict) -> None:
    """保存可读性更好的 TXT 参数报告。"""
    flat_rows = flatten_dict(to_builtin(report))
    width = max(len(k) for k, _ in flat_rows) if flat_rows else 20

    with open(filename, "w", encoding="utf-8") as f:
        f.write("Cycloid Geometry Parameter Report\n")
        f.write("=================================\n\n")
        for key, value in flat_rows:
            if isinstance(value, list):
                value = " | ".join(str(v) for v in value)
            f.write(f"{key:<{width}} : {value}\n")


def print_parameter_report(report: dict) -> None:
    """在控制台打印核心参数。"""
    print("\n========== K1 / 重要几何参数 ==========")
    for section in ("outer_design", "inner_design"):
        d = report[section]
        print(f"\n[{d['mode']}] Zp={d['pin_count_Zp']}, Zc={d['profile_teeth_Zc']}")
        print(f"Rp = {d['pin_ring_radius_Rp_mm']:.4f} mm, e = {d['eccentricity_e_mm']:.4f} mm")
        print(f"K1 = e*Zp/Rp = {d['K1_short_amplitude_by_pin_count_eZp_over_Rp']:.4f}")
        print(f"K1_alt = e*Zc/Rp = {d['K1_short_amplitude_by_profile_teeth_eZc_over_Rp']:.4f}")
        print(f"K2_radius = r_pin/Rp = {d['K2_pin_radius_over_Rp']:.4f}")
        print(f"K2_diameter = d_pin/Rp = {d['K2_pin_diameter_over_Rp']:.4f}")
        print(f"相邻针齿弦向间隙 = {d['adjacent_pin_chord_gap_mm']:.4f} mm")
        print(f"参考理论单级速比 = {d['conventional_single_stage_ratio_abs_reference']}:1")
        print(f"K1提示：{d['K1_evaluation']}")

    for section in ("outer_profile", "inner_profile"):
        p = report[section]
        print(f"\n[{p['profile_name']} profile]")
        print(f"外包络直径 ≈ {p['outer_envelope_diameter_by_radius_mm']:.4f} mm")
        print(f"BBox = {p['bbox_width_mm']:.4f} × {p['bbox_height_mm']:.4f} mm")
        print(f"最小曲率半径 ≈ {p['min_radius_of_curvature_mm']:.6f} mm")
        print(f"当前相位最小针齿间隙 ≈ {p['min_gap_to_reference_pins_mm']:.6f} mm")

    print("\n[检查提示]")
    for item in report.get("warnings", []):
        print(f"- {item}")


# ============================================================
# 3. 齿廓生成
# ============================================================

def generate_pin_envelope_profile(
    pin_count: int,
    profile_teeth: int,
    pin_pcd: float,
    pin_radius: float,
    eccentricity: float,
    sample_num: int,
    mode: str,
    phase_deg: float = 0.0,
    offset_mod: float = 0.0,
    clearance: float = 0.0,
) -> np.ndarray:
    """
    生成针齿圆包络齿廓。

    mode = "outer":
        外摆线齿廓，通常 profile_teeth = pin_count - 1

    mode = "inner":
        内摆线齿廓，通常 profile_teeth = pin_count + 1

    注意：
    这是用于前期建模的包络近似生成法。
    最终工程设计仍需做装配干涉检查、修形、相位扫描。
    """

    _ = pin_count

    rp = pin_pcd / 2.0
    offset_distance = pin_radius + offset_mod + clearance

    theta = np.linspace(
        0.0,
        2.0 * math.pi * profile_teeth,
        sample_num,
        endpoint=False
    )

    # 取一个针齿作为包络源
    pin_center_fixed = np.array([rp, 0.0], dtype=float)
    centers_in_wheel = np.zeros((sample_num, 2), dtype=float)

    if mode == "outer":
        # 外摆线：齿数比针齿少 1
        beta_sign = -1.0
        normal_side = "towards_origin"

    elif mode == "inner":
        # 内摆线：齿数比针齿多 1
        beta_sign = +1.0
        normal_side = "away_origin"

    else:
        raise ValueError("mode must be 'outer' or 'inner'")

    for i, th in enumerate(theta):
        # 摆线轮中心相对针齿壳中心的偏心运动
        c = np.array([
            eccentricity * math.cos(th),
            eccentricity * math.sin(th)
        ])

        # 摆线轮自转角
        beta = beta_sign * th / profile_teeth

        # 固定针齿中心变换到摆线轮坐标系
        q = rot2d(-beta) @ (pin_center_fixed - c)
        centers_in_wheel[i, :] = q

    normals = compute_normals(centers_in_wheel)
    normals = choose_normal_side(centers_in_wheel, normals, normal_side)

    # 按针齿半径 + 修形 + 侧隙偏置
    profile = centers_in_wheel + offset_distance * normals

    # 齿廓整体相位旋转
    profile = rotate_points(profile, math.radians(phase_deg))

    return profile


# ============================================================
# 4. 针齿生成
# ============================================================

def generate_pin_centers(
    pin_count: int,
    pin_pcd: float,
    eccentricity: float,
    ring_phase_deg: float = 0.0,
) -> np.ndarray:
    """
    生成当前装配相位下的针齿中心。

    坐标系：摆线轮坐标系。
    默认输入角 theta = 0，此时针齿壳中心相对摆线轮中心偏移 -e。
    """

    rp = pin_pcd / 2.0
    phase = math.radians(ring_phase_deg)

    centers = []

    for k in range(pin_count):
        a = 2.0 * math.pi * k / pin_count + phase
        x = rp * math.cos(a) - eccentricity
        y = rp * math.sin(a)
        centers.append([x, y])

    return np.array(centers, dtype=float)


def generate_circle_polyline(center: np.ndarray, radius: float, n: int) -> np.ndarray:
    """生成一个圆的多段线"""
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    x = center[0] + radius * np.cos(t)
    y = center[1] + radius * np.sin(t)
    return np.column_stack((x, y))


def generate_pin_circles(
    pin_centers: np.ndarray,
    pin_radius: float,
    n: int
) -> list[np.ndarray]:
    """生成所有针齿圆"""
    return [
        generate_circle_polyline(center, pin_radius, n)
        for center in pin_centers
    ]


# ============================================================
# 5. CSV / DXF 导出
# ============================================================

def save_csv(filename: str, points: np.ndarray) -> None:
    """保存 CSV"""
    np.savetxt(
        filename,
        points,
        delimiter=",",
        header="x_mm,y_mm",
        comments="",
        fmt="%.8f"
    )


def save_reference_dxf(filename: str, entities: list[tuple[np.ndarray, str, bool]]) -> None:
    """
    参考用 DXF，使用普通 POLYLINE，可放多个轮廓。
    entities:
        [
            (points, layer_name, closed_bool),
            ...
        ]
    """
    with open(filename, "w", encoding="utf-8") as f:
        f.write("0\nSECTION\n2\nHEADER\n")
        f.write("9\n$INSUNITS\n")
        f.write("70\n4\n")  # mm
        f.write("0\nENDSEC\n")

        f.write("0\nSECTION\n2\nENTITIES\n")

        for points, layer_name, closed in entities:
            pts = np.asarray(points, dtype=float)

            if len(pts) < 2:
                continue

            if closed and np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
                pts = pts[:-1]

            f.write("0\nPOLYLINE\n")
            f.write(f"8\n{layer_name}\n")
            f.write("66\n1\n")
            f.write(f"70\n{1 if closed else 0}\n")

            for x, y in pts:
                f.write("0\nVERTEX\n")
                f.write(f"8\n{layer_name}\n")
                f.write(f"10\n{x:.8f}\n")
                f.write(f"20\n{y:.8f}\n")
                f.write("30\n0.0\n")

            f.write("0\nSEQEND\n")

        f.write("0\nENDSEC\n0\nEOF\n")


def save_solidworks_lwpolyline_dxf(
    filename: str,
    points: np.ndarray,
    layer_name: str = "PROFILE"
) -> None:
    """
    输出 SolidWorks 友好的闭合 LWPOLYLINE DXF。

    特点：
    - 只有一个闭合轮廓
    - 无针齿
    - 无文字
    - 无中心线
    - 单位 mm
    - 点数适中
    """
    pts = np.asarray(points, dtype=float)

    if len(pts) < 3:
        raise ValueError("closed profile must have at least 3 points")

    # 去掉末尾重复点
    if np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
        pts = pts[:-1]

    with open(filename, "w", encoding="utf-8") as f:
        # HEADER
        f.write("0\nSECTION\n")
        f.write("2\nHEADER\n")
        f.write("9\n$INSUNITS\n")
        f.write("70\n4\n")  # 4 = millimeters
        f.write("0\nENDSEC\n")

        # ENTITIES
        f.write("0\nSECTION\n")
        f.write("2\nENTITIES\n")

        f.write("0\nLWPOLYLINE\n")
        f.write(f"8\n{layer_name}\n")
        f.write("90\n")
        f.write(f"{len(pts)}\n")
        f.write("70\n1\n")  # closed

        for x, y in pts:
            f.write("10\n")
            f.write(f"{x:.8f}\n")
            f.write("20\n")
            f.write(f"{y:.8f}\n")

        f.write("0\nENDSEC\n")
        f.write("0\nEOF\n")


# ============================================================
# 6. 绘图
# ============================================================

def plot_profile_with_pins(
    profile: np.ndarray,
    pin_centers: np.ndarray,
    pin_circles: list[np.ndarray],
    title: str,
    filename: str,
    profile_label: str
) -> None:
    plt.figure(figsize=(8, 8))

    plt.plot(
        profile[:, 0],
        profile[:, 1],
        linewidth=1.2,
        label=profile_label
    )

    for i, circle in enumerate(pin_circles):
        label = "pins" if i == 0 else None
        plt.plot(circle[:, 0], circle[:, 1], linewidth=0.8, label=label)

    plt.scatter(
        pin_centers[:, 0],
        pin_centers[:, 1],
        s=10,
        label="pin centers"
    )

    # 针齿中心分布圆
    rp = PIN_PCD / 2.0
    t = np.linspace(0, 2 * math.pi, 600)
    pcd_x = rp * np.cos(t) - ECCENTRICITY
    pcd_y = rp * np.sin(t)
    plt.plot(
        pcd_x,
        pcd_y,
        "--",
        linewidth=0.8,
        label="pin PCD in wheel frame"
    )

    # 摆线轮中心
    plt.scatter([0], [0], marker="+", s=30, label="cycloid wheel center")

    # 针齿壳中心
    plt.scatter(
        [-ECCENTRICITY],
        [0],
        marker="x",
        s=30,
        label="pin ring center"
    )

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title(title)
    plt.xlabel("x / mm")
    plt.ylabel("y / mm")
    plt.tight_layout()
    plt.savefig(filename, dpi=300)

    if SHOW_PLOT:
        plt.show()

    plt.close()


def plot_inner_outer_compare(
    inner_profile: np.ndarray,
    outer_profile: np.ndarray,
    pin_centers: np.ndarray,
    pin_circles: list[np.ndarray],
) -> None:
    plt.figure(figsize=(8, 8))

    plt.plot(
        inner_profile[:, 0],
        inner_profile[:, 1],
        linewidth=1.2,
        label=f"inner envelope, teeth={INNER_TEETH}"
    )

    plt.plot(
        outer_profile[:, 0],
        outer_profile[:, 1],
        linewidth=1.2,
        label=f"outer envelope, teeth={OUTER_TEETH}"
    )

    for i, circle in enumerate(pin_circles):
        label = "pins" if i == 0 else None
        plt.plot(circle[:, 0], circle[:, 1], linewidth=0.8, label=label)

    plt.scatter(
        pin_centers[:, 0],
        pin_centers[:, 1],
        s=10,
        label="pin centers"
    )

    rp = PIN_PCD / 2.0
    t = np.linspace(0, 2 * math.pi, 600)
    pcd_x = rp * np.cos(t) - ECCENTRICITY
    pcd_y = rp * np.sin(t)
    plt.plot(pcd_x, pcd_y, "--", linewidth=0.8, label="pin PCD")

    plt.scatter([0], [0], marker="+", s=30, label="cycloid wheel center")
    plt.scatter([-ECCENTRICITY], [0], marker="x", s=30, label="pin ring center")

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("Inner / Outer Cycloid Profiles with Pins")
    plt.xlabel("x / mm")
    plt.ylabel("y / mm")
    plt.tight_layout()
    plt.savefig("inner_outer_with_pins.png", dpi=300)

    if SHOW_PLOT:
        plt.show()

    plt.close()


# ============================================================
# 7. 主程序
# ============================================================

def main():
    print("========== 参数检查 ==========")
    print(f"PIN_COUNT     = {PIN_COUNT}")
    print(f"OUTER_TEETH   = {OUTER_TEETH} = PIN_COUNT - 1")
    print(f"INNER_TEETH   = {INNER_TEETH} = PIN_COUNT + 1")
    print(f"PIN_PCD       = {PIN_PCD}")
    print(f"PIN_RADIUS    = {PIN_RADIUS}")
    print(f"ECCENTRICITY  = {ECCENTRICITY}")
    print(f"OFFSET_MOD    = {OFFSET_MOD}")
    print(f"CLEARANCE     = {CLEARANCE}")
    print(f"SAMPLE_NUM    = {SAMPLE_NUM}")
    print(f"SW_POINTS     = {SW_EXPORT_POINTS}")

    # 外摆线齿廓：Zc = Zp - 1
    outer_profile = generate_pin_envelope_profile(
        pin_count=PIN_COUNT,
        profile_teeth=OUTER_TEETH,
        pin_pcd=PIN_PCD,
        pin_radius=PIN_RADIUS,
        eccentricity=ECCENTRICITY,
        sample_num=SAMPLE_NUM,
        mode="outer",
        phase_deg=OUTER_PROFILE_PHASE_DEG,
        offset_mod=OFFSET_MOD,
        clearance=CLEARANCE,
    )

    # 内摆线齿廓：Zc = Zp + 1
    inner_profile = generate_pin_envelope_profile(
        pin_count=PIN_COUNT,
        profile_teeth=INNER_TEETH,
        pin_pcd=PIN_PCD,
        pin_radius=PIN_RADIUS,
        eccentricity=ECCENTRICITY,
        sample_num=SAMPLE_NUM,
        mode="inner",
        phase_deg=INNER_PROFILE_PHASE_DEG,
        offset_mod=OFFSET_MOD,
        clearance=CLEARANCE,
    )

    # 重采样，生成 SolidWorks 拉伸用轮廓
    outer_sw = resample_closed_curve(outer_profile, SW_EXPORT_POINTS)
    inner_sw = resample_closed_curve(inner_profile, SW_EXPORT_POINTS)

    # 生成针齿
    outer_pin_centers = generate_pin_centers(
        pin_count=PIN_COUNT,
        pin_pcd=PIN_PCD,
        eccentricity=ECCENTRICITY,
        ring_phase_deg=OUTER_PIN_PHASE_DEG,
    )

    inner_pin_centers = generate_pin_centers(
        pin_count=PIN_COUNT,
        pin_pcd=PIN_PCD,
        eccentricity=ECCENTRICITY,
        ring_phase_deg=INNER_PIN_PHASE_DEG,
    )

    outer_pin_circles = generate_pin_circles(
        pin_centers=outer_pin_centers,
        pin_radius=PIN_RADIUS,
        n=PIN_CIRCLE_POINTS,
    )

    inner_pin_circles = generate_pin_circles(
        pin_centers=inner_pin_centers,
        pin_radius=PIN_RADIUS,
        n=PIN_CIRCLE_POINTS,
    )

    # ========================================================
    # K1 系数 / 重要参数报告
    # ========================================================

    parameter_report = {
        "outer_design": calculate_design_parameters(
            pin_count=PIN_COUNT,
            profile_teeth=OUTER_TEETH,
            pin_pcd=PIN_PCD,
            pin_radius=PIN_RADIUS,
            eccentricity=ECCENTRICITY,
            offset_mod=OFFSET_MOD,
            clearance=CLEARANCE,
            mode="outer",
        ),
        "inner_design": calculate_design_parameters(
            pin_count=PIN_COUNT,
            profile_teeth=INNER_TEETH,
            pin_pcd=PIN_PCD,
            pin_radius=PIN_RADIUS,
            eccentricity=ECCENTRICITY,
            offset_mod=OFFSET_MOD,
            clearance=CLEARANCE,
            mode="inner",
        ),
        "outer_profile": analyze_closed_profile(
            profile=outer_sw,
            pin_centers=outer_pin_centers,
            pin_radius=PIN_RADIUS,
            name="outer",
        ),
        "inner_profile": analyze_closed_profile(
            profile=inner_sw,
            pin_centers=inner_pin_centers,
            pin_radius=PIN_RADIUS,
            name="inner",
        ),
    }
    parameter_report["warnings"] = collect_report_warnings(parameter_report)

    print_parameter_report(parameter_report)
    save_parameter_report_txt("cycloid_parameter_report.txt", parameter_report)
    save_parameter_report_csv("cycloid_parameter_report.csv", parameter_report)
    save_parameter_report_json("cycloid_parameter_report.json", parameter_report)

    # ========================================================
    # CSV 输出
    # ========================================================

    save_csv("outer_profile_full.csv", outer_profile)
    save_csv("inner_profile_full.csv", inner_profile)
    save_csv("SW_outer_profile_resampled.csv", outer_sw)
    save_csv("SW_inner_profile_resampled.csv", inner_sw)
    save_csv("outer_pins.csv", outer_pin_centers)
    save_csv("inner_pins.csv", inner_pin_centers)

    # ========================================================
    # SolidWorks 拉伸用 DXF：单闭合轮廓，不带针齿
    # ========================================================

    save_solidworks_lwpolyline_dxf(
        filename=f"SW_outer_{OUTER_TEETH}teeth_closed_profile.dxf",
        points=outer_sw,
        layer_name="OUTER_PROFILE"
    )

    save_solidworks_lwpolyline_dxf(
        filename=f"SW_inner_{INNER_TEETH}teeth_closed_profile.dxf",
        points=inner_sw,
        layer_name="INNER_PROFILE"
    )

    # ========================================================
    # 参考用 DXF：带针齿，不建议直接拉伸
    # ========================================================

    save_reference_dxf(
        "outer_with_pins_reference.dxf",
        [(outer_sw, "OUTER_PROFILE", True)]
        + [(circle, "OUTER_PINS", True) for circle in outer_pin_circles]
    )

    save_reference_dxf(
        "inner_with_pins_reference.dxf",
        [(inner_sw, "INNER_PROFILE", True)]
        + [(circle, "INNER_PINS", True) for circle in inner_pin_circles]
    )

    save_reference_dxf(
        "inner_outer_with_pins_reference.dxf",
        [
            (inner_sw, "INNER_PROFILE", True),
            (outer_sw, "OUTER_PROFILE", True),
        ]
        + [(circle, "PINS", True) for circle in outer_pin_circles]
    )

    save_reference_dxf(
        "pins_only_reference.dxf",
        [(circle, "PINS", True) for circle in outer_pin_circles]
    )

    # ========================================================
    # PNG 输出
    # ========================================================

    plot_profile_with_pins(
        profile=outer_sw,
        pin_centers=outer_pin_centers,
        pin_circles=outer_pin_circles,
        title=f"Outer Cycloid Envelope with Pins: {PIN_COUNT} pins / {OUTER_TEETH} teeth",
        filename="outer_with_pins.png",
        profile_label="outer envelope"
    )

    plot_profile_with_pins(
        profile=inner_sw,
        pin_centers=inner_pin_centers,
        pin_circles=inner_pin_circles,
        title=f"Inner Cycloid Envelope with Pins: {PIN_COUNT} pins / {INNER_TEETH} teeth",
        filename="inner_with_pins.png",
        profile_label="inner envelope"
    )

    plot_inner_outer_compare(
        inner_profile=inner_sw,
        outer_profile=outer_sw,
        pin_centers=outer_pin_centers,
        pin_circles=outer_pin_circles,
    )

    print("\n========== 已输出 ==========")
    print("SolidWorks 拉伸用：")
    print(f"SW_outer_{OUTER_TEETH}teeth_closed_profile.dxf")
    print(f"SW_inner_{INNER_TEETH}teeth_closed_profile.dxf")

    print("\n参数报告：")
    print("cycloid_parameter_report.txt")
    print("cycloid_parameter_report.csv")
    print("cycloid_parameter_report.json")

    print("\nCSV：")
    print("outer_profile_full.csv")
    print("inner_profile_full.csv")
    print("SW_outer_profile_resampled.csv")
    print("SW_inner_profile_resampled.csv")
    print("outer_pins.csv")
    print("inner_pins.csv")

    print("\n参考 DXF：")
    print("outer_with_pins_reference.dxf")
    print("inner_with_pins_reference.dxf")
    print("inner_outer_with_pins_reference.dxf")
    print("pins_only_reference.dxf")

    print("\nPNG：")
    print("outer_with_pins.png")
    print("inner_with_pins.png")
    print("inner_outer_with_pins.png")

    print("\nSolidWorks 使用建议：")
    print("1. 优先打开 SW_outer / SW_inner 的 closed_profile.dxf")
    print("2. Import to new part as 2D sketch")
    print("3. 单位选 mm")
    print("4. 不要用 with_pins_reference 文件直接拉伸")
    print("5. 如果草图卡顿，把 SW_EXPORT_POINTS 改成 800")


if __name__ == "__main__":
    main()