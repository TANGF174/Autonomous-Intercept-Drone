#!/usr/bin/env python3
"""从视觉 PNG 仿真 CSV 中绘制截至第一次擦肩的像素误差和俯视轨迹。"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import os
from pathlib import Path
import sys
from typing import Dict

# 这台仿真机的用户目录中是 NumPy 2.x，而 Ubuntu 系统 Matplotlib 按
# NumPy 1.x 编译。发现这种混装时自动禁用 user-site 并重启一次。
if os.environ.get("PYTHONNOUSERSITE") != "1":
    numpy_spec = importlib.util.find_spec("numpy")
    matplotlib_spec = importlib.util.find_spec("matplotlib")
    numpy_path = str(numpy_spec.origin) if numpy_spec and numpy_spec.origin else ""
    matplotlib_path = (
        str(matplotlib_spec.origin) if matplotlib_spec and matplotlib_spec.origin else ""
    )
    if "/.local/" in numpy_path and "/usr/lib/python3/" in matplotlib_path:
        environment = os.environ.copy()
        environment["PYTHONNOUSERSITE"] = "1"
        os.execve(sys.executable, [sys.executable, *sys.argv], environment)

import matplotlib

matplotlib.use("Agg")

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


REQUIRED_COLUMNS = {
    "time_s",
    "self_x",
    "self_y",
    "target_x",
    "target_y",
    "dist_m",
    "ex",
    "ey",
    "detect_w",
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="使用保存的视觉 PNG 数据，生成截至第一次擦肩的两张图。"
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=repo_root / "outputs" / "vpng_intercept_stats.csv",
        help="输入 CSV（默认：outputs/vpng_intercept_stats.csv）",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "outputs" / "plots_first_miss",
        help="图片输出目录",
    )
    parser.add_argument("--image-width", type=int, default=1280, help="图像宽度，默认 1280")
    parser.add_argument("--image-height", type=int, default=720, help="图像高度，默认 720")
    parser.add_argument("--dpi", type=int, default=300, help="输出分辨率，默认 300 dpi")
    return parser.parse_args()


DataColumns = Dict[str, np.ndarray]


def load_data(csv_path: Path) -> DataColumns:
    if not csv_path.is_file():
        raise FileNotFoundError(f"找不到 CSV：{csv_path}")

    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = [row for row in csv.DictReader(line for line in stream if not line.startswith("#"))]
    if not rows:
        raise ValueError("CSV 中没有可绘制的数据")

    missing = REQUIRED_COLUMNS.difference(rows[0])
    if missing:
        raise ValueError(f"CSV 缺少列：{', '.join(sorted(missing))}")

    data = {
        name: np.asarray([float(row[name]) for row in rows if row.get("time_s")], dtype=float)
        for name in rows[0]
    }
    data["t"] = data["time_s"] - data["time_s"][0]
    return data


def find_first_miss(data: DataColumns) -> tuple[int, int, int]:
    """返回（最后有效检测、首次丢框记录、第一次擦肩最近点）的行号。"""
    valid = data["detect_w"] >= 0
    first_loss_candidates = np.flatnonzero((~valid) & np.r_[False, valid[:-1]])
    if first_loss_candidates.size == 0:
        raise ValueError("数据中没有出现由有效检测转为丢框的记录，无法定位第一次错过")

    first_loss = int(first_loss_candidates[0])
    last_valid = first_loss - 1

    # 第一次丢框到重新捕获之间为第一次擦肩区间。该区间内真实距离的
    # 最小值就是第一次错过的最近点；不使用后续第二次接近的数据。
    reacquired = np.flatnonzero(valid[first_loss:])
    encounter_end = first_loss + int(reacquired[0]) if reacquired.size else len(data["time_s"])
    miss_slice = data["dist_m"][last_valid:encounter_end]
    closest = last_valid + int(np.nanargmin(miss_slice))
    return last_valid, first_loss, closest


def configure_chinese_font() -> None:
    noto_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if noto_path.is_file():
        font_manager.fontManager.addfont(str(noto_path))
        chinese_family = font_manager.FontProperties(fname=str(noto_path)).get_name()
    else:
        chinese_family = "Noto Sans CJK SC"
    plt.rcParams["font.sans-serif"] = [
        chinese_family,
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "WenQuanYi Micro Hei",
        "SimHei",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def plot_pixel_error(
    data: DataColumns,
    last_valid: int,
    closest: int,
    image_width: int,
    image_height: int,
    output_path: Path,
    dpi: int,
) -> None:
    # 丢框后 CSV 中的 ex/ey 是缓存旧值，不能作为新视觉测量绘制。
    vision = {name: values[: last_valid + 1] for name, values in data.items()}
    t_loss = float(data["t"][last_valid])
    t_closest = float(data["t"][closest])

    fig, (ax_time, ax_track) = plt.subplots(1, 2, figsize=(13, 5.4))

    ax_time.plot(vision["t"], vision["ex"], "b-", linewidth=1.3, label="ex（水平）")
    ax_time.plot(vision["t"], vision["ey"], "r-", linewidth=1.3, label="ey（垂直）")
    ax_time.axhline(0, color="k", linestyle=":", linewidth=1)
    ax_time.axvline(t_loss, color="k", linestyle="--", linewidth=1.3, label="首次丢框")
    ax_time.set_xlim(left=0, right=max(t_closest, t_loss) * 1.015)
    ax_time.set_xlabel("时间 (s)")
    ax_time.set_ylabel("像素误差 (px)")
    ax_time.set_title("视场像素误差（仅有效检测）")
    ax_time.grid(True, alpha=0.7)
    ax_time.legend(loc="best")

    scatter = ax_track.scatter(
        vision["ex"], vision["ey"], s=9, c=vision["t"], cmap="jet", linewidths=0
    )
    colorbar = fig.colorbar(scatter, ax=ax_track)
    colorbar.set_label("时间 (s)")
    half_width = image_width / 2.0
    half_height = image_height / 2.0
    ax_track.axhline(0, color="k", linestyle=":")
    ax_track.axvline(0, color="k", linestyle=":")
    ax_track.add_patch(
        patches.Rectangle(
            (-half_width, -half_height),
            image_width,
            image_height,
            linewidth=1.4,
            edgecolor="k",
            facecolor="none",
        )
    )
    ax_track.set_xlim(-half_width, half_width)
    ax_track.set_ylim(-half_height, half_height)
    ax_track.set_xlabel("ex (px)")
    ax_track.set_ylabel("ey (px)")
    ax_track.set_title("检测中心在图像中的运动轨迹")
    ax_track.set_aspect("equal", adjustable="box")
    ax_track.grid(True, alpha=0.7)

    fig.suptitle(
        f"截至第一次错过：首次丢框 t = {t_loss:.2f} s，最近点 t = {t_closest:.2f} s",
        fontsize=15,
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def plot_top_view(
    data: DataColumns,
    last_valid: int,
    closest: int,
    output_path: Path,
    dpi: int,
) -> None:
    encounter = {name: values[: closest + 1] for name, values in data.items()}
    min_dist = float(data["dist_m"][closest])
    t_closest = float(data["t"][closest])

    self_e = encounter["self_y"]
    self_n = encounter["self_x"]
    target_e = encounter["target_y"]
    target_n = encounter["target_x"]
    miss_self_e = float(data["self_y"][closest])
    miss_self_n = float(data["self_x"][closest])
    miss_target_e = float(data["target_y"][closest])
    miss_target_n = float(data["target_x"][closest])
    loss_self_e = float(data["self_y"][last_valid])
    loss_self_n = float(data["self_x"][last_valid])
    loss_target_e = float(data["target_y"][last_valid])
    loss_target_n = float(data["target_x"][last_valid])
    t_loss = float(data["t"][last_valid])

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(self_e, self_n, "b-", linewidth=2.2, label="拦截机")
    ax.plot(target_e, target_n, "r--", linewidth=2.2, label="目标机")
    ax.plot(self_e[0], self_n[0], "bs", markersize=10, label="拦截机起点")
    ax.plot(target_e[0], target_n[0], "rs", markersize=10, label="目标机起点")
    ax.plot(
        miss_self_e,
        miss_self_n,
        "k*",
        markersize=13,
        label=f"第一次错过最近点 ({min_dist:.3f} m)",
    )
    ax.plot(
        loss_self_e,
        loss_self_n,
        marker="X",
        color="darkorange",
        markeredgecolor="black",
        markeredgewidth=0.5,
        markersize=7,
        linestyle="none",
        zorder=6,
        label=f"丢失时拦截机 (t≈{t_loss:.2f} s)",
    )
    ax.plot(
        loss_target_e,
        loss_target_n,
        marker="o",
        markerfacecolor="none",
        markeredgecolor="darkorange",
        markeredgewidth=1.4,
        markersize=6,
        linestyle="none",
        zorder=6,
        label=f"丢失时目标机 (t≈{t_loss:.2f} s)",
    )
    ax.plot(
        [loss_self_e, loss_target_e],
        [loss_self_n, loss_target_n],
        color="darkorange",
        linestyle="--",
        linewidth=1.0,
        alpha=0.9,
        zorder=5,
    )
    ax.annotate(
        f"视野丢失\nt≈{t_loss:.2f} s",
        xy=((loss_self_e + loss_target_e) / 2, (loss_self_n + loss_target_n) / 2),
        xytext=(-58, 32),
        textcoords="offset points",
        ha="center",
        va="bottom",
        color="darkorange",
        fontsize=8.5,
        fontweight="bold",
        arrowprops={
            "arrowstyle": "->",
            "color": "darkorange",
            "linewidth": 1.1,
            "shrinkA": 3,
            "shrinkB": 4,
        },
        zorder=7,
    )

    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(
        miss_target_e + min_dist * np.cos(theta),
        miss_target_n + min_dist * np.sin(theta),
        "k:",
        linewidth=1.4,
        label="最近接圆",
    )
    ax.plot(
        [miss_self_e, miss_target_e],
        [miss_self_n, miss_target_n],
        color="0.35",
        linestyle=":",
        linewidth=1.1,
    )

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"俯视拦截轨迹（截至第一次错过） | 最近接 = {min_dist:.3f} m @ t = {t_closest:.2f} s")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.7)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    configure_chinese_font()
    data = load_data(args.csv)
    last_valid, first_loss, closest = find_first_miss(data)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    pixel_path = args.output_dir / "5_Pixel_Error_Until_First_Miss.png"
    top_path = args.output_dir / "6_2D_Top_View_Until_First_Miss.png"
    plot_pixel_error(
        data,
        last_valid,
        closest,
        args.image_width,
        args.image_height,
        pixel_path,
        args.dpi,
    )
    plot_top_view(data, last_valid, closest, top_path, args.dpi)

    print(f"输入数据：{args.csv.resolve()}")
    print(f"最后有效检测：t={data['t'][last_valid]:.4f} s（行 {last_valid + 2}）")
    print(f"首次丢框记录：t={data['t'][first_loss]:.4f} s（行 {first_loss + 2}）")
    print(
        f"第一次错过最近点：t={data['t'][closest]:.4f} s，"
        f"距离={data['dist_m'][closest]:.4f} m（行 {closest + 2}）"
    )
    print(f"已生成：{pixel_path.resolve()}")
    print(f"已生成：{top_path.resolve()}")


if __name__ == "__main__":
    main()
