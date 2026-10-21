#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DETECTORS = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
DESCRIPTORS = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]


def numeric(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value


def read_lead_rows(path):
    with path.open(newline="") as source:
        rows = [row for row in csv.DictReader(source) if row.get("lead_object") == "1"]
    rows.sort(key=lambda row: (row["detector"], row["descriptor"], int(row["current_frame"])))
    return rows


def group_rows(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["detector"], row["descriptor"])].append(row)
    return grouped


def combination_metrics(grouped):
    metrics = {}
    for combination, rows in grouped.items():
        relative_errors = []
        invalid = 0
        outliers = 0
        for row in rows:
            lidar = numeric(row, "ttc_lidar")
            camera = numeric(row, "ttc_camera")
            if not math.isfinite(camera) or camera <= 0.0:
                invalid += 1
                outliers += 1
                continue
            relative_error = abs(camera - lidar) / lidar
            relative_errors.append(relative_error)
            if relative_error > 0.25:
                outliers += 1
        metrics[combination] = {
            "median_relative_error": statistics.median(relative_errors) if relative_errors else math.inf,
            "invalid": invalid,
            "outliers": outliers,
        }
    return metrics


def plot_best_combinations(grouped, metrics, output):
    ranked = sorted(grouped, key=lambda combo: (metrics[combo]["median_relative_error"], metrics[combo]["outliers"]))
    selected = ranked[:5]
    reference = grouped[selected[0]]
    frames = [int(row["current_frame"]) for row in reference]
    lidar = [numeric(row, "ttc_lidar") for row in reference]

    fig, axis = plt.subplots(figsize=(12, 6.5))
    axis.plot(frames, lidar, color="#111111", linewidth=3.0, marker="o", label="Lidar median TTC")
    colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00"]
    for color, combination in zip(colors, selected):
        rows = grouped[combination]
        camera = [numeric(row, "ttc_camera") for row in rows]
        label = f"{combination[0]}+{combination[1]}"
        axis.plot(frames, camera, linewidth=1.8, marker=".", color=color, label=label)
    axis.set_title("Lead-vehicle TTC: five camera combinations with lowest median relative gap")
    axis.set_xlabel("Current frame")
    axis.set_ylabel("TTC (seconds)")
    axis.set_xticks(frames)
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_error_heatmap(metrics, output):
    matrix = np.full((len(DETECTORS), len(DESCRIPTORS)), np.nan)
    for row, detector in enumerate(DETECTORS):
        for column, descriptor in enumerate(DESCRIPTORS):
            value = metrics.get((detector, descriptor), {}).get("median_relative_error", math.nan)
            if math.isfinite(value):
                matrix[row, column] = 100.0 * value

    fig, axis = plt.subplots(figsize=(10, 7))
    image = axis.imshow(matrix, cmap="viridis_r", vmin=0, vmax=min(50, np.nanmax(matrix)))
    axis.set_xticks(range(len(DESCRIPTORS)), DESCRIPTORS)
    axis.set_yticks(range(len(DETECTORS)), DETECTORS)
    axis.set_xlabel("Descriptor")
    axis.set_ylabel("Detector")
    axis.set_title("Median absolute Camera-Lidar TTC gap (% of Lidar TTC)")
    for row in range(matrix.shape[0]):
        for column in range(matrix.shape[1]):
            value = matrix[row, column]
            label = "N/A" if not math.isfinite(value) else f"{value:.1f}%"
            axis.text(column, row, label, ha="center", va="center", fontsize=8,
                      color="white" if math.isfinite(value) and value > 22 else "black")
    fig.colorbar(image, ax=axis, label="Median relative gap (%)")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_failure_examples(grouped, output):
    examples = [("SIFT", "BRISK"), ("ORB", "BRIEF"), ("HARRIS", "SIFT")]
    reference = grouped[examples[0]]
    frames = [int(row["current_frame"]) for row in reference]
    lidar = [numeric(row, "ttc_lidar") for row in reference]

    fig, axes = plt.subplots(len(examples), 1, figsize=(12, 10), sharex=True)
    for axis, combination in zip(axes, examples):
        camera = [numeric(row, "ttc_camera") for row in grouped[combination]]
        axis.plot(frames, lidar, color="#222222", linewidth=2.4, marker="o", label="Lidar")
        axis.plot(frames, camera, color="#D55E00", linewidth=1.8, marker=".", label="Camera")
        axis.set_title(f"{combination[0]}+{combination[1]}")
        axis.set_ylabel("TTC (s)")
        axis.set_ylim(-5, 45)
        axis.grid(True, alpha=0.25)
        axis.legend(frameon=False, loc="upper right")
    axes[-1].set_xlabel("Current frame")
    axes[-1].set_xticks(frames)
    fig.suptitle("Stable and failure-prone camera TTC examples (display clipped to -5..45 s)", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=180)
    plt.close(fig)


def load_lidar(path):
    points = np.fromfile(path, dtype=np.float32)
    if points.size % 4 != 0:
        raise ValueError(f"Invalid KITTI point cloud: {path}")
    return points.reshape((-1, 4))


def plot_lidar_anomalies(grouped, data_root, output):
    reference = {int(row["current_frame"]): row for row in grouped[("FAST", "BRIEF")]}
    frame_pairs = [(11, 12), (14, 15), (17, 18)]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5), sharex=True, sharey=True)
    base = data_root / "images/KITTI/2011_09_26/velodyne_points/data"

    for axis, (previous_frame, current_frame) in zip(axes, frame_pairs):
        row = reference[current_frame]
        previous_distance = numeric(row, "lidar_distance_previous")
        current_distance = numeric(row, "lidar_distance_current")
        for frame, color, label in [
            (previous_frame, "#0072B2", f"frame {previous_frame}"),
            (current_frame, "#D55E00", f"frame {current_frame}"),
        ]:
            points = load_lidar(base / f"{frame:010d}.bin")
            mask = (
                (points[:, 0] >= 6.0) & (points[:, 0] <= 10.5) &
                (np.abs(points[:, 1]) <= 2.0) &
                (points[:, 2] >= -1.5) & (points[:, 2] <= -0.9) &
                (points[:, 3] >= 0.1)
            )
            selected = points[mask]
            axis.scatter(selected[:, 1], selected[:, 0], s=7, alpha=0.45, color=color, label=label)
        axis.axhline(previous_distance, color="#0072B2", linewidth=2.0, linestyle="--")
        axis.axhline(current_distance, color="#D55E00", linewidth=2.0, linestyle="--")
        axis.set_title(f"{previous_frame}->{current_frame}: Lidar TTC {numeric(row, 'ttc_lidar'):.2f}s")
        axis.set_xlabel("Lateral y (m)")
        axis.grid(True, alpha=0.2)
        axis.legend(frameon=False, fontsize=8)
    axes[0].set_ylabel("Forward x (m), nearer at bottom")
    fig.suptitle("Raw ego-lane Lidar top views at three low-TTC jumps", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Create TTC figures for the final report")
    parser.add_argument("--input", type=Path, default=Path("results/ttc_benchmark.csv"))
    parser.add_argument("--data-root", type=Path, default=Path("."))
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = read_lead_rows(args.input)
    grouped = group_rows(rows)
    metrics = combination_metrics(grouped)
    plot_best_combinations(grouped, metrics, args.output_dir / "ttc_best_combinations.png")
    plot_error_heatmap(metrics, args.output_dir / "ttc_error_heatmap.png")
    plot_failure_examples(grouped, args.output_dir / "ttc_failure_examples.png")
    plot_lidar_anomalies(grouped, args.data_root, args.output_dir / "lidar_anomaly_topviews.png")
    print(f"Created four figures in {args.output_dir}")


if __name__ == "__main__":
    main()
