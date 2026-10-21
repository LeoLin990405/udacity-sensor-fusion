#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
from pathlib import Path


def finite_values(rows, field):
    values = []
    for row in rows:
        try:
            value = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0.0:
            values.append(value)
    return values


def main():
    parser = argparse.ArgumentParser(description="Merge TTC benchmark shards and summarize lead-object estimates")
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("results/ttc_benchmark.csv"))
    parser.add_argument("--summary", type=Path, default=Path("results/ttc_summary.csv"))
    args = parser.parse_args()

    files = sorted({path for root in args.inputs for path in root.rglob("*.csv")})
    rows = []
    fieldnames = None
    for path in files:
        with path.open(newline="") as source:
            reader = csv.DictReader(source)
            fieldnames = fieldnames or reader.fieldnames
            rows.extend(reader)
    if not rows or not fieldnames:
        raise SystemExit("No benchmark rows found")

    rows.sort(key=lambda row: (row["detector"], row["descriptor"], int(row["current_frame"]), int(row["current_box"])))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    grouped = {}
    for row in rows:
        if row.get("lead_object") != "1":
            continue
        grouped.setdefault((row["detector"], row["descriptor"]), []).append(row)

    summary_fields = [
        "detector", "descriptor", "lead_rows", "valid_lidar", "median_lidar", "stdev_lidar",
        "valid_camera", "median_camera", "stdev_camera", "median_absolute_sensor_gap",
        "mean_absolute_sensor_gap", "median_relative_sensor_gap", "camera_outliers",
    ]
    with args.summary.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=summary_fields)
        writer.writeheader()
        for (detector, descriptor), group in sorted(grouped.items()):
            lidar = finite_values(group, "ttc_lidar")
            camera = finite_values(group, "ttc_camera")
            gaps = []
            relative_gaps = []
            camera_outliers = 0
            for row in group:
                try:
                    lidar_value = float(row["ttc_lidar"])
                    camera_value = float(row["ttc_camera"])
                except (TypeError, ValueError):
                    camera_outliers += 1
                    continue
                if math.isfinite(lidar_value) and math.isfinite(camera_value) and lidar_value > 0.0 and camera_value > 0.0:
                    gaps.append(abs(lidar_value - camera_value))
                    relative_gap = abs(lidar_value - camera_value) / lidar_value
                    relative_gaps.append(relative_gap)
                    if relative_gap > 0.25:
                        camera_outliers += 1
                else:
                    camera_outliers += 1
            writer.writerow({
                "detector": detector,
                "descriptor": descriptor,
                "lead_rows": len(group),
                "valid_lidar": len(lidar),
                "median_lidar": statistics.median(lidar) if lidar else "",
                "stdev_lidar": statistics.pstdev(lidar) if len(lidar) > 1 else "",
                "valid_camera": len(camera),
                "median_camera": statistics.median(camera) if camera else "",
                "stdev_camera": statistics.pstdev(camera) if len(camera) > 1 else "",
                "median_absolute_sensor_gap": statistics.median(gaps) if gaps else "",
                "mean_absolute_sensor_gap": statistics.fmean(gaps) if gaps else "",
                "median_relative_sensor_gap": statistics.median(relative_gaps) if relative_gaps else "",
                "camera_outliers": camera_outliers,
            })

    print(f"Merged {len(files)} files and {len(rows)} rows into {args.output}")


if __name__ == "__main__":
    main()
