#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path


DETECTOR_ORDER = {name: index for index, name in enumerate(
    ("SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT")
)}
DESCRIPTOR_ORDER = {name: index for index, name in enumerate(
    ("BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT")
)}


def read_rows(paths):
    rows = []
    fieldnames = None
    for path in paths:
        with path.open(newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)
            if fieldnames is None:
                fieldnames = reader.fieldnames
            elif reader.fieldnames != fieldnames:
                raise ValueError(f"CSV columns do not match: {path}")
            rows.extend(reader)
    return fieldnames, rows


def write_rows(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="Merge benchmark CSV files from cluster workers.")
    parser.add_argument("--summaries", type=Path, nargs="+", required=True)
    parser.add_argument("--frames", type=Path, nargs="+", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    summary_fields, summaries = read_rows(args.summaries)
    frame_fields, frames = read_rows(args.frames)

    summaries.sort(key=lambda row: (
        DETECTOR_ORDER[row["detector"]], DESCRIPTOR_ORDER[row["descriptor"]]
    ))
    frames.sort(key=lambda row: (
        DETECTOR_ORDER[row["detector"]], DESCRIPTOR_ORDER[row["descriptor"]], int(row["image"])
    ))

    write_rows(args.output_dir / "feature_tracking_benchmark.csv", summary_fields, summaries)
    write_rows(args.output_dir / "feature_tracking_frames.csv", frame_fields, frames)

    fastest = sorted(summaries, key=lambda row: float(row["total_ms"]))[:3]
    for rank, row in enumerate(fastest, start=1):
        print(f"{rank}. {row['detector']} + {row['descriptor']}: "
              f"{row['total_ms']} ms, {row['avg_matches']} mean matches")


if __name__ == "__main__":
    main()
