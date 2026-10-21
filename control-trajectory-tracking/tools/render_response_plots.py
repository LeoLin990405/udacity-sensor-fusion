#!/usr/bin/env python3
import csv
import os
import sys

from PIL import Image, ImageDraw, ImageFont


COLORS = {
    "P": "#8f99a8",
    "PI": "#e09f3e",
    "PID": "#126782",
    "target": "#c1292e",
    "error": "#126782",
    "command": "#2f5d50",
}


def load_rows(path):
    grouped = {"P": [], "PI": [], "PID": []}
    with open(path, newline="", encoding="ascii") as source:
        for row in csv.DictReader(source):
            grouped[row["controller"]].append(
                {key: float(value) for key, value in row.items()
                 if key != "controller"}
            )
    return grouped


def font(size, bold=False):
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold else
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default(size=size)


def draw_panel(draw, box, title, series, y_min, y_max):
    left, top, right, bottom = box
    draw.text((left, top - 38), title, fill="#1f2933", font=font(21, True))
    draw.rectangle(box, fill="#ffffff", outline="#9aa5b1", width=1)

    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = bottom - int((bottom - top) * fraction)
        draw.line((left, y, right, y), fill="#e4e7eb", width=1)
        value = y_min + (y_max - y_min) * fraction
        draw.text((left - 58, y - 8), f"{value:.1f}", fill="#52606d",
                  font=font(13))

    max_step = max(point[0] for _, points in series for point in points)

    def transform(point):
        step, value = point
        x = left + int((right - left) * step / max_step)
        clipped = max(y_min, min(value, y_max))
        y = bottom - int((bottom - top) * (clipped - y_min) /
                         (y_max - y_min))
        return x, y

    for label, points in series:
        coordinates = [transform(point) for point in points]
        draw.line(coordinates, fill=COLORS[label], width=3)

    legend_x = right - 255
    for index, (label, _) in enumerate(series):
        y = top + 14 + index * 25
        draw.line((legend_x, y + 7, legend_x + 34, y + 7),
                  fill=COLORS[label], width=4)
        draw.text((legend_x + 44, y), label, fill="#1f2933", font=font(14))


def render(csv_path, output_path, title, actual_limits, error_limits):
    rows = load_rows(csv_path)
    image = Image.new("RGB", (1280, 820), "#f5f7fa")
    draw = ImageDraw.Draw(image)
    draw.text((70, 28), title, fill="#102a43", font=font(30, True))
    draw.text((70, 70),
              "Production PID implementation, deterministic closed-loop plant",
              fill="#52606d", font=font(16))

    actual_series = [
        (name, [(row["step"], row["actual"]) for row in rows[name]])
        for name in ("P", "PI", "PID")
    ]
    actual_series.append(
        ("target", [(row["step"], row["target"]) for row in rows["PID"]])
    )
    tuned_series = [
        ("error", [(row["step"], row["error"]) for row in rows["PID"]]),
        ("command", [(row["step"], row["command"]) for row in rows["PID"]]),
    ]

    draw_panel(draw, (95, 145, 1210, 425), "Gain comparison: tracked value",
               actual_series, *actual_limits)
    draw_panel(draw, (95, 510, 1210, 790), "Selected PID: error and command",
               tuned_series, *error_limits)
    image.save(output_path)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("Usage: render_response_plots.py DATA_DIR PLOTS_DIR")
    data_dir, plots_dir = sys.argv[1:]
    os.makedirs(plots_dir, exist_ok=True)
    render(os.path.join(data_dir, "throttle_gain_sweep.csv"),
           os.path.join(plots_dir, "throttle_pid.png"),
           "Throttle PID gain experiment", (-1.2, 9.5), (-3.5, 3.5))
    render(os.path.join(data_dir, "steering_gain_sweep.csv"),
           os.path.join(plots_dir, "steer_pid.png"),
           "Steering PID gain experiment", (-0.7, 0.9), (-1.4, 1.4))


if __name__ == "__main__":
    main()
