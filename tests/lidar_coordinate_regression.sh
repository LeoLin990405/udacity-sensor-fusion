#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_file="${project_root}/c3-main.cpp"

expected_conversion='PointT(-detection.y, detection.x, -detection.z)'

if ! grep -Fq "${expected_conversion}" "${source_file}"; then
  echo "Missing the Udacity CARLA-to-PCL coordinate conversion" >&2
  exit 1
fi

echo "LiDAR coordinate conversion regression check passed"
