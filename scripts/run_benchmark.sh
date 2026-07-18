#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BINARY="${1:-$ROOT_DIR/build/3D_object_tracking}"
OUTPUT_DIR="${2:-$ROOT_DIR/results/raw}"
SHARD_INDEX="${3:-0}"
SHARD_COUNT="${4:-1}"

detectors=(SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT)
descriptors=(BRISK BRIEF ORB FREAK AKAZE SIFT)

mkdir -p "$OUTPUT_DIR"
valid_index=0
for detector in "${detectors[@]}"; do
  for descriptor in "${descriptors[@]}"; do
    if [[ "$descriptor" == "AKAZE" && "$detector" != "AKAZE" ]]; then
      continue
    fi
    if [[ "$detector" == "SIFT" && "$descriptor" == "ORB" ]]; then
      continue
    fi
    if (( valid_index % SHARD_COUNT == SHARD_INDEX )); then
      output="$OUTPUT_DIR/${detector,,}_${descriptor,,}.csv"
      "$BINARY" "$detector" "$descriptor" --data "$ROOT_DIR" --output "$output"
    fi
    ((valid_index += 1))
  done
done
