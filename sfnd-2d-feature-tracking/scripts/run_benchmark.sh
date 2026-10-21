#!/usr/bin/env bash

set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
binary="${1:-$repo_root/build/2D_feature_tracking}"
data_root="${2:-$repo_root/}"
result_dir="${3:-$repo_root/results}"

if [[ ! -x "$binary" ]]; then
    echo "Benchmark binary is not executable: $binary" >&2
    exit 1
fi

mkdir -p "$result_dir"
summary_csv="$result_dir/feature_tracking_benchmark.csv"
frame_csv="$result_dir/feature_tracking_frames.csv"

echo "detector,descriptor,avg_keypoints,avg_matches,total_detector_ms,total_descriptor_ms,total_ms" > "$summary_csv"
echo "image,detector,descriptor,keypoints,mean_keypoint_size,matches,detector_ms,descriptor_ms" > "$frame_csv"

if [[ -n "${DETECTORS:-}" ]]; then
    read -r -a detectors <<< "$DETECTORS"
else
    detectors=(SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT)
fi
if [[ -n "${DESCRIPTORS:-}" ]]; then
    read -r -a descriptors <<< "$DESCRIPTORS"
else
    descriptors=(BRISK BRIEF ORB FREAK AKAZE SIFT)
fi

for detector in "${detectors[@]}"; do
    for descriptor in "${descriptors[@]}"; do
        if [[ "$descriptor" == "AKAZE" && "$detector" != "AKAZE" ]]; then
            continue
        fi

        echo "Running $detector + $descriptor" >&2
        if run_output="$($binary "$detector" "$descriptor" "$data_root" 2>&1)"; then
            summary="$(printf '%s\n' "$run_output" | awk -F, '$1 == "SUMMARY" { print; exit }')"
            if [[ -z "$summary" ]]; then
                echo "No summary produced for $detector + $descriptor" >&2
                continue
            fi
            echo "${summary#SUMMARY,}" >> "$summary_csv"
            printf '%s\n' "$run_output" | awk -F, '$1 == "METRIC" { sub(/^METRIC,/, ""); print }' >> "$frame_csv"
        else
            echo "Skipping incompatible combination $detector + $descriptor" >&2
        fi
    done
done

echo "Wrote $summary_csv" >&2
echo "Wrote $frame_csv" >&2
