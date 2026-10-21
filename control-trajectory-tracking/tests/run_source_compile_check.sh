#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${repo_root}/project/pid_controller"
build_dir="${TMPDIR:-/tmp}/udacity-control-source-check"

rm -rf "${build_dir}"
mkdir -p "${build_dir}"

sources=(
  main.cpp
  behavior_planner_FSM.cpp
  motion_planner.cpp
  cubic_spiral.cpp
  spiral_base.cpp
  integral.cpp
  spiral_equations.cpp
  cost_functions.cpp
  utils.cpp
  velocity_profile_generator.cpp
  pid_controller.cpp
  control_errors.cpp
)

for source in "${sources[@]}"; do
  echo "Compiling ${source}"
  "${CXX:-c++}" \
    -std=c++14 \
    -Wall \
    -Wextra \
    -Wno-unused-parameter \
    -pedantic \
    -I"${repo_root}/tests/stubs" \
    -I"${source_dir}" \
    -isystem "${source_dir}/eigen-3.3.7" \
    -c "${source_dir}/${source}" \
    -o "${build_dir}/${source%.cpp}.o"
done

echo "All project translation units compiled successfully"
rm -rf "${build_dir}"
