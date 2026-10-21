#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$repo_root/project/starter_files"
build_dir="${TMPDIR:-/tmp}/udacity-motion-planning-core-tests"
compiler="${CXX:-c++}"

mkdir -p "$build_dir"

"$compiler" \
  -std=c++14 \
  -Wall \
  -Wextra \
  -Wno-deprecated-copy \
  -Wno-unused-parameter \
  -I"$repo_root/tests/stubs" \
  -I"$source_dir" \
  -I"$source_dir/eigen-3.3.7" \
  "$repo_root/tests/core_logic_test.cpp" \
  "$source_dir/behavior_planner_FSM.cpp" \
  "$source_dir/cost_functions.cpp" \
  "$source_dir/cubic_spiral.cpp" \
  "$source_dir/integral.cpp" \
  "$source_dir/motion_planner.cpp" \
  "$source_dir/spiral_base.cpp" \
  "$source_dir/spiral_equations.cpp" \
  "$source_dir/utils.cpp" \
  "$source_dir/velocity_profile_generator.cpp" \
  -o "$build_dir/core_logic_test"

"$build_dir/core_logic_test"
