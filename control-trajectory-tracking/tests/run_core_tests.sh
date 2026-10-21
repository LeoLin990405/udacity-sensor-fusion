#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${repo_root}/project/pid_controller"
build_dir="${TMPDIR:-/tmp}/udacity-control-core-tests"

rm -rf "${build_dir}"
mkdir -p "${build_dir}"

"${CXX:-c++}" \
  -std=c++14 \
  -Wall \
  -Wextra \
  -Werror \
  -pedantic \
  -I"${source_dir}" \
  "${repo_root}/tests/core_control_test.cpp" \
  "${source_dir}/pid_controller.cpp" \
  "${source_dir}/control_errors.cpp" \
  -o "${build_dir}/core_control_test"

"${build_dir}/core_control_test"
rm -rf "${build_dir}"
