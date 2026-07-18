#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${repo_root}/project/pid_controller"
build_dir="${TMPDIR:-/tmp}/udacity-control-response"
python_bin="${PYTHON_BIN:-python3}"

rm -rf "${build_dir}"
mkdir -p "${build_dir}" "${repo_root}/validation" "${repo_root}/plots"

"${CXX:-c++}" -std=c++14 -Wall -Wextra -Werror -pedantic \
  -I"${source_dir}" \
  "${repo_root}/tests/response_experiment.cpp" \
  "${source_dir}/pid_controller.cpp" \
  "${source_dir}/control_errors.cpp" \
  -o "${build_dir}/response_experiment"

"${build_dir}/response_experiment" "${repo_root}/validation"
"${python_bin}" "${repo_root}/tools/render_response_plots.py" \
  "${repo_root}/validation" "${repo_root}/plots"
rm -rf "${build_dir}"
