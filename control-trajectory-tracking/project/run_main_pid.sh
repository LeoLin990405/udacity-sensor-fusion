#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
controller="${script_dir}/pid_controller/pid_controller"

if [[ ! -x "${controller}" ]]; then
  echo "Missing ${controller}. Build project/pid_controller first." >&2
  exit 1
fi

"${controller}" &
controller_pid=$!
trap 'kill "${controller_pid}" 2>/dev/null || true' EXIT INT TERM

sleep 1
cd "${script_dir}"
python3 simulatorAPI.py
