#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-/opt/ardupilot}"
SITL_DIR="${ROOT}/.sitl"

mkdir -p "${SITL_DIR}"
cd "${SITL_DIR}"

export PATH="${HOME}/.local/bin:${PATH}"

# Match sim/worlds/baylands.sdf spherical_coordinates. Yaw 90 deg = rover +X (east) at spawn.
exec "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" \
  -v Rover \
  --no-rebuild \
  --model JSON \
  --custom-location 37.412173071650805,-121.998878727967,38,90 \
  --out 127.0.0.1:14551 \
  --out 127.0.0.1:14550 \
  --add-param-file "${ARDUPILOT_DIR}/Tools/autotest/default_params/rover-skid.parm" \
  --add-param-file "${ROOT}/sim/params/ardurover.parm" \
  "$@"
