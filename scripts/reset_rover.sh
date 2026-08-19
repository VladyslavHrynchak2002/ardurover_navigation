#!/usr/bin/env bash
set -euo pipefail

# Moves only the Gazebo model. ArduRover's EKF may stay where it was.
# Restart sim.launch.py if GUIDED control looks wrong after a reset.

gz service -s /world/baylands/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: 'husky', position: {x: 0, y: 0, z: 0.25}, orientation: {x: 0, y: 0, z: 0, w: 1}"
