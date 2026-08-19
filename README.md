# ArduRover path-following assignment

A skid-steer rover in Gazebo + ArduRover SITL. Drive it from QGroundControl, record the path from Gazebo ground truth, then implement a controller that follows that path.

Localization is given (`/ground_truth/odom` from Gazebo). The task is the tracking law, not state estimation.

## Host setup

Needs Docker, X11, and [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html). A GPU is optional.

```bash
cd /path/to/ardurover_navigation
./docker/build.sh          # first time only, ~30–60 min
./docker/run.sh            # opens a shell in the container
```

`./docker/run.sh` again attaches if the container already exists.

Attaching VS Code or Cursor to the container is optional. See [dev-env-setup.md](dev-env-setup.md).

## Inside the container

```bash
./scripts/build.sh
source install/setup.bash

ros2 launch ardurover_nav sim.launch.py
```

This starts Gazebo, ArduRover SITL, the Gazebo→ROS pose bridge, and MAVROS.

The Clearpath Husky A200 skid-steer rover always spawns on the Baylands map at **(0, 0, 0.25)** facing +X (east). The first Gazebo start may download the map from Fuel.

### Record a path

1. In QGroundControl on the host: add a UDP connection to `127.0.0.1:14550` (or it may auto-connect).
2. Arm in **Manual** / **Acro** and drive with a joystick or QGC virtual joystick.
3. In a second container shell (`./docker/attach.sh`):

```bash
source install/setup.bash
ros2 run ardurover_nav path_recorder_node --ros-args \
  -p output_file:=/home/developer/ardurover_navigation/paths/recorded.path
```

The node samples `x y yaw` every 200 ms from `/ground_truth/odom`. Ctrl+C writes the file.

Restart the sim (or the whole launch) so the rover is back at the spawn pose before following.

### Follow a path

Edit `ComputeCommand()` in `src/ardurover_nav/src/trajectory_controller_node.cpp`, then `./scripts/build.sh`.

```bash
source install/setup.bash
ros2 launch ardurover_nav follow.launch.py \
  path_file:=/home/developer/ardurover_navigation/paths/recorded.path
```

`paths/example.path` is a rectangle from spawn if you want to try without recording.

The node arms ArduRover, switches to GUIDED, and publishes body-frame velocity setpoints. You only implement the command.

Allowed output: `geometry_msgs/Twist` on `/mavros/setpoint_velocity/cmd_vel_unstamped`

- `linear.x` — forward speed (m/s), clamped to `v_max` (default 1.2)
- `angular.z` — yaw rate (rad/s), clamped to `w_max` (default 1.0)

Do not upload missions or use position setpoints.

### Score

`path_scorer_node` (started by `follow.launch.py`) samples the rover pose and writes `paths/score.txt` when the last waypoint is reached (within 1 m for 1 s) or after 180 s:

```
score = 100 * completion * exp(-rms_cte / 0.75) * exp(-max_cte / 4.0)
```

- **completion** — fraction of path length reached (closest-point progress)
- **rms_cte / max_cte** — RMS and max distance to the reference polyline (m)

Higher is better. A perfect run on the line to the end is 100.

## Layout

| Path | Role |
|---|---|
| `src/ardurover_nav/src/trajectory_controller_node.cpp` | Your controller |
| `src/ardurover_nav/src/path_recorder_node.cpp` | Records Gazebo pose |
| `src/ardurover_nav/src/path_scorer_node.cpp` | Score |
| `sim/worlds/baylands.sdf` | Map |
| `sim/models/clearpath_husky/` | Rover |
| `paths/` | Path files (`x y yaw`) |

## Notes

- `./scripts/reset_rover.sh` teleports the Gazebo model only. Prefer restarting `sim.launch.py` so ArduRover and Gazebo stay aligned.
- Headless Gazebo: `ros2 launch ardurover_nav sim.launch.py gz_gui:=false`
