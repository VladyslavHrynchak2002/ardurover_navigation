# Controller report — Pure Pursuit path tracking

## Principle

The controller implemented in [`ArduroverController::Control()`](src/ardurover_nav/src/ardurover_controller.cpp)
is a **Pure Pursuit** tracking law. Pure Pursuit steers the rover along a circular arc that
starts at the rover's current position and passes through a **lookahead point** — a point on
the reference path a fixed distance `L` ahead of the rover.

## Algorithm steps (per control tick, 20 Hz)

1. **Localize on the path** — `ClosestWaypointIndex()` finds the path index closest to the
   rover's current `(x, y)`, searching forward from the previously found index (bounded
   40-point window) so progress stays monotonic even if the path crosses itself.

2. **Find the lookahead point** — `LookaheadPoint()` walks forward along the path polyline
   from that index, accumulating arc length, and linearly interpolates the point where the
   accumulated distance reaches the lookahead distance `L` (parameter `lookahead_m`, default
   1.5 m).

3. **Compute the heading error `α`** — the angle between the rover's current heading (`yaw`,
   from odometry) and the direction to the lookahead point `(x_t, y_t)`:

   ```
   α = atan2(y_t − y, x_t − x) − yaw        (wrapped to [−π, π])
   ```

4. **Compute the required curvature** — the classic Pure Pursuit geometry relates the
   curvature `κ` of the arc through the lookahead point to the heading error and lookahead
   distance:

   ```
   κ = 2 · sin(α) / L_d
   ```

   where `L_d` is the actual distance to the lookahead point (≈ `lookahead_m`, clamped away
   from zero).

5. **Compute forward speed** — the nominal cruise speed (`cruise_speed_mps`, default 1.0 m/s)
   is reduced on sharp turns so the rover doesn't overshoot the path:

   ```
   v = clamp( cruise_speed / (1 + 2·|κ|),  0.3,  cruise_speed )
   ```

6. **Compute angular speed** — for a differential-drive / skid-steer rover, the angular rate
   needed to follow curvature `κ` at speed `v` is:

   ```
   ω = κ · v        (clamped to ± max_angular_speed_rps, default 1.5 rad/s)
   ```

7. **Stop condition** — once the closest path index reaches the last waypoint and the rover
   is within `goal_radius_m` (default 0.5 m) of the final point, the controller publishes a
   zero instead of a pursuit command.

## Commands sent

The controller publishes `geometry_msgs/TwistStamped` on `/mavros/setpoint_velocity/cmd_vel`:

- `twist.linear.x = v` — forward speed in the rover's body frame
- `twist.angular.z = ω` — turn rate in the rover's body frame


## Tuning parameters

| Parameter | Default | Effect |
|---|---|---|
| `lookahead_m` | 1.5 m | Larger → smoother but cuts corners more; smaller → tighter tracking but more prone to oscillation |
| `cruise_speed_mps` | 1.0 m/s | Nominal forward speed on straight sections (kept below ArduRover's `CRUISE_SPEED`/`WP_SPEED` of 1.5 m/s to leave margin for turn slowdown) |
| `max_angular_speed_rps` | 1.5 rad/s | Caps how fast the rover is commanded to turn |
| `goal_radius_m` | 0.5 m | Distance to the final waypoint that counts as "arrived" |

All four are ROS 2 parameters and can be overridden at launch, e.g.:

```bash
ros2 launch ardurover_nav control.launch.py path_file:=paths/2-complicated.path lookahead_m:=2.0
```
