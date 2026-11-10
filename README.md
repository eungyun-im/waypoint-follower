# waypoint-follower

A ROS 2 path tracking controller for a small autonomous vehicle. It follows a recorded waypoint path and outputs steering and speed commands.

Steering blends pure pursuit with an extended Stanley law, and speed comes from a curvature-based profile with additional limits for tracking error and oscillation. The controller was developed on an ERP42 Racing platform. The repository is self-contained: every message type it uses is defined in the bundled `waypoint_follower_msgs` package, so it builds with a standard ROS 2 installation and nothing else.

## Packages

```
waypoint_follower/        Controller node, parameters, launch file and test scripts
waypoint_follower_msgs/   Message definitions used by the controller
```

## Build

Requires ROS 2 (tested with Humble) and colcon.

```bash
mkdir -p ~/wf_ws/src && cd ~/wf_ws/src
git clone https://github.com/eungyun-im/waypoint-follower.git
cd ~/wf_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch waypoint_follower waypoint_follower.launch.py
```

The launch file loads `config/tracking_params.yaml`. To use another file, pass `params_file:=/path/to/params.yaml`.

The node runs a 20 Hz control loop. It starts commanding once it has received both a pose on `/odom` and a path on `/final_waypoint`, and commands a stop if either goes quiet for longer than `odom_timeout_sec` / `path_timeout_sec`.

## Controller

**Steering**

- Pure pursuit on the rear axle with a speed-scheduled lookahead, `Ld = clamp(gain * v + min, min, max)`.
- The lookahead is capped near tight bends by `curv_coef * sqrt(R)`, and optionally per mission state. The cap is rate limited so it does not step the steering.
- Extended Stanley on the front axle: heading error, cross-track error, yaw-rate damping, and steering-lag compensation from the measured wheel angle.
- Stanley is faded in between `stanley.fade_lo_mps` and `stanley.fade_hi_mps`, up to `stanley.max_ratio` of the command.
- The pose is projected forward by `delay_comp_sec` to compensate for localisation latency.
- The final command is slew-rate limited (`steer.rate_limit_deg_s`).

**Speed**

- The mission speed is set per mission state, or by a single `mission.uniform_kmh`.
- The curvature profile limits it to `sqrt(a_lat_max / |k|)`, with a backward pass so the car brakes before a bend, and a hold after the bend exit.
- Speed is reduced further as cross-track error grows, and when repeated steering reversals indicate oscillation.
- Optional adaptive cruise input on `lidar_topic` scales speed by the gap to a vehicle ahead.
- Acceleration is ramped. Speed is capped at 7 km/h while the INS status is not good.

## Topics

Subscribed:

| Topic | Type | Notes |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Vehicle pose. Best-effort QoS, depth 1 |
| `/final_waypoint` | `waypoint_follower_msgs/Lane` | Local path to follow |
| `/gps_state` | `waypoint_follower_msgs/State` | Current mission state and lane number |
| `/pva/ins_status` | `std_msgs/UInt32` | INS status. Anything other than `3` caps speed at 7 km/h |
| `/erp42_racing/feedback` | `waypoint_follower_msgs/Feedback` | Measured speed and steering angle. Optional; falls back to odometry speed |
| `lidar_topic` | `waypoint_follower_msgs/LidarTopicMsg` | Adaptive cruise input. Optional |
| `/change_dist` | `std_msgs/Float64` | Optional |

Published:

| Topic | Type | Notes |
|---|---|---|
| `/erp42_racing/control_command` | `waypoint_follower_msgs/ControlCommand` | Speed (m/s), steering (rad), brake |
| `/ego_speed` | `std_msgs/Float32` | Measured speed in km/h, from feedback |
| `/wf_debug/lookahead` | `std_msgs/Float64` | Lookahead in use (m) |
| `/wf_debug/cte` | `std_msgs/Float64` | Signed cross-track error (m) |
| `/wf_debug/terms` | `std_msgs/Float64MultiArray` | Every steering and speed term for one cycle |
| `front_pose`, `back_pose`, `target_pose` | `visualization_msgs/Marker` | RViz markers |
| `course_path` | `std_msgs/Float64` | Vehicle heading (deg) |

## Messages

The definitions in `waypoint_follower_msgs` match the original packages field for field. Only the package name differs.

| Message | Origin |
|---|---|
| `Waypoint`, `Lane`, `State` | Team-internal `waypoint_maker` package |
| `LidarTopicMsg` | Team-internal `new_lane` package |
| `Feedback`, `ControlCommand` | `erp42_racing_msgs` from [mingminQ/erp42_racing_ros](https://github.com/mingminQ/erp42_racing_ros) (Apache-2.0) |

To connect the controller to a stack that uses the original packages, either change the includes back or bridge the topics.

## Parameters

All parameters are in [`waypoint_follower/config/tracking_params.yaml`](waypoint_follower/config/tracking_params.yaml), with the reasoning behind each value in the comments. The main groups:

| Group | Purpose |
|---|---|
| `stanley.*` | Stanley gains and the speed range over which Stanley is blended in |
| `lookahead.*` | Speed schedule, per-state cap, curvature cap and rate limits for the lookahead |
| `speed.*` | Lateral acceleration budget, braking and acceleration rates, profile horizon |
| `mission.*` | Mission speed per state or a single uniform speed |
| `cte.*`, `osc.*` | Speed reduction for tracking error and oscillation |
| `reference.*` | Offset between `/odom` and the rear axle |
| `delay_comp_sec`, `steer.rate_limit_deg_s` | Latency compensation and steering slew limit |

The vehicle constants (wheelbase 1.353 m, 25 km/h speed limit, ±20° steering) are set in the headers under `waypoint_follower/include/waypoint_follower/`.

## Test scripts

The scripts in `waypoint_follower/test/` need a sourced workspace.

- `wf_scenarios.py` drives the running node with synthetic odometry, paths and feedback, and checks the commands it publishes.
- `record_fit_bag.sh` records the topics needed for gain fitting.
- `fit_from_bag.py` and `fit_osc_from_bag.py` fit delays, gains and oscillation from a recorded bag.

```bash
ros2 launch waypoint_follower waypoint_follower.launch.py
python3 waypoint_follower/test/wf_scenarios.py
```

Tuning notes (in Korean) are in [`waypoint_follower/TUNING_NOTES.md`](waypoint_follower/TUNING_NOTES.md).
