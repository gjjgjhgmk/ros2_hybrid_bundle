# Plane Hybrid Planner: Phase 1

This module adds a standalone 2D tabletop planning path without changing the
existing 6D hybrid planner or the laboratory `ur_move` server.

The phase-1 pipeline is:

```text
2D nominal path
  -> circle collision scan
  -> MATLAB-compatible local intent-biased Informed RRT*
  -> MATLAB-compatible TA-HDI-FMP modulation
  -> strict 2D path post-check
  -> arc-length resampling
  -> UV-to-Cartesian waypoints
  -> ur_move ZMQ
  -> MoveIt IK/planning
  -> plan-only or mock-controller execution
```

The original ROS 2 joint-space hybrid planner remains untouched and can still
be run independently.

## Located Workspace

The laboratory workspace was discovered at:

```text
workspaces/multibot_ws
```

The integration was checked against these files:

```text
workspaces/multibot_ws/ur_move/README.md
workspaces/multibot_ws/ur_move/src/server_cpp/trajectory_planner_server.cpp
workspaces/multibot_ws/ur_move/src/server_cpp/waypoint_message.cpp
workspaces/multibot_ws/ur_move/src/server_cpp/moveit_planner.cpp
workspaces/multibot_ws/ur_move/launch/ur_move_server.launch.py
workspaces/multibot_ws/dual_arm/dual_arm_moveit_config/config/kinematics.yaml
workspaces/multibot_ws/dual_arm/dual_arm_moveit_config/config/moveit_controllers.yaml
workspaces/multibot_ws/dual_arm/dual_arm_moveit_config/config/ros2_controllers.yaml
workspaces/multibot_ws/dual_arm/dual_arm_moveit_config/config/joint_limits.yaml
```

## Install Python Dependencies

From the FMP repository root:

```bash
python3 -m pip install -r plane_hybrid_planner/requirements.txt
```

`pyzmq` is only required for the live `ur_move` connection. Without it, the 2D
planner, evaluator, and plots still run and report
`failure_reason=ur_move_unavailable`.

## Coordinate Contract

The 2D planner and MATLAB-compatible RRT/FMP always work in normalized
coordinates `u,v in [0,1]`. No ROS frame is carried inside the 2D algorithms.
Metric data enters or leaves only at the plane boundary:

```text
x = x_min + u * (x_max - x_min)
y = y_min + v * (y_max - y_min)
u = (x - x_min) / (x_max - x_min)
v = (y - y_min) / (y_max - y_min)
```

For the current left-arm plane, `x=[0.50,0.68]` and `y=[0.06,0.24]`, so
`[0.5504, 0.1806] m` maps to `[0.28, 0.67] UV` and maps back exactly within
floating-point tolerance.

Metric obstacles must declare their input contract:

```yaml
obstacle_input:
  coordinate_mode: metric
  frame_id: left_interface_link
  radius_scale_mode: min
```

They are converted once by `obstacle_coordinates.normalize_obstacles_to_uv()`.
Legacy scenarios without `obstacle_input` remain normalized UV. The deprecated
`plane.coordinate_mode` field is ignored for algorithm semantics; use
`plane.algorithm_coordinate_mode: normalized`.

Waypoints, MoveIt collision objects, RViz obstacle markers, and debug metric
XYZ values are expressed in `plane.frame_id`. Changing only a `frame_id` label
is rejected unless code performs a real TF2 transform.

The workstation URDF places `left_interface_link` at z=`0.9102` above
`workstation_base`. The workstation collision box top is z=`0.9`, therefore the
table surface in `left_interface_link` is z=`-0.0102`. Scene obstacle cylinders
default to `center_z = table_surface_z + cylinder_height / 2`.

Before sending, the final path is resampled by arc length to 30 points by
default. This avoids forwarding the full 150/300 point MATLAB/FMP discretization to
MoveIt.

## Start the Dual-Arm Platform

Terminal 1:

```bash
cd /home/woody/simple_fmp_v1/workspaces/multibot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch ur_move ur_move_server.launch.py \
  use_mock_hardware:=true \
  use_fake_gripper_hardware:=true
```

Verify the controllers and actions:

```bash
ros2 control list_controllers
ros2 action list -t | grep follow_joint_trajectory
```

Expected active arm controllers are `left_arm_controller` and
`right_arm_controller`. Expected actions include:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

The server's trajectory-planner ZMQ endpoint is `tcp://127.0.0.1:5605`.

If CMake reports that `nlohmann_jsonConfig.cmake` is missing, install the
workspace dependency before rebuilding (on Ubuntu 22.04 this is normally
provided by `nlohmann-json3-dev`). The current development machine reached
this dependency error during the phase-1 build attempt, so live MoveIt/ZMQ
execution still requires the Workspace environment to be completed.

## Run a Clear-Path Scenario

Terminal 2, from the FMP repository root:

```bash
python3 -m plane_hybrid_planner.run_plane_plan \
  --config plane_hybrid_planner/configs/table_plane_left.yaml \
  --scenario plane_hybrid_planner/configs/scenarios_minimal.yaml \
  --scenario-name no_obstacle_line \
  --group left_arm \
  --plan-only \
  --out-dir outputs/plane_no_obstacle_left
```

## Run the Verified Obstacle Demo

```bash
python3 -m plane_hybrid_planner.run_plane_plan \
  --config plane_hybrid_planner/configs/table_plane_left.yaml \
  --scenario plane_hybrid_planner/configs/scenarios_minimal.yaml \
  --scenario-name matlab_sine_verified_obstacles \
  --group left_arm \
  --plan-only \
  --out-dir outputs/plane_matlab_sine_left_planonly
```

Use the right arm by selecting `table_plane_right.yaml` and
`--group right_arm`.

Metric measured obstacle check:

```bash
python3 -m plane_hybrid_planner.run_plane_plan \
  --config plane_hybrid_planner/configs/table_plane_left.yaml \
  --scenario plane_hybrid_planner/configs/scenarios_minimal.yaml \
  --scenario-name metric_measured_obstacles \
  --group left_arm \
  --plan-only \
  --out-dir outputs/metric_obstacles_coordinate_fix
```

To request mock-controller execution instead of plan-only:

```bash
python3 -m plane_hybrid_planner.run_plane_plan \
  --config plane_hybrid_planner/configs/table_plane_left.yaml \
  --scenario plane_hybrid_planner/configs/scenarios_minimal.yaml \
  --scenario-name matlab_sine_verified_obstacles \
  --group left_arm \
  --execute \
  --out-dir outputs/plane_matlab_sine_left_execute
```

## Outputs

Each run writes:

```text
result.json
result.csv
coordinate_debug.json
cart_waypoints.json
scene_obstacles.json
tip_waypoints.json
ee_waypoints.json
tool_transform.json
drawing_plane.json
ur_move_response.json
uv_path_compare.png
clearance_plot.png
cart_xy_path.png
```

`result.json` contains 2D collision counts, minimum clearance, path length,
jerk proxy, RRT diagnostics, waypoint count, MoveIt status, execution ID, and a
failure reason. The 2D result remains interpretable if `ur_move` is offline;
only the MoveIt planning status is unavailable.

## Planner Comparison Mode

To switch away from the demo path and compare the current MATLAB-compatible
hybrid pipeline against OMPL planners, use the dedicated benchmark entry point:

```bash
python3 -m plane_hybrid_planner.run_algorithm_comparison \
  --config plane_hybrid_planner/configs/table_plane_left.yaml \
  --scenario plane_hybrid_planner/configs/scenarios_minimal.yaml \
  --scenario-name matlab_sine_verified_obstacles \
  --out-dir outputs/compare_matlab_vs_ompl \
  --planners rrt,rrt_star,informed_rrt_star \
  --ompl-mode raw
```

This writes:

```text
comparison_result.json
comparison_result.csv
comparison_paths.png
comparison_clearance.png
```

Use `--ompl-mode both` after the raw round if you want a second pass that
enables OMPL `PathSimplifier`.

## Dispatch Summary

For every `run_plane_plan` invocation, the CLI now prints a compact dispatch
summary so you can immediately tell whether the upstream pipeline actually
released a request to `ur_move`:

```text
[plane_hybrid] nominal_count=58
[plane_hybrid] modulated_count=61
[plane_hybrid] algorithm_failure=None
[plane_hybrid] safety_pass=True
[plane_hybrid] simplified_count=18
[plane_hybrid] resampled_count=30
[plane_hybrid] cart_waypoint_count=30
[plane_hybrid] dispatch=True
[plane_hybrid] group=left_arm
[plane_hybrid] planner=lin
[plane_hybrid] frame_id=left_interface_link
[plane_hybrid] ik_frame=left_ee_link
```

If the pipeline blocks before dispatch, the summary will explicitly say so:

```text
[plane_hybrid] dispatch=False
[plane_hybrid] blocked_stage=safety_check
[plane_hybrid] blocked_reason=first_invalid_idx=17, clearance=-0.0120
```

This is the quickest way to distinguish:

- upstream algorithm/safety gate blocked the request
- the request was sent but `ur_move` or MoveIt rejected/aborted it
- the path was never assembled into Cartesian waypoints

## Direct Downstream Test

To isolate the `PlaneMapper -> ur_move` side without involving the 2D planner,
run the direct dispatch helper:

```bash
python3 -m plane_hybrid_planner.run_direct_plane_dispatch \
  --config plane_hybrid_planner/configs/table_plane_left.yaml \
  --out-dir outputs/direct_plane_left \
  --path-mode polyline \
  --num-points 30 \
  --plan-only
```

This bypasses MATLAB compatibility, RRT, and FMP entirely. If this direct path
is still jittery, the issue is downstream in `ur_move` or MoveIt trajectory
composition. If it is smooth while obstacle runs are empty, the issue is in the
upstream safety gate or modulation pipeline.

## Confirmed ur_move Protocol

The protocol was derived from the current C++ server and its existing Python
client. A planning request has this shape:

```json
{
  "waypoints": [
    {
      "name": "plane_waypoint_000",
      "group": "left_arm",
      "planner": "lin",
      "type": "cart",
      "ik_frame": "left_ee_link",
      "frame_id": "left_interface_link",
      "position": [0.34, 0.25, 0.20],
      "orientation": [0.0, 1.0, 0.0, 0.0],
      "velocity_scaling_factor": 0.1,
      "acceleration_scaling_factor": 0.1
    }
  ],
  "execute": false
}
```

The client records the unmodified request and response. It parses `success`,
`message`, `trajectories`, and `execution_id`. When available, the existing
`ur_move/client/zmq_ur_move_client.py` implementation is reused; otherwise a
protocol-compatible minimal transport is used.

One current server-side limitation is important: the existing
`moveit_planner.cpp` plans Cartesian waypoints as sequential MoveIt requests
and concatenates the resulting trajectories. Phase 1 does not rewrite this
behavior. Therefore, a successful ZMQ/MoveIt plan confirms integration, but
trajectory continuity should be evaluated before physical execution.

## Drawing Tool

The left drawing tool is a real URDF fixed tool, not an RViz-only marker. Enable
it when launching the robot model:

```bash
ros2 launch ur_move ur_move_server.launch.py \
  use_mock_hardware:=true \
  use_fake_gripper_hardware:=true \
  use_left_drawing_tool:=true
```

The default remains `false`, so existing `left_arm` and `left_ee_link` workflows
continue to work unchanged. When enabled, the URDF adds:

```text
left_ee_link
  -> left_pen_body_link
  -> left_pen_tip_link
```

The body has visual, collision, and non-zero inertial properties. The collision
body stops slightly above the mathematical tip frame by
`collision_tip_clearance`, while drawing uses a default
`contact_clearance=0.002 m` above the table surface.

`table_plane_left_pen.yaml` treats the UV path as the desired
`left_pen_tip_link` path. Because the current `ur_move` server initializes only
`left_arm/right_arm` and the SRDF `left_arm` chain ends at `left_ee_link`, the
default dispatch uses a full SE(3) fallback:

```text
F_T_E_desired = F_T_T_desired * inverse(E_T_T)
```

where `F=left_interface_link`, `E=left_ee_link`, and
`T=left_pen_tip_link`. The configured fallback transform is:

```text
E_T_T translation = [0.0, -0.10606601717798213, -0.10606601717798213]
E_T_T rotation_xyzw = [-0.3826834323650898, 0.0, 0.0, 0.9238795325112867]
```

With the existing drawing orientation `Rx(+45deg)`, this places the pen axis
along the table normal toward the surface. Runs write `tip_waypoints.json`,
`ee_waypoints.json`, `tool_transform.json`, and `drawing_plane.json`.

Plan-only drawing check:

```bash
python3 -m plane_hybrid_planner.run_plane_plan \
  --config plane_hybrid_planner/configs/table_plane_left_pen.yaml \
  --scenario plane_hybrid_planner/configs/scenarios_minimal.yaml \
  --scenario-name metric_measured_obstacles \
  --group left_arm \
  --plan-only \
  --out-dir outputs/left_pen_planonly
```

## MATLAB Compatibility

The default runner follows `raw_speed_intent_refine.m` rather than the earlier
phase-1 approximation:

- Internal planning coordinates are `[0,100] x [0,100]`; UV conversion happens
  only at the module boundary.
- Danger indices are joined while their gap is at most 10 samples, then each
  local segment receives four nominal samples of padding at both ends.
- Local planning uses intent-biased Informed RRT* with rewiring and MATLAB's
  first-solution/post-solution sampling probabilities.
- The selected via path is captured after 100 successful post-first
  expansions, matching `fixed_budget=100`.
- Via points are interpolated every `0.5` internal units, trimmed by `0.05 s`,
  and assigned times by cumulative arc length.
- The existing Python `fmp_core.py` supplies GK clustering and Switch/Add soft
  boundary modulation. The x and y models are trained separately with 20
  centers and the MATLAB `+/-1` demonstration augmentation.
- A 150-point demonstration at `demo_dt=0.1` produces a 300-point modulated
  trajectory at online `dt=0.05` whenever via points exist.
- The ambiguous MATLAB `local_window=0.1` is explicit in Python. Supported
  modes are `time`, `samples`, and `ratio`; the default is `time=0.1 s`.
  For a 300-point FMP trajectory at `dt=0.05`, this resolves to a two-sample
  radius around every detected corner.

The complete parameter set is stored under `matlab_defaults` in
`configs/scenarios_minimal.yaml` and is copied into every result JSON.

The `corner_smoothing_metadata` result object records the configured mode and
value, trajectory sample interval, resolved integer sample radius, angle
threshold, sharp-point and region counts, and maximum/mean path displacement.
Corner smoothing runs before the strict collision post-check. If smoothing
pulls the path back into an obstacle, no Cartesian waypoint is sent to
`ur_move`.

## Planner and Safety Behavior

- Nominal paths: line, sine, or polyline.
- Obstacles: circles in normalized UV space.
- RRT*: every candidate state and edge is collision checked using MATLAB's
  `rrt_inflation=0.5` and `edge_sample_step=0.5` internal-unit parameters.
- FMP: the runner preserves the MATLAB-generated geometry. It does not silently
  replace a failed FMP result with the RRT path.
- Post-check: collision or insufficient clearance prevents ZMQ/MoveIt dispatch,
  while the original MATLAB-compatible output remains available for diagnosis.
- The earlier bidirectional RRT-Connect and simple displacement smoother remain
  in separate modules only for controlled comparison; they are not the default.

## Tests

```bash
python3 -m pytest plane_hybrid_planner/tests -q
```

The tests cover plane mapping, path resampling, legacy RRT collision behavior,
MATLAB parameter preservation, danger segmentation, 100-expansion refinement,
300-point FMP output, and evaluator metrics.

## Phase-1 Boundary

Implemented now:

- MATLAB-compatible 2D planning and evaluation.
- Left/right single-arm Cartesian waypoint generation.
- Exact current ZMQ request format and graceful offline handling.
- Plan-only and execute request modes.

Deferred:

- Coupled dual-arm planning.
- Physical Gazebo dynamics, cameras, and grippers.
- pRRTC, GPU planning, and parameter sweeps.
- Reworking the current ur_move trajectory concatenation strategy.
