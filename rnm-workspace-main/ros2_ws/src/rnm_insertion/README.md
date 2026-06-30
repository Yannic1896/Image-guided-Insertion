# RNM Insertion

This package contains the `insertion_node`, which converts the mapped needle
target and selected entry point into a straight, fixed-orientation needle
insertion trajectory.

The node is for the needle-critical motion only. A general trajectory planner can
still be used for free-space movement near the patient/model, but the insertion
segment is generated here so the needle tip stays on the planned Cartesian line.

## Workflow

1. Read the target point from `target_output/target_location.txt`.
   The default key is `scan_center_m`.
2. Read entry candidate 0 from `entry_point_output/needle_entry_point.txt`.
3. Compute the insertion axis:

   ```text
   axis = normalize(target - entry)
   ```

4. Compute a pre-entry point behind the entry point:

   ```text
   pre_entry = entry - axis * approach_distance_m
   ```

5. Build a fixed end-effector orientation where the end-effector local `+Z`
   axis points along the insertion axis.
6. Generate Cartesian needle-tip samples for:

   ```text
   pre-entry -> entry -> target
   ```

7. Convert each needle-tip pose into an end-effector pose using the configured
   needle tip offset.
8. Solve IK for every sample using `rnm_kinematics.IKSolver`.
9. Prepend a joint-space transition from the current `/joint_states` position
   to the first pre-entry IK sample.
10. Validate the full joint trajectory against `max_joint_speed_rad_s`.
11. Publish the complete joint trajectory once as a flattened
   `std_msgs/msg/Float64MultiArray`.

## Important Inputs

File inputs:

```text
target_output/target_location.txt
entry_point_output/needle_entry_point.txt
```

ROS input:

```text
/joint_states
```

Type:

```text
sensor_msgs/msg/JointState
```

The node waits for current joint states before solving IK.

Launch-loaded kinematics:

```text
rnm_kinematics/config/panda_dh.yaml
rnm_kinematics/config/panda_joint_limits.yaml
```

## Important Outputs

Main robot/controller output:

```text
/joint_position_example_controller/joint_trajectory_command
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

The array is flattened as:

```text
[q0_joint1, ... q0_joint7, q1_joint1, ... qN_joint7]
```

The layout dimensions are:

```text
dim[0] = points
dim[1] = joints
```

Debug/inspection outputs:

```text
/insertion/pre_entry_pose
/insertion/entry_pose
/insertion/target_pose
/insertion/planned_tip_path
```

TF output:

```text
panda_fk_flange -> needle_tip
```

By default the needle tip is 16 cm along end-effector `+Z`.

## Key Parameters

Edit `config/insertion_params.yaml`.

```yaml
needle_tip_offset_m: 0.16
needle_axis_in_ee_frame: [0.0, 0.0, 1.0]
approach_distance_m: 0.03
trajectory_rate_hz: 1000.0
approach_speed_m_s: 0.06
insertion_speed_m_s: 0.01
joint_transition_speed_rad_s: 0.5
max_joint_speed_rad_s: 1.0
cartesian_step_m: 0.002
```

Faster speeds produce fewer trajectory points:

```text
duration = distance / speed
points ~= duration * trajectory_rate_hz
```

`cartesian_step_m` is a safety cap on Cartesian spacing. The node uses whichever
requires more samples: speed/rate timing or maximum Cartesian step size.

`joint_transition_speed_rad_s` controls the non-Cartesian joint-space segment
from the robot's current joint state to the pre-entry IK solution. This avoids
commanding the robot to jump directly to pre-entry in one 1 ms controller tick.

`max_joint_speed_rad_s` is a hard validation limit for the final published
trajectory. If any adjacent pair of joint samples exceeds this speed, the node
does not publish the trajectory.

## Build

From the ROS workspace:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rnm_kinematics rnm_insertion --symlink-install
source install/setup.bash
```

## Launch

Normal launch:

```bash
ros2 launch rnm_insertion insertion_path.launch.py
```

The launch file loads:

```text
rnm_insertion/config/insertion_params.yaml
rnm_kinematics/config/panda_dh.yaml
rnm_kinematics/config/panda_joint_limits.yaml
```

Inspect topics:

```bash
ros2 topic list
ros2 topic echo /insertion/pre_entry_pose
ros2 topic echo /insertion/entry_pose
ros2 topic echo /insertion/target_pose
```

Check the trajectory message shape:

```bash
ros2 topic echo --once /joint_position_example_controller/joint_trajectory_command
```

View the needle-tip TF:

```bash
ros2 run tf2_ros tf2_echo panda_fk_flange needle_tip
```

## Notes

- The node publishes the full joint trajectory once after IK validation.
- It does not publish one joint command at a time.
- It needs `/joint_states` before it can solve IK.
- If IK fails, no trajectory is published.
- If the generated joint trajectory exceeds `max_joint_speed_rad_s`, no
  trajectory is published.
- The `planned_tip_path` topic is decimated for visualization using
  `max_path_output_points`; the actual joint trajectory is not decimated.
