# RNM Mapping Pipeline

This package turns robot-mounted Azure Kinect point clouds into a target
location in the robot base frame.

The pipeline has four stages:

1. Stitch settled camera point clouds into one scan.
2. Register the high-resolution STL model to that scan.
3. Locate the target ball in the registered model.
4. Find a needle entry point and optionally publish entry/target as one PoseArray.

## Frames And TF

The whole pipeline depends on a valid TF path from the robot base to the point
cloud frame at each cloud timestamp:

```text
panda_link0 -> ... -> panda_EE -> rgb_camera_link -> depth_camera_link
```

The stitcher does not compute robot kinematics. It asks TF for:

```text
panda_link0 <- cloud.header.frame_id at cloud.header.stamp
```

If this TF chain is wrong, duplicated, or missing, the stitched scan will be
wrong. On bag replay, avoid publishing two transforms for the same child frame.

Useful checks:

```bash
ros2 topic echo --once /k4a/points2 --field header
ros2 topic echo --once /k4a/points2 --field fields
ros2 run tf2_ros tf2_echo panda_link0 depth_camera_link
ros2 run tf2_tools view_frames
```

## Cloud Stitching

Node:

```bash
ros2 launch rnm_mapping cloud_stitcher.launch.py use_sim_time:=false
```

Code:

```text
rnm_mapping/cloud_stitcher.py
```

Config:

```text
config/cloud_stitcher.yaml
```

The stitcher subscribes to a `sensor_msgs/msg/PointCloud2`, transforms accepted
points into `target_frame` (normally `panda_link0`), accumulates them, publishes
the result, and saves it as PLY.

### Settled-Pose Capture

The robot should not be moving while a cloud is accepted. The node tracks the
camera pose from TF and waits until it has stayed within translation and
rotation limits for `settled_time_sec`.

Important parameters:

```yaml
settled_time_sec: 1.0
settled_translation_m: 0.001
settled_rotation_rad: 0.005
capture_once_per_settled_pose: false
max_clouds_per_settled_pose: 2
```

`capture_once_per_settled_pose` accepts only one cloud per pose when true.
`max_clouds_per_settled_pose` bounds repeated captures at one settled pose.

### Source-Frame Filtering

Before transforming points into the base frame, the node filters in the camera
source frame:

```yaml
min_depth_m: 0.25
max_depth_m: 0.9
max_range_m: 0.0
```

Depth is the source-frame `z` coordinate. This removes points too close to the
camera or too far in the background.

### Base-Frame Cropping

After transforming points into `panda_link0`, the node can crop by workspace:

```yaml
crop_min: []
crop_max: []
```

When known, this is one of the best filters because it removes table, robot, and
background points independent of camera view.

### Voxel Downsampling

The node downsamples points by voxel grid. Each voxel is represented by the
centroid of points that fell inside it. Color is averaged too.

```yaml
input_voxel_size_m: 0.01
accumulated_voxel_size_m: 0.01
```

Input voxel size filters each accepted cloud. Accumulated voxel size controls
the merged scan resolution.

For model registration, fewer clean points are better than many noisy points.

### Outlier Filtering

The node can remove isolated points by radius-neighbor tests:

```yaml
input_outlier_radius_m: 0.02
input_outlier_min_neighbors: 4
output_outlier_radius_m: 0.025
output_outlier_min_neighbors: 4
```

A point survives only if enough neighboring points exist within the radius.

### Observation Support

The accumulated cloud stores a support count per voxel. The final published and
saved scan can require a minimum number of observations:

```yaml
min_accumulated_observations: 2
```

This removes one-off noise but can delete real geometry seen from only one
viewpoint.

### Cloud-Level Overlap Gate

The node can reject a whole cloud if too few points overlap the existing scan:

```yaml
min_cloud_overlap_ratio: 0.0
overlap_distance_m: 0.03
```

This is disabled by default. It can reject bad views, but it can also reject
useful new sides of the object.

### Publishing And Saving

The node publishes:

```text
/stitched_cloud   sensor_msgs/msg/PointCloud2
```

Services:

```bash
ros2 service call /reset_scan std_srvs/srv/Trigger {}
ros2 service call /save_scan std_srvs/srv/Trigger {}
```

The saved scan is written to `output_path`, usually:

```text
/workspaces/RNM/rnm-workspace-main/stitched_cloud.ply
```

## STL Registration

Command:

```bash
ros2 run rnm_mapping register_stl_to_scan \
  --scan /workspaces/RNM/rnm-workspace-main/stitched_cloud.ply \
  --model /workspaces/RNM/rnm-workspace-main/scanning_bag/Skeleton_Target.stl \
  --output-dir /workspaces/RNM/rnm-workspace-main/registration_output
```

Code:

```text
rnm_mapping/model_registration.py
```

The STL is treated as the semantic high-resolution model. The current STL is in
millimeters, so the default model scale is:

```text
0.001
```

The registration tool:

1. Loads the stitched scan PLY.
2. Loads and samples points on the STL surface.
3. Scales the model from millimeters to meters.
4. Downsamples both point sets.
5. Computes a PCA-based initial rigid alignment.
6. Refines with trimmed point-to-point ICP.
7. Writes the model-to-scan transform and overlay PLY.

Outputs:

```text
registration_output/model_to_scan_transform.txt
registration_output/scan_to_model_transform.txt
registration_output/registration_overlay.ply
registration_output/registered_model_points.ply
registration_output/scan_for_registration.ply
```

The important file is:

```text
model_to_scan_transform.txt
```

It maps model-frame points, in meters, into the scan/base frame.

## Target Localization

Command:

```bash
ros2 run rnm_mapping locate_stl_target --publish-duration-sec 30
```

Code:

```text
rnm_mapping/target_locator.py
```

The target locator uses the registered STL, not the noisy scan, to identify the
small ball target.

It:

1. Samples the STL surface.
2. Scales the sample from millimeters to meters.
3. Voxelizes the model.
4. Finds compact spherical connected components within a size range.
5. Selects the target ball candidate.
6. Transforms the target center with `model_to_scan_transform.txt`.
7. Validates the target against the stitched scan.
8. Writes outputs and publishes ROS messages.

Validation checks:

```text
inside scan bounding box
nearest scan point distance
number of scan points within support radius
```

Outputs:

```text
target_output/target_location.txt
target_output/target_overlay.ply
```

Standalone published topics:

```text
/target_point   geometry_msgs/msg/PointStamped
/target_pose    geometry_msgs/msg/PoseStamped
```

Default frame:

```text
panda_link0
```

`/target_pose` uses the target position and identity orientation. A trajectory
planner can subscribe to this as the target pose seed.

Useful check:

```bash
ros2 topic echo --once /target_pose
```

## Needle Entry Planning

Command:

```bash
ros2 run rnm_mapping find_needle_entry --publish-duration-sec 30
```

Config:

```text
config/needle_entry_planner.yaml
```

Code:

```text
rnm_mapping/needle_entry_planner.py
```

The planner loads the registered STL, the model-to-scan transform, and the
target report from `locate_stl_target`. It creates a sampling plane just above
the mesh along the configured approach axis, casts rays from the target toward
that plane, and chooses the best valid plane point as the needle entry. The STL
mesh is treated as an obstacle: candidates are rejected when the target-to-entry
segment intersects the mesh before reaching the plane.

Important defaults are configured in YAML:

```text
plane clearance: 0.005 m
needle length:   0.160 m
needle diameter: 0.005 m
approach axis:   [0, 0, 1]
path elevation:  10 to 50 deg from horizontal
hard clearance:  at least 0.005 m from model surface samples
preferred clearance: 0.010 m, used for scoring when available
```

Path elevation is measured against the horizontal plane through the target:
`0 deg` is horizontal and `90 deg` is vertical. The horizontal plane normal is
configured with `horizontal_axis`.

The path-clearance check samples the entry-to-target segment and measures the
nearest registered-model surface point. `min_path_clearance` is the hard
reject threshold. `preferred_path_clearance` increases candidate score when
more room is available, but does not make the planner return no result.

The entry and target endpoints are excluded with
`path_clearance_entry_exclusion` and `path_clearance_target_exclusion`. Optional
`entry_region_min` / `entry_region_max` bounds can restrict candidates to a
robot- or procedure-friendly entry patch.

Outputs:

```text
entry_point_output/needle_entry_point.txt
entry_point_output/needle_entry_overlay.ply
target_output/target_entry_overlay.ply
```

`needle_entry_point.txt` records the highest-scored candidate in the top-level
fields and then writes the ranked candidate list as `[candidate_0]`,
`[candidate_1]`, etc. `candidate_0` is the selected entry used by the combined
PoseArray publisher. The insertion package can parse the retained candidates
and choose a robot-feasible entry based on joint limits or motion efficiency.

`target_entry_overlay.ply` starts from `target_output/target_overlay.ply` when
available, then adds the selected entry marker, alternate candidate markers,
and the selected entry-to-target needle axis.

Standalone published topics:

```text
/needle_entry_point  geometry_msgs/msg/PointStamped
/needle_entry_pose   geometry_msgs/msg/PoseStamped
```

`/needle_entry_pose` is positioned at the entry point. Its local z-axis points
from entry to target, so the insertion package can use it as the initial needle
axis reference.

## Combined Entry/Target PoseArray

The model-registration pipeline can publish one final PoseArray after target
localization and needle-entry planning finish:

```text
/needle_entry_target_poses  geometry_msgs/msg/PoseArray
```

Pose order:

```text
poses[0] = selected entry, candidate_0 from needle_entry_point.txt
poses[1] = target, scan_center_m from target_location.txt
```

Both poses use the same orientation. The local z-axis points from entry to
target, matching the intended needle insertion axis. Configure this final
publisher in:

```text
config/model_target_entry_pipeline.yaml
```

## Recommended Real-Robot Workflow

1. Start robot bringup and Azure Kinect driver.
2. Publish hand-eye/static camera TF.
3. Verify cloud frame and TF tree.
4. Run the stitcher.
5. Move the robot around the skeleton and capture settled views.
6. Save the scan.
7. Register STL to scan.
8. Run the model-target-entry pipeline.
9. Let the trajectory planner consume `/needle_entry_target_poses`.

Minimal command sequence:

```bash
ros2 launch rnm_tools camera_static_tf.launch.py
ros2 launch rnm_mapping cloud_stitcher.launch.py use_sim_time:=false
ros2 service call /save_scan std_srvs/srv/Trigger {}
ros2 launch rnm_mapping model_target_entry_pipeline.launch.py
```

## Assumptions

- The TF tree is correct and has no duplicate child-frame publishers.
- The point cloud frame matches the coordinates inside the cloud.
- The saved scan and STL are in metric units after applying model scale.
- The STL target ball is a compact connected component.
- Registration quality is good enough before trusting the target.
