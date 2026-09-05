# RNM Calibration and Mapping Report

This report is based on the current workspace state in
`/mnt/c/Users/Heisenberg/Documents/RNM/rnm-workspace-main`.

The main focus is:

- `ros2_ws/src/rnm_calibration`
- `ros2_ws/src/rnm_mapping`

The surrounding packages were also checked to understand how these two packages
fit into the full RNM workflow.

## 1. Workspace Context

This repository is a ROS 2 Humble workspace for a robot-assisted needle
insertion workflow. The central idea is:

1. Calibrate the Azure Kinect camera system.
2. Calibrate or publish the transform between the robot and camera.
3. Move the robot-mounted camera through scan poses.
4. Stitch RGB-D point clouds into one scan in the robot base frame.
5. Register a high-resolution STL model to the scan.
6. Locate a target feature on the registered model.
7. Plan a safe needle entry point.
8. Feed entry and target poses into needle path planning, IK, and trajectory
   execution.

The relevant packages around the requested packages are:

- `rnm_handeye`: computes camera-to-gripper hand-eye calibration from paired
  robot and camera target poses.
- `rnm_scanning`: commands scan poses and coordinates image or point cloud
  capture.
- `rnm_kinematics`: forward and inverse kinematics for the Panda robot.
- `rnm_trajectory`: generates joint trajectories, mainly with quintic
  polynomials and joint-limit validation.
- `rnm_needle` and `rnm_insertion`: consume target and entry outputs to build
  an insertion path.
- `rnm_tools`: simulation, visualization, static camera TF, and support tools.

The `rnm_calibration` package provides camera calibration data. The
`rnm_mapping` package turns calibrated, transformed camera data into geometric
decisions: target location and needle entry.

## 2. `rnm_calibration` Overview

Package path:

```text
ros2_ws/src/rnm_calibration
```

Main entry points from `setup.py`:

- `azure_kinect_calibrate = rnm_calibration.azure_kinect_calibrator:main`
- `azure_kinect_calibration_publisher = rnm_calibration.calibration_publisher:main`
- `k4a_extract_calibration = rnm_calibration.k4a_factory_calibration:main`

Important files:

- `azure_kinect_calibrator.py`: live OpenCV calibration from synchronized RGB
  and depth/IR images.
- `calibration_targets.py`: checkerboard and ChArUco target detection.
- `calibration_math.py`: rigid-transform inverse, rotation averaging, and
  quaternion conversion.
- `calibration_io.py`: YAML schema for calibration results.
- `calibration_publisher.py`: publishes `CameraInfo` and static TF from YAML.
- `k4a_factory_calibration.py`: extracts Azure Kinect factory calibration
  through the Azure Kinect Sensor SDK C API using `ctypes`.
- `config/azure_kinect_calibration.yaml`: node parameters.
- `config/calib.yaml`: current generated calibration file.

## 3. Calibration Data Flow

The live calibration flow is:

```text
RGB Image + depth/IR calibration Image
    -> approximate timestamp synchronization
    -> grayscale conversion
    -> target detection in both images
    -> paired 2D image points and 3D target points
    -> RGB intrinsic calibration
    -> depth/IR intrinsic calibration
    -> stereo extrinsic calibration
    -> calibration YAML
    -> CameraInfo publishers and static TF publisher
```

The package supports two calibration sources:

- A live OpenCV calibration using checkerboard or ChArUco images.
- Factory calibration read directly from the Azure Kinect SDK.

Both sources are written into a schema that the publisher can load.

## 4. Camera Calibration Concepts

### 4.1 Camera Intrinsics

Camera intrinsics describe how 3D points in a camera coordinate frame project
onto the image sensor.

The code uses the standard pinhole camera matrix:

```text
K = [fx  0 cx
      0 fy cy
      0  0  1]
```

Where:

- `fx`, `fy`: focal lengths in pixels.
- `cx`, `cy`: principal point in pixels.
- The third row makes the projection work in homogeneous coordinates.

In `calib.yaml`, these values appear under:

- `rgb_camera.camera_matrix`
- `depth_camera.camera_matrix`

The current RGB image size is `2048 x 1536`. The current depth/IR calibration
image size is `640 x 576`.

### 4.2 Distortion Coefficients

Real lenses do not behave like perfect pinhole cameras. The image is warped by
radial and tangential distortion.

The live OpenCV calibration writes:

```text
distortion_model: plumb_bob
```

This is ROS terminology for the Brown-Conrady-style model commonly represented
as:

```text
[k1, k2, p1, p2, k3]
```

Where:

- `k1`, `k2`, `k3`: radial distortion.
- `p1`, `p2`: tangential distortion.

The factory calibration extractor writes:

```text
distortion_model: rational_polynomial
```

That model includes more coefficients:

```text
[k1, k2, p1, p2, k3, k4, k5, k6]
```

The Azure Kinect factory API itself exposes parameters including `cx`, `cy`,
`fx`, `fy`, `k1` through `k6`, `p1`, `p2`, and additional model-specific fields
such as `codx`, `cody`, and `metric_radius`.

### 4.3 Extrinsics

Extrinsics describe the rigid transform between two coordinate frames. In this
package, the important relationship is between the RGB optical frame and the
depth optical frame.

The calibration YAML stores both directions:

```text
rotation_depth_to_rgb
translation_depth_to_rgb
rotation_rgb_to_depth
translation_rgb_to_depth
```

The transform is a rigid transform:

```text
p_target = R * p_source + t
```

Where:

- `R` is a 3 x 3 rotation matrix.
- `t` is a 3D translation vector.

The inverse of a rigid transform is:

```text
R_inverse = R.T
t_inverse = -R.T * t
```

This is implemented in `calibration_math.invert_rigid_transform`.

### 4.4 CameraInfo

ROS uses `sensor_msgs/msg/CameraInfo` to distribute camera calibration. The
publisher fills:

- `width`, `height`
- `distortion_model`
- `k`: intrinsic matrix
- `d`: distortion coefficients
- `r`: rectification matrix
- `p`: projection matrix

The calibration publisher uses a reliable transient-local QoS profile. That is
appropriate for calibration data because late subscribers should still receive
the latest camera info.

### 4.5 Static TF

The publisher can broadcast the depth camera frame under the RGB camera frame.
By default, it takes frame IDs from the calibration file:

```text
parent_frame_id: rgb_camera_optical_frame
child_frame_id: depth_camera_optical_frame
```

It uses `rotation_depth_to_rgb` and `translation_depth_to_rgb` to place the
depth frame relative to the RGB parent frame. The rotation matrix is converted
to a quaternion before publication.

## 5. Calibration Target Detection

Target detection is in `calibration_targets.py`.

### 5.1 Checkerboard Target

The checkerboard target uses inner corners, not square counts.

The current config uses:

```text
checkerboard_columns: 8
checkerboard_rows: 5
checkerboard_square_size_m: 0.04
```

Algorithm:

1. Build 3D object points on a flat plane with `z = 0`.
2. Detect corners with `cv2.findChessboardCorners`.
3. Refine corner positions with `cv2.cornerSubPix`.
4. Return corresponding 2D image points and 3D object points.

The 3D object points are:

```text
(0, 0, 0), (square_size, 0, 0), ...
```

This gives OpenCV known physical distances between detected image points.

### 5.2 ChArUco Target

ChArUco combines ArUco markers with chessboard corners.

The code supports:

- ArUco dictionary selection, such as `DICT_4X4_50`.
- Marker detection with `aruco.detectMarkers`.
- ChArUco corner interpolation with `aruco.interpolateCornersCharuco`.
- Matching by unique corner IDs.

ChArUco is useful when part of the board is occluded, because marker IDs allow
the code to know which physical point each detected corner represents.

The config currently defaults to checkerboard, but the code supports ChArUco.

### 5.3 Grayscale Conversion

The calibrator accepts RGB, grayscale, IR, or depth-like images. The helper
`image_to_grayscale` handles:

- 3-channel BGR images.
- 4-channel BGRA images.
- Already-grayscale `uint8` images.
- Numeric depth or IR arrays.

For non-`uint8` images, it:

1. Converts to `float32`.
2. Keeps finite positive values.
3. Finds the 2nd and 98th percentiles.
4. Scales that range to `0..255`.
5. Clips and converts to `uint8`.

This makes IR or depth images usable by OpenCV corner detectors.

## 6. Calibration Sample Collection

The calibrator subscribes to:

- `rgb_image_topic`
- `depth_calibration_image_topic`

It uses `message_filters.ApproximateTimeSynchronizer` with:

- `sync_queue_size`
- `sync_slop_s`

This accepts image pairs with timestamps close enough to count as the same
observation.

A sample is accepted only if:

1. The minimum time since the previous sample has elapsed.
2. The target is visible in both images.
3. Enough common target points are visible.
4. The image sizes remain consistent.

The current config uses:

```text
sample_count: 25
min_sample_interval_s: 1.0
min_common_points: 8
sync_slop_s: 0.1
```

## 7. Intrinsic Calibration Algorithm

Intrinsic calibration is performed with:

```python
cv2.calibrateCamera(object_points, image_points, image_size, None, None)
```

For each camera, OpenCV receives:

- A list of 3D target points for each sample.
- A list of matching 2D image points for each sample.
- The image size.

It estimates:

- Camera matrix `K`.
- Distortion coefficients.
- Per-view target poses, which this package does not store.
- RMS reprojection error.

Reprojection error means:

1. OpenCV estimates the camera model.
2. It projects each known 3D target point back into the image.
3. It compares projected points to detected image points.
4. RMS summarizes the average image-space error in pixels.

Lower RMS usually means a better fit, but it must be interpreted with target
quality, image resolution, and number of views.

The current generated calibration file records:

```text
rgb RMS:    0.15298390905162473
depth RMS:  0.530323633321728
stereo RMS: 0.4697509476488676
```

## 8. Stereo Calibration Algorithm

Stereo calibration estimates the rigid transform between RGB and depth/IR
cameras.

The package first calibrates intrinsics separately. Then it calls:

```python
cv2.stereoCalibrate(..., flags=cv2.CALIB_FIX_INTRINSIC)
```

`CALIB_FIX_INTRINSIC` means:

- Keep the RGB intrinsics fixed.
- Keep the depth intrinsics fixed.
- Solve only the relative rotation and translation between cameras.

The stereo solver uses the same physical target observed by both cameras. The
important idea is that the target has one physical coordinate system, but each
camera sees it from a different viewpoint. The relationship between those
viewpoints gives the camera-to-camera transform.

## 9. solvePnP Fallback

If `cv2.stereoCalibrate` fails, the code falls back to a per-sample PnP method.

For each paired sample:

1. Estimate target pose in RGB camera using `cv2.solvePnP`.
2. Estimate target pose in depth camera using `cv2.solvePnP`.
3. Convert OpenCV rotation vectors to matrices with `cv2.Rodrigues`.
4. Derive the relative camera transform from the two target poses.
5. Save one relative transform per sample.

The fallback then averages:

- Translations by arithmetic mean.
- Rotations by averaging matrices and projecting the result back onto SO(3).

SO(3) is the group of valid 3D rotation matrices. A valid rotation matrix must
be orthonormal and have determinant `+1`. Directly averaging rotation matrices
can produce a matrix that is not a valid rotation. The code fixes this with
SVD:

```text
mean_rotation = U * S * V.T
averaged_rotation = U * V.T
```

If the determinant is negative, it flips the last singular vector to avoid
creating a reflection.

## 10. Azure Kinect Factory Calibration Extraction

`k4a_factory_calibration.py` reads factory calibration from the Azure Kinect
Sensor SDK.

Important concepts:

- `ctypes` mirrors C structs such as `k4a_calibration_t`.
- The code dynamically loads `libk4a`.
- It opens a device by index.
- It asks the SDK for calibration for a selected depth mode and color
  resolution.
- It writes YAML or JSON.

Supported depth modes include:

- `NFOV_2X2BINNED`
- `NFOV_UNBINNED`
- `WFOV_2X2BINNED`
- `WFOV_UNBINNED`
- `PASSIVE_IR`

Supported color resolutions include:

- `720P`
- `1080P`
- `1440P`
- `1536P`
- `2160P`
- `3072P`

The factory SDK stores translation in millimeters. The package converts the
published calibration translations to meters. It also stores raw extrinsics in
millimeters under `k4a_raw_extrinsics`.

## 11. `rnm_mapping` Overview

Package path:

```text
ros2_ws/src/rnm_mapping
```

Main entry points from `setup.py`:

- `cloud_stitcher = rnm_mapping.cloud_stitcher:main`
- `register_stl_to_scan = rnm_mapping.model_registration:main`
- `locate_stl_target = rnm_mapping.target_locator:main`
- `find_needle_entry = rnm_mapping.needle_entry_planner:main`
- `publish_target_entry_pose_array = rnm_mapping.target_entry_pose_array_publisher:main`
- `post_handeye_pipeline_supervisor = rnm_mapping.post_handeye_pipeline_supervisor:main`

The mapping package implements this pipeline:

```text
Azure Kinect PointCloud2 at multiple robot poses
    -> transform every accepted cloud into robot base frame
    -> filter and stitch point clouds
    -> save stitched scan as PLY
    -> sample STL model
    -> register STL to scan
    -> find spherical target in registered model
    -> transform target into robot base frame
    -> sample possible needle entry points
    -> reject unsafe or impossible paths
    -> publish entry and target poses
```

## 12. Coordinate Frames and Units

The mapping package depends on a valid TF chain:

```text
panda_link0 -> ... -> panda_EE -> rgb_camera_link -> depth_camera_link
```

The cloud stitcher does not compute kinematics. For each incoming cloud, it
asks TF for:

```text
target_frame <- cloud.header.frame_id at cloud.header.stamp
```

The default target frame is:

```text
panda_link0
```

The STL model is assumed to be in millimeters by default. The model is scaled
by:

```text
model_scale: 0.001
```

After scaling, model coordinates are in meters.

The important registration output is:

```text
registration_output/model_to_scan_transform.txt
```

It maps model-frame points in meters into the scan or robot-base frame:

```text
p_scan = T_model_to_scan * p_model
```

## 13. PointCloud2 and PLY Handling

Point cloud utilities are in `point_cloud_utils.py`.

### 13.1 PointCloud2 Parsing

The code reads `sensor_msgs/msg/PointCloud2` using `sensor_msgs_py.point_cloud2`.
It supports fields:

- `x`
- `y`
- `z`
- optional `rgb`
- optional `rgba`

It filters out non-finite points. If color is present, it unpacks ROS-style
packed RGB values.

### 13.2 Packed RGB

Some ROS point clouds store color as a packed `float32`. The code handles this
by viewing the float bits as `uint32` and extracting:

```text
red   = (packed >> 16) & 0xFF
green = (packed >> 8)  & 0xFF
blue  = packed         & 0xFF
```

When publishing colored clouds, it packs RGB back into the same float32 layout.

### 13.3 PLY Files

The package writes ASCII PLY files for easy inspection:

- `stitched_cloud.ply`
- `registration_overlay.ply`
- `registered_model_points.ply`
- `target_overlay.ply`
- `needle_entry_overlay.ply`
- `target_entry_overlay.ply`

PLY is used as a simple exchange and visualization format for point sets with
optional RGB colors.

## 14. Cloud Stitching

Cloud stitching is implemented in `cloud_stitcher.py`.

The node subscribes to a point cloud topic, transforms accepted points into a
fixed frame, accumulates them, publishes the stitched cloud, and saves a PLY on
request.

Default launch:

```bash
ros2 launch rnm_mapping cloud_stitcher.launch.py
```

Important topics and services:

```text
Subscribes: /k4a/points2
Publishes:  /stitched_cloud
Publishes:  /cloud_stitcher/pose_capture_done
Service:    /reset_scan
Service:    /save_scan
```

### 14.1 Settled-Pose Capture

The robot-mounted camera should not be moving while a cloud is accepted. The
node stores a reference camera pose and checks later poses against it.

Translation difference:

```text
norm(current_translation - reference_translation)
```

Rotation difference:

```text
2 * acos(abs(dot(q_current, q_reference)))
```

This quaternion formula measures the angular separation between orientations.
The absolute dot product handles the fact that `q` and `-q` represent the same
rotation.

The current config uses:

```text
settled_time_sec: 1.0
settled_translation_m: 0.001
settled_rotation_rad: 0.005
capture_once_per_settled_pose: true
max_clouds_per_settled_pose: 1
```

So the pose must remain within 1 mm and about 0.286 degrees for 1 second before
one cloud is accepted.

### 14.2 Time-Jump Reset

If bag playback jumps backward in time, TF and accumulated data can become
inconsistent. The node can reset accumulation when it detects a backward time
jump:

```text
reset_on_time_jump: true
```

### 14.3 Source-Frame Filtering

Before transforming points, the code filters in the cloud source frame:

```text
min_depth_m
max_depth_m
max_range_m
```

Depth is the source-frame `z` coordinate. Range is full Euclidean distance from
the camera origin.

Current config:

```text
min_depth_m: 0.25
max_depth_m: 0.9
max_range_m: 0.0
```

`0.0` disables a limit.

### 14.4 Transforming Points into the Base Frame

The TF lookup produces a rotation and translation from the cloud source frame
into the target frame. Points are transformed as:

```text
points_target = points_source * R.T + t
```

This is equivalent to applying:

```text
p_target = R * p_source + t
```

to every point, just vectorized for row-major point arrays.

### 14.5 Workspace Cropping

After transformation, the cloud can be cropped in the target frame with:

```text
crop_min: [x, y, z]
crop_max: [x, y, z]
```

This is often more reliable than source-frame filtering because it removes
robot, table, or background geometry by physical workspace position.

The current `cloud_stitcher.yaml` leaves cropping disabled.

### 14.6 Voxel Downsampling

Voxel downsampling reduces point count by dividing space into cubic cells.

Algorithm:

1. Compute voxel index:

   ```text
   floor(point / voxel_size)
   ```

2. Group points with the same voxel index.
3. Replace each group by its weighted centroid.
4. Average colors when present.
5. Carry support weights when accumulating.

Current config:

```text
input_voxel_size_m: 0.01
accumulated_voxel_size_m: 0.01
global_downsample_every_n_clouds: 1
```

This keeps the scan manageable and reduces depth noise.

### 14.7 Radius Outlier Filtering

The code removes isolated points with a radius-neighbor test using
`scipy.spatial.cKDTree`.

For each point:

1. Count neighbors within `radius_m`.
2. Keep the point only if the count is high enough.

The implementation compares against `min_neighbors + 1` because the point
itself is included in the KD-tree query result.

Current config:

```text
input_outlier_radius_m: 0.025
input_outlier_min_neighbors: 5
output_outlier_radius_m: 0.035
output_outlier_min_neighbors: 5
```

### 14.8 Observation Support

The accumulated cloud stores a weight for each downsampled voxel. This is used
as a support count. A voxel can be kept only if it has been observed enough
times:

```text
min_accumulated_observations: 2
```

This removes one-off depth noise but can also remove real surfaces seen from
only one viewpoint.

### 14.9 Cloud-Level Overlap Gate

The stitcher can reject a whole cloud if it does not overlap enough with the
existing accumulated scan.

Algorithm:

1. Sample points from the new transformed cloud.
2. Build a KD-tree for the accumulated cloud.
3. Find each sampled point's nearest accumulated point.
4. Compute the ratio whose nearest distance is under `overlap_distance_m`.
5. Reject if the ratio is below `min_cloud_overlap_ratio`.

Current config:

```text
min_cloud_overlap_ratio: 0.15
overlap_distance_m: 0.035
overlap_min_accumulated_points: 1000
overlap_max_sample_points: 8000
```

This protects against bad TF or unrelated clouds, but aggressive overlap
requirements can reject useful new views.

## 15. Model Registration

Model registration is implemented in `model_registration.py`.

Command:

```bash
ros2 run rnm_mapping register_stl_to_scan
```

The goal is to align a known STL model to the stitched scan.

Inputs:

- `stitched_cloud.ply`
- `Skeleton_Target.stl`

Outputs:

- `registration_output/model_to_scan_transform.txt`
- `registration_output/scan_to_model_transform.txt`
- `registration_output/registration_overlay.ply`
- `registration_output/registered_model_points.ply`
- `registration_output/scan_for_registration.ply`

### 15.1 Surface Sampling

An STL mesh has vertices and triangles. Registration needs point sets, so the
code samples points on the STL surface using `trimesh.sample.sample_surface`.

Current pipeline config:

```text
model_sample_count: 120000
model_scale: 0.001
```

The model points are scaled from millimeters to meters.

### 15.2 Scan Preprocessing

Before registration, scan points may be:

- Cropped by a bounding box.
- Radius-outlier filtered.
- Voxel-downsampled.

Current pipeline config:

```text
scan_voxel_size: 0.008
model_voxel_size: 0.008
crop_min: [0.1, -0.5, 0.0]
crop_max: [0.5, 0.5, 0.7]
```

### 15.3 PCA Initial Alignment

ICP needs a reasonable initial transform. The code computes one using PCA.

PCA idea:

1. Compute centroid of each point cloud.
2. Center points around the centroid.
3. Compute covariance.
4. Compute eigenvectors of covariance.
5. Eigenvectors are principal axes of the shape.

The code computes PCA frames for:

- Source: scan points.
- Target: model points.

Principal axes have sign and order ambiguities. For example, an axis can point
in the opposite direction and still be a valid principal axis. The code handles
this by testing:

- All 6 permutations of the 3 axes.
- Proper sign flips with determinant `+1`.

For each candidate transform:

1. Align source axes to target axes.
2. Align source centroid to target centroid.
3. Transform a sampled scan.
4. Query nearest model points with a KD-tree.
5. Score with trimmed RMSE.

Trimmed RMSE keeps only the best fraction of distances. This makes the score
less sensitive to outliers and partial overlap.

### 15.4 ICP Refinement

After PCA alignment, the code runs point-to-point ICP.

ICP means Iterative Closest Point.

Each iteration:

1. Transform source scan points using the current transform.
2. Find nearest target model point for each transformed source point.
3. Keep correspondences under a max distance.
4. If too few correspondences pass that threshold, keep the nearest points
   anyway to satisfy the minimum correspondence count.
5. Trim to the best correspondence fraction.
6. Solve the best rigid transform between current source correspondences and
   target correspondences.
7. Compose the new correction into the running transform.
8. Stop if RMSE improvement is below `1e-5`.

Current config:

```text
icp_iterations: 60
icp_max_correspondence_distance: 0.04
icp_min_correspondences: 300
icp_trim_fraction: 0.75
```

### 15.5 Kabsch / Orthogonal Procrustes Solve

The rigid transform inside each ICP iteration is solved by SVD.

Given matched point sets:

```text
source_corr
target_corr
```

The code:

1. Computes both centroids.
2. Centers both point sets.
3. Computes covariance:

   ```text
   covariance = source_centered.T * target_centered
   ```

4. Computes SVD:

   ```text
   covariance = U * S * V.T
   ```

5. Computes rotation:

   ```text
   R = V * U.T
   ```

6. Fixes reflection if determinant is negative.
7. Computes translation:

   ```text
   t = target_centroid - R * source_centroid
   ```

This is the standard Kabsch algorithm.

### 15.6 Registration Transform Direction

The ICP result maps scan points to model points:

```text
scan_to_model
```

The package then inverts it:

```text
model_to_scan = inverse(scan_to_model)
```

The later target and entry stages use `model_to_scan`, because the target is
identified in model coordinates and must be expressed in the robot base frame.

## 16. Target Localization

Target localization is implemented in `target_locator.py`.

Command:

```bash
ros2 run rnm_mapping locate_stl_target
```

Purpose:

- Find the small spherical target in the STL model.
- Transform it into the registered scan frame.
- Validate it against the stitched scan.
- Publish and write the result.

### 16.1 Why Use the STL Instead of the Raw Scan?

The raw scan is noisy and incomplete. The STL is clean and contains semantic
geometry. Once the STL is registered to the scan, it is better to find the
target in the STL and transform the target into scan coordinates.

### 16.2 Voxelized Connected Components

The target detector:

1. Samples the STL surface.
2. Scales points to meters.
3. Converts points to voxel indices.
4. Builds a 3D boolean occupancy grid.
5. Uses `scipy.ndimage.label` to find connected occupied components.

The connectivity structure is a full `3 x 3 x 3` block, so voxels touching by
faces, edges, or corners are considered connected.

### 16.3 Spherical Component Filtering

For each connected component, the code computes:

- Bounding-box extents.
- Maximum extent.
- Minimum extent.
- Compactness:

  ```text
  compactness = min_extent / max_extent
  ```

A sphere-like object has similar extents in all directions, so compactness is
closer to `1.0`.

Current config:

```text
detection_voxel_size: 0.008
min_target_diameter: 0.012
max_target_diameter: 0.060
min_compactness: 0.45
```

Candidates are sorted by:

1. Higher compactness.
2. More occupied voxels.
3. Closer maximum extent to `0.025 m`.

The selected center is the mean of the component voxel centers. The estimated
radius is half of the maximum component extent.

### 16.4 Transforming Target to Scan Frame

The detected target starts in model coordinates. The code applies:

```text
model_to_scan_transform.txt
```

to compute:

```text
scan_center_m
```

This is the target point in the robot base frame if the stitched scan was built
in `panda_link0`.

### 16.5 Target Validation

The target is checked against the stitched scan:

1. Is it inside the scan bounding box with margin?
2. How far is the nearest scan point?
3. How many scan points are within a support radius?

Current config:

```text
bbox_margin: 0.03
support_radius: 0.05
min_support_points: 20
```

If support is too low, the code marks `inside_bbox` false in the validation
object. It still writes the report, so users should check validation fields.

### 16.6 Current Target Output Example

The current `target_output/target_location.txt` reports:

```text
scan_center_m: [0.235670, -0.240813, 0.074981]
estimated_radius_m: 0.012000
inside_scan_bbox_with_margin: True
nearest_scan_distance_m: 0.004086
support_points: 212
```

This is a plausible target because it is inside the scan bounds, close to scan
points, and has sufficient local support.

## 17. Needle Entry Planning

Needle entry planning is implemented across:

- `needle_entry.py`: pure geometry and scoring.
- `needle_entry_planner.py`: CLI, config loading, reports, overlays, and ROS
  publishing.

Command:

```bash
ros2 run rnm_mapping find_needle_entry
```

Purpose:

- Load the registered STL mesh.
- Load the target point.
- Generate candidate entry points on a plane above the object.
- Reject candidates that are too long, blocked by the mesh, outside angle
  limits, or too close to bone/model geometry.
- Rank valid candidates.
- Write and publish the selected entry.

### 17.1 Registered Mesh Loading

The planner loads:

- STL model.
- `model_to_scan_transform.txt`.
- Model scale.

It creates a mesh in scan/base coordinates. That means all candidate planning is
performed in the same frame as the robot and target.

### 17.2 Approach Axis

The approach axis defines which direction is considered "from above" for the
entry sampling plane.

Current config:

```text
approach_axis: [0.0, 0.0, 1.0]
```

This means the planner samples a plane above the mesh along positive Z.

### 17.3 Candidate Plane Generation

The planner creates an orthonormal basis perpendicular to the approach axis.

Algorithm:

1. Choose two perpendicular basis vectors on the plane.
2. Project all mesh vertices onto the approach axis.
3. Find the top projection.
4. Place the candidate plane at:

   ```text
   top + plane_clearance
   ```

5. Project mesh vertices onto the two plane basis axes.
6. Build a rectangular sampling region around those extents with margin.
7. Sample a grid using `plane_sample_spacing`.
8. If too many points would be generated, reduce grid density to respect
   `max_plane_samples`.

Current config:

```text
plane_clearance: 0.005
plane_sample_spacing: 0.005
plane_margin: 0.02
max_plane_samples: 20000
```

### 17.4 Candidate Direction and Needle Length

For each candidate entry point:

```text
direction = unit(entry - target)
path_length = norm(entry - target)
```

Candidates are rejected if:

```text
path_length > needle_length
```

Current config:

```text
needle_length: 0.17
needle_diameter: 0.005
```

### 17.5 Mesh Ray Intersection

The planner treats the STL mesh as an obstacle. It casts rays from the target
toward each candidate entry direction.

A candidate is rejected if the ray intersects the mesh before reaching the
entry point, excluding a small region around the target.

The code first tries `trimesh` ray intersection. If that is unavailable, it
uses a fallback ray-triangle intersection implementation based on the
Moller-Trumbore method.

The fallback checks each triangle by solving for:

- Distance along the ray.
- Barycentric coordinates inside the triangle.

### 17.6 Target Exclusion

The target itself is part of the model. If the planner treated target geometry
as an obstacle, many valid rays would be rejected immediately. Therefore the
code ignores hits very close to the target.

The minimum ray distance is:

```text
max(target_radius + target_exclusion_margin, 0.5 * needle_diameter)
```

Current config:

```text
target_exclusion_margin: 0.003
```

### 17.7 Path Elevation

Path elevation is measured relative to the horizontal plane through the target:

- `0 deg`: horizontal.
- `90 deg`: vertical.

The horizontal plane normal is:

```text
horizontal_axis: [0.0, 0.0, 1.0]
```

The code computes:

```text
vertical_alignment = abs(dot(ray_direction, horizontal_axis))
path_elevation = asin(vertical_alignment)
```

Current config:

```text
min_path_elevation_deg: 10
max_path_elevation_deg: 70.0
```

### 17.8 Clearance Sampling

The planner checks how close the entry-to-target path comes to the model
surface.

Algorithm:

1. Sample many points from the registered mesh surface.
2. Optionally remove points around the target from the clearance set.
3. Build a KD-tree from clearance points.
4. Sample points along the candidate needle segment.
5. Exclude a small distance near the entry and target endpoints.
6. Query nearest clearance point for each segment sample.
7. Compute minimum and mean clearance.

Current config:

```text
min_path_clearance: 0.005
preferred_path_clearance: 0.030
path_clearance_sample_spacing: 0.0025
path_clearance_entry_exclusion: 0.008
path_clearance_surface_sample_count: 120000
exclude_target_from_clearance: true
```

Candidates with minimum clearance below `min_path_clearance` are rejected.
Preferred clearance does not reject candidates. It only improves the score.

### 17.9 Entry Region Bounds

The planner can restrict entry points to a box:

```text
entry_region_min: [x, y, z]
entry_region_max: [x, y, z]
```

The current config leaves these empty, so region filtering is disabled.

### 17.10 Candidate Scoring

Hard constraints run first. Only valid candidates are scored.

Score components:

- `axis_alignment_score`: dot product between ray direction and approach axis.
- `elevation_score`: best near the middle of the allowed elevation range.
- `min_clearance_score`: saturated ratio of actual min clearance to preferred
  clearance.
- `mean_clearance_score`: saturated ratio of mean clearance to preferred
  clearance.
- `length_score`: shorter paths score higher.

Weighted score:

```text
score =
    axis_weight * axis_alignment_score
  + elevation_weight * elevation_score
  + min_clearance_weight * min_clearance_score
  + mean_clearance_weight * mean_clearance_score
  + length_weight * length_score
```

Current config:

```text
score_axis_alignment_weight: 1.0
score_elevation_weight: 1.0
score_min_clearance_weight: 6.0
score_mean_clearance_weight: 1.0
score_length_weight: 0.25
```

Minimum clearance is intentionally dominant.

### 17.11 Entry Pose Orientation

The selected entry pose uses an orientation whose local Z axis points from
entry to target:

```text
insertion_axis = unit(target - entry)
```

The code builds an orthonormal rotation matrix with this Z axis, then converts
it to a quaternion.

This matches the later needle insertion assumption: local Z is the needle axis.

### 17.12 Current Entry Output Example

The current `entry_point_output/needle_entry_point.txt` reports:

```text
target_point_m: [0.235670, -0.240813, 0.074981]
entry_point_m: [0.259047, -0.360520, 0.177629]
insertion_axis_entry_to_target: [-0.146645, 0.750918, -0.643908]
path_length_m: 0.159414
path_elevation_deg: 40.083865
actual_min_path_clearance_m: 0.005775
actual_mean_path_clearance_m: 0.029185
valid_candidates: 286
recorded_candidates: 10
```

The current selected path is just under a 160 mm needle length and passes the
5 mm hard clearance threshold.

## 18. Combined Target and Entry PoseArray

`target_entry_pose_array_publisher.py` publishes one `PoseArray`.

Input reports:

- `target_output/target_location.txt`
- `entry_point_output/needle_entry_point.txt`

The publisher reads:

- `scan_center_m` from the target report.
- `entry_point_m` from a selected candidate block in the entry report.

Pose order:

```text
poses[0] = selected entry
poses[1] = target
```

Both poses use the same orientation. The local Z axis points from entry to
target.

The default topic is:

```text
/needle_entry_target_poses
```

The current pipeline config sets:

```text
pose_array_topic: /needle_poses
publish: false
```

So the final PoseArray stage is available but disabled in the current YAML.

## 19. Mapping Pipeline Launch

`model_target_entry_pipeline.launch.py` runs the compute stages in sequence:

1. `register_stl_to_scan`
2. `locate_stl_target --no-publish`
3. `find_needle_entry --no-publish`
4. Optional `publish_target_entry_pose_array`

It uses `OnProcessExit` handlers:

- If a stage succeeds, start the next stage.
- If a stage fails, shut down the pipeline.

This launch file converts YAML config sections into CLI arguments.

## 20. Post-Handeye Pipeline Supervisor

`post_handeye_pipeline_supervisor.py` orchestrates the larger workflow after
hand-eye calibration.

It starts:

1. IK solver.
2. Trajectory node.
3. Cloud stitcher.
4. Scanning node.

Then it:

1. Waits for scan completion on `/scanning/complete`.
2. Calls the cloud stitcher's `/save_scan` service.
3. Stops scanning and cloud stitching.
4. Runs the model-target-entry mapping pipeline.
5. Waits for needle mounting delay.
6. Starts the insertion path node.

This is process orchestration, not a geometry algorithm, but it connects the
mapping result to the rest of the robot workflow.

## 21. Core Math and Algorithm Glossary

### 21.1 Rigid Transform

A rigid transform preserves distances and angles:

```text
p_out = R * p_in + t
```

It consists of a rotation and translation. It is used throughout calibration,
TF, registration, and insertion planning.

### 21.2 Homogeneous Transform

A 4 x 4 matrix combines rotation and translation:

```text
[R t]
[0 1]
```

It allows points to be transformed with one matrix multiplication.

### 21.3 Quaternion

A quaternion stores 3D orientation compactly:

```text
[x, y, z, w]
```

The code uses quaternions for ROS poses and TF. Rotation matrices are used for
linear algebra, then converted to quaternions for messages.

### 21.4 KD-tree

A KD-tree accelerates nearest-neighbor queries. The mapping package uses
`scipy.spatial.cKDTree` for:

- Outlier filtering.
- Cloud overlap testing.
- ICP correspondences.
- Target support validation.
- Needle path clearance.

### 21.5 Voxel Grid

A voxel grid partitions 3D space into cubes. It is used for:

- Downsampling point clouds.
- Averaging colors and support weights.
- Target component detection.

### 21.6 PCA

Principal Component Analysis estimates the main axes of a point cloud. It is
used for initial scan-to-model alignment before ICP.

### 21.7 ICP

Iterative Closest Point refines alignment between point sets by repeatedly
matching nearest points and solving a rigid transform.

### 21.8 Trimmed RMSE

Trimmed RMSE computes error only on the best fraction of matches. This helps
when the scan has outliers, missing model sections, or extra background.

### 21.9 Connected Components

Connected-component labeling groups adjacent occupied voxels. Target detection
uses it to isolate compact model components.

### 21.10 Ray Casting

Ray casting checks whether a line from the target to an entry candidate hits the
mesh before reaching the entry point. It is used to reject blocked needle paths.

### 21.11 Clearance Field Approximation

The planner approximates distance to the mesh by sampling mesh surface points
and querying nearest neighbors from path samples. This is not an exact signed
distance field, but it is a practical collision-clearance approximation.

## 22. Outputs Produced by the Current Workspace

Current generated artifacts include:

```text
stitched_cloud.ply
registration_output/model_to_scan_transform.txt
registration_output/scan_to_model_transform.txt
registration_output/registration_overlay.ply
target_output/target_location.txt
target_output/target_overlay.ply
entry_point_output/needle_entry_point.txt
entry_point_output/needle_entry_overlay.ply
target_output/target_entry_overlay.ply
```

The current target and entry reports show a complete run through registration,
target localization, and needle entry ranking.

## 23. Important Assumptions

The whole pipeline assumes:

- Camera intrinsics and RGB-depth extrinsics are valid.
- Hand-eye or static camera TF is valid.
- The TF tree has no duplicate child-frame publishers.
- Point cloud timestamps match the TF data.
- The incoming point cloud frame matches the actual point coordinates.
- The STL scale is correct.
- The STL contains a compact spherical target.
- The stitched scan contains enough of the model for registration and target
  validation.
- The registered model is accurate enough before target and entry outputs are
  trusted.

## 24. Practical Tuning Notes

For calibration:

- Use many target poses across the whole image, not only the center.
- Avoid motion blur.
- Avoid nearly identical views.
- Use IR for depth-camera calibration if available.
- Watch RMS error, but also visually inspect calibration and TF behavior.

For cloud stitching:

- Start with strict TF correctness checks.
- Prefer workspace cropping when the approximate object region is known.
- Increase voxel size if registration is slow or noisy.
- Increase `min_accumulated_observations` for denoising, but decrease it if
  thin surfaces disappear.
- Be careful with `min_cloud_overlap_ratio`; too high can reject useful views.

For registration:

- Crop the scan to the model region before registration.
- Use enough model samples to represent the STL, but rely on voxel downsampling
  for speed.
- Check `registration_overlay.ply` visually.
- If PCA picks the wrong symmetric alignment, tighter crop or additional
  geometric constraints may be needed.

For target detection:

- Tune diameter limits around the physical target.
- Lower `min_compactness` only if the target is incomplete in the mesh sample.
- Check validation fields in `target_location.txt`.

For entry planning:

- Use `entry_region_min` and `entry_region_max` if the robot or procedure only
  allows a specific entry zone.
- Treat `min_path_clearance` as a hard safety filter.
- Treat `preferred_path_clearance` and score weights as ranking preferences.
- Check the overlay PLY before trusting a new configuration.

## 25. Main Limitations to Keep in Mind

- The mapping package does not compute robot kinematics for stitching. It trusts
  TF completely.
- Registration has no semantic success guarantee. A low ICP RMSE is helpful,
  but the overlay should still be inspected.
- Target validation records plausibility, but it does not stop downstream
  stages by itself.
- The needle planner approximates clearance using sampled surface points, not a
  continuous exact collision model.
- Entry and target report parsing is text/regex based, so report format changes
  can break downstream readers.
- Many configs currently contain absolute paths from a specific local machine.
  Moving the workspace may require path cleanup.

## 26. Short Summary

`rnm_calibration` solves the camera geometry problem: it estimates or extracts
camera intrinsics, distortion, RGB-depth extrinsics, and publishes them as ROS
`CameraInfo` and TF.

`rnm_mapping` solves the scan-to-procedure geometry problem: it stitches
robot-mounted point clouds, aligns the known STL model to the scan, finds the
target in model geometry, transforms the target into robot coordinates, samples
and scores feasible needle entry candidates, and publishes the selected entry
and target for downstream insertion planning.
