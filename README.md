# RNM

This repository contains the software, workspace configuration, data artifacts, and documentation developed for the RNM project. Its ROS 2 components support a robot-assisted workflow that combines camera calibration, point-cloud acquisition and mapping, target and needle-entry estimation, kinematics, and trajectory execution.

## Repository Contents

The main project workspace is located in `rnm-workspace-main/`. It contains the ROS 2 source workspace, container definitions, generated artifacts, and workflow-specific data.

### ROS 2 Packages

The packages in `rnm-workspace-main/ros2_ws/src/` are organized by responsibility:

| Package | Purpose |
| --- | --- |
| `rnm_calibration` | Camera and system-calibration functionality. |
| `rnm_handeye` | Hand-eye calibration support between the robot and camera frames. |
| `rnm_kinematics` | Inverse-kinematics utilities for the robot. |
| `rnm_mapping` | Point-cloud stitching, model registration, target localization, and needle-entry estimation. |
| `rnm_needle` | Calculation of the needle insertion path. |
| `rnm_scanning` | Nodes and launch descriptions for scan acquisition. |
| `rnm_tools` | Shared utilities, TF helpers, visualization support, and Franka Panda simulation resources. |
| `rnm_trajectory` | Trajectory planning and execution components. |

Together, these packages implement the processing chain from calibrated sensor data to a planned robot motion.

### Supporting Resources

- `rnm-workspace-main/docker/` contains a lightweight ROS 2 Humble container environment.
- `rnm-workspace-main/.devcontainer/` contains the more complete VS Code development environment, including the Franka simulation stack.
- `rnm-workspace-main/docs/` contains technical notes and reports.
- `rnm-workspace-main/scanning_bag/` contains scan-bag-related input data.
- `rnm-workspace-main/registration_output/` stores generated model-to-scan registration results.
- `rnm-workspace-main/target_output/` stores generated target and needle-entry artifacts.
- `rnm-workspace-main/entry_point_output/` contains computed needle entry-point data.

## Workflow Overview

The project is structured around the following stages:

1. **Calibration** establishes the geometric relationship between the camera, end effector, and robot coordinate frames.
2. **Scanning and mapping** acquire point clouds and combine them into a coherent representation of the workspace.
3. **Registration and localization** align a reference model to the scan and determine the target location.
4. **Needle planning** determines a suitable entry point and insertion path toward the target.
5. **Kinematics and trajectory control** convert the planned path into feasible robot motion.

## Project Layout

```text
RNM/
├── README.md                    # This repository overview
└── rnm-workspace-main/
    ├── ros2_ws/                 # ROS 2 workspace and project packages
    ├── docker/                  # Lightweight ROS 2 container setup
    ├── .devcontainer/           # Full VS Code development environment
    ├── docs/                    # Reports and technical documentation
    ├── scanning_bag/            # Scan input data
    ├── registration_output/     # Registration results
    ├── target_output/           # Target-localization results
    └── entry_point_output/      # Needle-entry-point results
```

## Further Documentation

A detailed project description is attached as a PDF.
Further workspace documentation, including environment setup and ROS 2 usage, is available in [rnm-workspace-main/README.md](rnm-workspace-main/README.md).

