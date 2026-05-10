# RNM Lab Devcontainer

This repository is a VS Code devcontainer wrapper for lab sessions that need:

- ROS 2 Humble on Ubuntu 22.04
- `multipanda_ros2` already built into the image under `/opt/ros2_ws`
- MuJoCo 3.2.0, Eigen 3.3.9, `libfranka` 0.9.2, and DQ Robotics preinstalled in the image

The container layout mirrors the upstream `multipanda_ros2` setup documented in its `humble` branch: ROS 2 Humble, MuJoCo, `libfranka`, and `rosdep`, while keeping user code in a separate bind-mounted ROS workspace.

## Getting started

1. Open the folder in VS Code and choose `Dev Containers: Reopen in Container`.
2. Put your own packages under `ros2_ws/src`.
3. Use `/opt/ros2_ws` as the baked upstream workspace when you need to inspect the bundled `multipanda_ros2` stack.

### Example Franka Simulation

Launch the simulation with the integrated RNM visualizer:
```bash
ros2 launch rnm_tools rnm_panda_sim.launch.py
```

Then open the visualization in your browser at `http://localhost:8080`.

To send a position command to the controller, for example:
```bash
ros2 topic pub --once /joint_position_example_controller/joint_command std_msgs/msg/Float64MultiArray "{data: [0.05, -0.78, 0.0, -2.40, 0.0, 1.57, 0.79]}"
```

If you want to use ROS GUI tools but are unable to do so, due to issues related to GPU/driver pass through, you can start noVNC:
```bash
novnc
```

Then open the displayed URL, or go directly to `http://localhost:6080/vnc.html?autoconnect=1&resize=scale` and click `connect`.

Commands that should render into the noVNC desktop need the display variable set, for example:
```bash
DISPLAY=:99 ros2 run rqt_controller_manager rqt_controller_manager
```

## Usage notes

- The devcontainer can start without `.env`. If you need host networking for ROS 2 discovery, copy either `.env.example` or `.devcontainer/.env.example` to the matching `.env` file and adjust `DEVCONTAINER_NETWORK_MODE`.
- The image build needs internet access because it installs Ubuntu and ROS dependencies, downloads MuJoCo, builds `libfranka`, clones `multipanda_ros2`, and builds the workspace template.
- Changes under `/opt/ros2_ws` are part of the image. If you need to update that bundled dependency workspace, rebuild the devcontainer image.
- Docker Compose publishes `6080` and `8080` directly on the host. VS Code port auto-forwarding is disabled so it does not create duplicate tunnels on top of those published ports.
- For noVNC, run `novnc` inside the container and then open `http://localhost:6080/vnc.html?autoconnect=1&resize=scale` in a normal browser. Port `5901` is only the internal `x11vnc` backend.
- GUI applications that should appear inside the noVNC desktop need `DISPLAY=:99`, for example `DISPLAY=:99 ros2 run rqt_controller_manager rqt_controller_manager`.

## Common commands

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash
source /rnm-workspace/ros2_ws/install/setup.bash
ros2 pkg list | grep panda
```

If you need to rebuild after changing sources:

```bash
cd /rnm-workspace/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
