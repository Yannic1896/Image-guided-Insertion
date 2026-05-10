# rnm_tools

Python ROS 2 package for RNM-specific tools and utilities.

## Included tools

- `urdf_visualizer`: Viser-based visualization node that subscribes to
  `/robot_description` and `/joint_states`

## Launch

```bash
ros2 launch rnm_tools visualizer.launch.py
```

The web UI is exposed on port `8080` by default.
