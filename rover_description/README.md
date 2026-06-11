# rover_description

## Overview
The `rover_description` package contains the physical and visual modeling data for the Rover. This includes the Unified Robot Description Format (URDF) files, visual meshes, collision geometries, and RViz configuration files necessary for simulation and visualization.

## Contents

- **`/urdf`**: Contains `rover.urdf`, which defines the kinematic tree, joints, links, and physical dimensions of the rover.
- **`/meshes`**: (If applicable) 3D model files (.stl or .dae) representing the rover's chassis, wheels, and lift mechanisms.
- **`/launch`**: Contains visualizer launch files like `display_rover.launch.py` to easily inspect the model in RViz.
- **`/rviz`**: Pre-configured RViz workspaces for optimal viewing.

## Nodes

This package does not contain custom C++ or Python executables. Instead, it relies on the standard ROS 2 `robot_state_publisher` and `joint_state_publisher` nodes to read the URDF and broadcast the `tf` transform tree.

## Usage

To visualize the rover model without hardware connected:
```bash
ros2 launch rover_description display_rover.launch.py
```
This will launch RViz 2 along with the `robot_state_publisher` and a `joint_state_publisher_gui` to allow you to manually manipulate the robot's joints.
