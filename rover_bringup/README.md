# rover_bringup

## Overview
The `rover_bringup` package provides centralized launch configurations for the rover system. It consolidates necessary nodes from across the workspace (hardware controllers, descriptions, SocketCAN interfaces) into single-command launch files.

## Launch Files

- **`bringup.launch.py`**: The primary launch file for deploying the rover on actual hardware.
  - Initializes the `ros2_socketcan` bridge.
  - Starts the `rover_base` hardware control nodes (`rover_wheel_control`, `rover_lift_control`, `camera_lift_control`).
  - Broadcasts the physical robot description to the ROS 2 ecosystem.

## Usage
To bring up the physical rover system:
```bash
ros2 launch rover_bringup bringup.launch.py
```
*(Ensure the CAN bus interfaces, such as `can0`, are physically connected and configured on the host machine before launching).*
