# ros2_socketcan

## Overview
The `ros2_socketcan` package is a placeholder/submodule for the standard ROS 2 SocketCAN bridge. It is designed to expose a physical CAN interface (e.g., `can0`) to the ROS 2 ecosystem.

## Dependencies
If this directory is empty, it means the `ros2_socketcan` submodule has not been initialized. It is typically cloned from the official `ros2_socketcan` repository or installed via binaries:

```bash
sudo apt install ros-$ROS_DISTRO-ros2-socketcan
```

## Functionality
When active, this node provides a bi-directional bridge:
- **SocketCAN Receiver**: Reads raw CAN frames from the Linux network interface (`can0`) and publishes them as `can_msgs/Frame` to the `/canbus/rx` topic.
- **SocketCAN Sender**: Subscribes to the `/canbus/tx` topic and writes the `can_msgs/Frame` data to the physical CAN bus.
