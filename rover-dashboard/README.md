# rover-dashboard

## Overview
The `rover-dashboard` is a web-based Graphical User Interface (GUI) designed to provide remote teleoperation and monitoring capabilities for the Rover. It is built using standard web technologies (HTML, CSS, JavaScript) and relies on the `rosbridge_suite` to communicate seamlessly with the ROS 2 network via WebSockets.

## Features
- **Real-Time Telemetry**: Displays current velocity, odometry, and system health status.
- **Hardware Control**: Buttons and interfaces to actuate the main rover lift and camera lift using the custom Action Servers defined in `rover_msgs`.
- **Teleoperation**: Virtual joystick for publishing `/cmd_vel` commands directly to the `rover_base` node.
- **Cross-Platform**: Accessible from any modern web browser on the same network as the rover, eliminating the need to install ROS 2 on the operator's machine.

## Dependencies
To allow the dashboard to communicate with the ROS 2 system, you must have the `rosbridge_server` package installed and running:

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Usage
1. Ensure the ROS 2 rover stack and the `rosbridge_server` are actively running on the rover's compute unit.
2. If accessing locally, simply open the `index.html` file in any modern web browser.
3. Connect to the WebSocket IP address of the rover.

## Hosting and Remote Access

To control the rover from another device (like a phone, tablet, or separate laptop) on the same Wi-Fi network, the dashboard must be served over HTTP.

### Option 1: Quick Testing (Python)
For rapid, temporary deployment without requiring root privileges:
1. Navigate to the dashboard directory:
   ```bash
   cd ~/rover_ws/src/rover-dashboard
   ```
2. Start the built-in Python web server on port 8000:
   ```bash
   python3 -m http.server 8000
   ```
3. Find the rover's IP address (e.g., `192.168.1.50`) by running `ip a` in a new terminal.
4. On your remote device, open a browser and navigate to `http://<ROVER_IP>:8000`.

### Option 2: Production Deployment (Apache)
For a robust, permanent deployment that automatically serves the dashboard whenever the rover boots:
1. Install the Apache web server:
   ```bash
   sudo apt install apache2
   ```
2. Create a symbolic link from your ROS 2 workspace to the Apache web root. This ensures any updates pulled from GitHub are immediately reflected on the server:
   ```bash
   sudo ln -s $(pwd) /var/www/html/rover
   ```
   *(Run this command from inside the `rover-dashboard` directory).*
3. Restart the Apache service to apply changes:
   ```bash
   sudo systemctl restart apache2
   ```
4. On your remote device, open a browser and navigate to `http://<ROVER_IP>/rover`.
