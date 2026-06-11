# rover_base

## Overview
The `rover_base` package serves as the core hardware interface and control logic for the rover. It bridges the gap between high-level ROS 2 velocity commands (`cmd_vel`) and the low-level physical actuators via the CAN bus. 

It implements mecanum kinematics, robust velocity smoothing, hardware E-Stop handling, and node interfaces for auxiliary systems such as the main lift and the camera lift.

## Nodes

### 1. `rover_wheel_control`
Translates `/cmd_vel` into individual wheel speeds based on mecanum kinematics, applies acceleration/deceleration smoothing, and manages emergency stops. It calculates and broadcasts odometry (`/odom`).

**Subscribed Topics:**
- `/cmd_vel` (`geometry_msgs/Twist`): Target velocity commands.
- `/canbus/rx` (`can_msgs/Frame`): Feedback from the motor controllers.

**Published Topics:**
- `/canbus/tx` (`can_msgs/Frame`): Target velocities sent to motor controllers.
- `/odom` (`nav_msgs/Odometry`): Calculated odometry based on wheel feedback.
- `/tf` (`tf2_msgs/TFMessage`): Transform from `odom` to `base_link`.

**Services:**
- `/reset_odometry` (`std_srvs/Trigger`): Resets the internal odometry to (0,0).
- `/enable_cmd_vel` (`std_srvs/SetBool`): Enables or disables velocity processing.
- `/estop` (`std_srvs/SetBool`): Emergency hardware stop.

**Parameters:**
- `wheel_radius` (double): Physical radius of the wheels.
- `lx`, `ly` (double): Distance from rover center to wheels (width/length).
- `accel_smooth_duration`, `decel_smooth_duration` (double): Tuning for velocity smoothing.

---

### 2. `rover_lift_control`
Manages the hardware actuation of the main rover lift plate via CAN bus. Exposes a ROS 2 Action Server for robust, preemptable height control.

**Action Servers:**
- `/rover_lift` (`rover_msgs/action/RoverLift`): Accepts a target height and provides continuous feedback as the lift moves.

**Subscribed Topics:**
- `/canbus/rx` (`can_msgs/Frame`): Feedback from the lift motor.

**Published Topics:**
- `/canbus/tx` (`can_msgs/Frame`): Motor commands.
- `rover_lift/status` (`rover_msgs/RoverLiftStatus`): High-level lift state data.

---

### 3. `camera_lift_control`
Controls the auxiliary camera lift mast. Similar to the main lift, it uses an Action Server to smoothly adjust the camera's height.

**Action Servers:**
- `/camera_lift` (`rover_msgs/action/CameraLift`): Accepts a target height for the camera lift.

---

### 4. `guidance_mode`
An experimental vision-based guidance node leveraging OpenCV and ArUco markers to direct the rover.

## Usage
Typically, these nodes are not launched individually. Use the launch files provided in `rover_bringup` to start the entire hardware abstraction layer.
