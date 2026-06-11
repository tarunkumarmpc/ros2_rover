# rover_msgs

## Overview
The `rover_msgs` package contains custom ROS 2 interface definitions (`.msg` and `.action` files) required by the rover. It provides the minimal set of interface definitions required by the system.

## Custom Message Types (`msg`)

- **`LiftData.msg`**: A low-level data structure representing the current state, ID, and read time of a physical lift actuator.
- **`RoverLiftStatus.msg`**: An array of `LiftData` messages representing the aggregate status of the main rover lift.
- **`CameraLiftStatus.msg`**: An array of `LiftData` messages representing the aggregate status of the camera lift mast.

## Custom Action Types (`action`)

Actions provide robust, preemptable, and feedback-rich communication for long-running hardware tasks (like moving a physical lift to a specified height).

- **`RoverLift.action`**: 
  - **Goal**: Target height (`float64`).
  - **Result**: Success flag (`bool`) and a status message (`string`).
  - **Feedback**: Current height (`float64`).
- **`CameraLift.action`**: 
  - **Goal**: Target height (`float64`).
  - **Result**: Success flag (`bool`) and a status message (`string`).
  - **Feedback**: Current height (`float64`).

## Dependencies
This package relies on:
- `std_msgs`
- `geometry_msgs`
- `action_msgs`
- `rosidl_default_generators`

## Usage
If you are developing a C++ or Python node that needs to interface with the Rover, you must depend on this package in your `CMakeLists.txt` or `package.xml`.

```xml
<!-- package.xml -->
<depend>rover_msgs</depend>
```

```cmake
# CMakeLists.txt
find_package(rover_msgs REQUIRED)
ament_target_dependencies(my_node rover_msgs)
```
