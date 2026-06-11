# rover_core

## Overview
The `rover_core` package acts as a metapackage and orchestrator for the rover system. It provides top-level launch files that can be used to coordinate multiple subsystems simultaneously.

## Contents
This package primarily consists of high-level configurations and overarching launch architecture.

- **`/launch`**: Contains `rover.launch.py`, a centralized launch script designed to encapsulate the hardware (`rover_bringup`), perception, and navigation stacks in a single, configurable command.

## Usage
To execute the overarching system launch:
```bash
ros2 launch rover_core rover.launch.py
```
*(Note: Depending on your specific deployment, you may use `rover_bringup` directly for hardware-only testing, or `rover_core` for full autonomous operation).*
