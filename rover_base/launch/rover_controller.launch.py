"""
Launch file for starting all motion control nodes in the 'rover_base' package.

This launch file:
- Loads parameters from YAML config files.
- Applies configurable namespace and log level.
- Verifies that required configuration files exist before launching.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def check_config_files(context, *args, **kwargs):
    """
    Check that all required YAML configuration files exist before launching.
    Raises FileNotFoundError if any are missing.
    """
    pkg_share = FindPackageShare('rover_base').perform(context)
    config_files = [
        'config/camera_lift.yaml',
        'config/rover_lift.yaml',
        'config/rover_wheel.yaml'
    ]

    for rel_path in config_files:
        full_path = os.path.join(pkg_share, rel_path)
        if not os.path.exists(full_path):
            raise FileNotFoundError(f"Missing config file: {full_path}")

    return []  # No actions to return


def generate_launch_description():
    """
    Generate the launch description for motion control nodes.
    """

    # Declare launch arguments
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS logging level: debug, info, warn, error, fatal'
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='rover',
        description='Namespace for all nodes'
    )

    # Helper to create a node with shared settings
    def create_motion_node(name, executable, config_file):
        return Node(
            package='rover_base',
            executable=executable,
            name=name,
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('rover_base'),
                    config_file
                ])
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )

    # Define nodes using helper function
    camera_lift_control = create_motion_node(
        name='camera_lift_control',
        executable='camera_lift_control',
        config_file='config/camera_lift.yaml'
    )

    rover_lift_control = create_motion_node(
        name='lift_control',
        executable='rover_lift_control',
        config_file='config/rover_lift.yaml'
    )

    rover_wheel_control = create_motion_node(
        name='wheel_control',
        executable='rover_wheel_control',
        config_file='config/rover_wheel.yaml'
    )

    # Return final launch description
    return LaunchDescription([
        OpaqueFunction(function=check_config_files),
        log_level,
        namespace,
        camera_lift_control,
        rover_lift_control,
        rover_wheel_control
    ])