from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_base_ros2',
            executable='motion_node',  # Replace with your executable name
            output='screen'
        )
    ])
