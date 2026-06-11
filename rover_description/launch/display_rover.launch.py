from launch import LaunchDescription
from launch_ros.actions import Node
import os

def load_urdf():
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'rover.urdf'
    )
    urdf_path = os.path.abspath(urdf_path)
    with open(urdf_path, 'r') as infp:
        return infp.read()

def generate_launch_description():
    robot_description = {'robot_description': load_urdf()}

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

    ])

