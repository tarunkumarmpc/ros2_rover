from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource

def generate_launch_description():
    # 1. CAN setup
    cansetup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'),
                'launch',
                'cansetup.launch.py'
            ])
        )
    )

    # 2. Motion base controller
    rover_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rover_base'),
                'launch',
                'rover_controller.launch.py'
            ])
        )
    )

    # 3. Rover description
    rover_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rover_description'),
                'launch',
                'display_rover.launch.py'
            ])
        )
    )

    # 4. ROSBridge websocket (XML launch, loaded last)
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        )
    )

    # Sequence with TimerActions
    motion_and_rover = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="CAN setup complete. Launching motion base and rover description..."),
            rover_base_launch,
            rover_description_launch,
        ]
    )

    rosbridge = TimerAction(
        period=10.0,  # Adjust as needed to ensure previous nodes are up
        actions=[
            LogInfo(msg="Motion base and rover description launched. Launching rosbridge websocket..."),
            rosbridge_launch
        ]
    )

    return LaunchDescription([
        LogInfo(msg="Starting CAN setup..."),
        cansetup_launch,
        motion_and_rover,
        rosbridge
    ])

