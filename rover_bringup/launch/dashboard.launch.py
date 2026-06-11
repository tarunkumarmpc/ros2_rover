from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('url_path', default_value='/'),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('retry_startup_delay', default_value='5.0'),
        DeclareLaunchArgument('fragment_timeout', default_value='600'),
        DeclareLaunchArgument('delay_between_messages', default_value='0'),
        DeclareLaunchArgument('max_message_size', default_value='10000000'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0'),
        DeclareLaunchArgument('use_compression', default_value='false'),
        DeclareLaunchArgument('call_services_in_new_thread', default_value='true'),
        DeclareLaunchArgument('default_call_service_timeout', default_value='5.0'),
        DeclareLaunchArgument('send_action_goals_in_new_thread', default_value='true'),
        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),
        DeclareLaunchArgument('params_timeout', default_value='5.0'),
        DeclareLaunchArgument('bson_only_mode', default_value='false'),

        # SSL group
        GroupAction(
            condition=IfCondition(LaunchConfiguration('ssl')),
            actions=[
                Node(
                    package='rosbridge_server',
                    executable='rosbridge_websocket',
                    name='rosbridge_websocket',
                    namespace=LaunchConfiguration('namespace'),
                    output='screen',
                    parameters=[{
                        'port': LaunchConfiguration('port'),
                        'address': LaunchConfiguration('address'),
                        'url_path': LaunchConfiguration('url_path'),
                        'retry_startup_delay': LaunchConfiguration('retry_startup_delay'),
                        'fragment_timeout': LaunchConfiguration('fragment_timeout'),
                        'delay_between_messages': LaunchConfiguration('delay_between_messages'),
                        'max_message_size': LaunchConfiguration('max_message_size'),
                        'unregister_timeout': LaunchConfiguration('unregister_timeout'),
                        'use_compression': LaunchConfiguration('use_compression'),
                        'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread'),
                        'default_call_service_timeout': LaunchConfiguration('default_call_service_timeout'),
                        'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread'),
                        'topics_glob': LaunchConfiguration('topics_glob'),
                        'services_glob': LaunchConfiguration('services_glob'),
                        'params_glob': LaunchConfiguration('params_glob'),
                        'bson_only_mode': LaunchConfiguration('bson_only_mode'),
                        'certfile': LaunchConfiguration('certfile'),
                        'keyfile': LaunchConfiguration('keyfile'),
                    }]
                )
            ]
        ),

        # Non-SSL group
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('ssl')),
            actions=[
                Node(
                    package='rosbridge_server',
                    executable='rosbridge_websocket',
                    name='rosbridge_websocket',
                    namespace=LaunchConfiguration('namespace'),
                    output='screen',
                    parameters=[{
                        'port': LaunchConfiguration('port'),
                        'address': LaunchConfiguration('address'),
                        'url_path': LaunchConfiguration('url_path'),
                        'retry_startup_delay': LaunchConfiguration('retry_startup_delay'),
                        'fragment_timeout': LaunchConfiguration('fragment_timeout'),
                        'delay_between_messages': LaunchConfiguration('delay_between_messages'),
                        'max_message_size': LaunchConfiguration('max_message_size'),
                        'unregister_timeout': LaunchConfiguration('unregister_timeout'),
                        'use_compression': LaunchConfiguration('use_compression'),
                        'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread'),
                        'default_call_service_timeout': LaunchConfiguration('default_call_service_timeout'),
                        'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread'),
                        'topics_glob': LaunchConfiguration('topics_glob'),
                        'services_glob': LaunchConfiguration('services_glob'),
                        'params_glob': LaunchConfiguration('params_glob'),
                        'bson_only_mode': LaunchConfiguration('bson_only_mode'),
                    }]
                )
            ]
        ),

        # rosapi_node
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'topics_glob': LaunchConfiguration('topics_glob'),
                'services_glob': LaunchConfiguration('services_glob'),
                'params_glob': LaunchConfiguration('params_glob'),
                'params_timeout': LaunchConfiguration('params_timeout'),
            }]
        ),

        # web_video_server node with optimized parameters
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': 8081,
                'address': '0.0.0.0',
                'server_threads': 2,
                'ros_threads': 2,
                'default_stream_type': 'ros_compressed',
                'publish_rate': 10.0,
                'verbose': False,
            }]
        )
    ])

