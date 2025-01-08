from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',  # Changed to lifelong node
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'transform_publish_period': 0.02,
            'max_laser_range': 12.0,
            'resolution': 0.05,
            'enable_interactive_mode': False,
            'debug_logging': True,
            'map_update_interval': 1.0,
            'minimum_time_interval': 0.5,
            'publish_period': 1.0,  # Added this
            'enable_mapping': True,  # Added this
            'max_update_rate': 10.0
        }],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    return LaunchDescription([slam_toolbox_node])
