from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_url_arg = DeclareLaunchArgument(
        'robot_url',
        default_value='http://192.168.86.28/events',
        description='URL for the robot state events server'
    )
    
    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter',
        default_value='0.072',
        description='Wheel diameter in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.232',
        description='Wheel separation in meters'
    )

    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_revolution',
        default_value='508.8',
        description='Encoder ticks per wheel revolution'
    )

    correction_factor = DeclareLaunchArgument(
        'correction_factor',
        default_value='0.7928',
        description='Correction factor'
    )

    # Create node with parameters
    robot_state_node = Node(
        package='my_bot',  # Replace with your package name
        executable='robot_state_node.py',  # Replace with your node's executable name
        name='robot_state_node',
        parameters=[{
            'robot_url': LaunchConfiguration('robot_url'),
            'wheel_diameter': LaunchConfiguration('wheel_diameter'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'ticks_per_revolution': LaunchConfiguration('ticks_per_revolution'),
        }],
        output='screen'
    )

    # Differential drive control node
    drive_node = Node(
        package='my_bot',
        executable='diff_drive_node.py',
        name='diff_drive_node',
        parameters=[{
            'robot_url': LaunchConfiguration('robot_url'),
            'wheel_diameter': LaunchConfiguration('wheel_diameter'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            # Additional control parameters
            'max_wheel_speed': 500,
            'min_wheel_speed': 50,
            'max_acceleration': 200,
            'max_deceleration': 300,
            'max_jerk': 500,
            'command_timeout': 0.5
        }],
        output='screen'
    )

    # This node is responsible for providing a static transform from the robot's base_footprint
    # frame to a new laser_frame, which will be the coordinate frame for the lidar.

    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.05', '0.10', '0.10', '0', '0', '0', 'base_link', 'laser'],
    )

    return LaunchDescription([
        robot_url_arg,
        wheel_diameter_arg,
        wheel_separation_arg,
        ticks_per_rev_arg,
        robot_state_node,
        drive_node,
        laser_tf_node
    ])
