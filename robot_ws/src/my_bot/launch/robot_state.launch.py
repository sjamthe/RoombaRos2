from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='http://192.168.86.28/events',
        description='URL for the robot state events server'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
        description='Wheel radius in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
        description='Wheel separation in meters'
    )

    # Create node with parameters
    robot_state_node = Node(
        package='my_bot',  # Replace with your package name
        executable='robot_state_node.py',  # Replace with your node's executable name
        name='robot_state_node',
        parameters=[{
            'server_url': LaunchConfiguration('server_url'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation')
        }],
        output='screen'
    )

    return LaunchDescription([
        server_url_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        robot_state_node
    ])