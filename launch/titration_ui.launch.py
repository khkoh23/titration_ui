from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    default_output_dir = os.path.join(os.environ.get('HOME', '/tmp'), 'titration_logs')

    log_rate_arg = DeclareLaunchArgument(
        'log_rate_hz',
        default_value='5.0',
        description='Fixed-rate logging frequency in Hz'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory to write CSV logs'
    )

    node = Node(
        package='titration_ui',
        executable='titration_ui',
        name='titration_ui',
        output='screen',
        parameters=[{
            'log_rate_hz': LaunchConfiguration('log_rate_hz'),
            'output_dir': LaunchConfiguration('output_dir'),
        }]
    )

    return LaunchDescription([
        log_rate_arg,
        output_dir_arg,
        node
    ])