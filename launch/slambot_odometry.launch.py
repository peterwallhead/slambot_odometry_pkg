from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0'],
            output='screen',
        ),
        Node(
            package='slambot_odometry_pkg',
            executable='odometry_publisher'
        )
    ])
