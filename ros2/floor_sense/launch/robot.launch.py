"""
Start the bridge and the dashboard together.

    ros2 launch floor_sense robot.launch.py
    ros2 launch floor_sense robot.launch.py port:=/tmp/floorsense_fake
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = ParameterValue(LaunchConfiguration('port'), value_type=str)
    http_port = ParameterValue(LaunchConfiguration('http_port'), value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the Nucleo'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='Port of the dashboard web page'),
        Node(package='floor_sense', executable='bridge', name='floor_sense_bridge',
             parameters=[{'port': port}], output='screen'),
        Node(package='floor_sense', executable='dashboard', name='floor_sense_dashboard',
             parameters=[{'http_port': http_port}], output='screen'),
    ])
