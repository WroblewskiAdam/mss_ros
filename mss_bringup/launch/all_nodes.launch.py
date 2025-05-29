# mss_bringup/launch/all_nodes.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_rtk_reader',
            executable='gps_rtk_node',
            name='gps_rtk_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='imu_reader',
            executable='imu_node',
            name='imu_publisher',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='servo_controller',
            executable='servo_node',
            name='servo_controller',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gear_controller',
            executable='gear_shifter',
            name='gear_shifter',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='speed_controller',
            executable='speed_filter_node',
            name='speed_filter_node',
            output='screen',
            emulate_tty=True,
        ),
    ])