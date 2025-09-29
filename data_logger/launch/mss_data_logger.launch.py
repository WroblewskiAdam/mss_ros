#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file dla MSS Data Logger"""
    return LaunchDescription([
        Node(
            package='data_logger',
            executable='mss_data_logger_node',
            name='mss_data_logger_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'log_directory': '~/mss_logs',
                'log_frequency_hz': 10.0,
            }]
        ),
    ])
