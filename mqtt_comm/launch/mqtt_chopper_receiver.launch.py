#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file dla MQTT Chopper Receiver"""
    return LaunchDescription([
        Node(
            package='mqtt_comm',
            executable='mqtt_node',
            name='mqtt_chopper_receiver_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mqtt_broker_host': 'mss-mqtt.ddns.net',
                'mqtt_broker_port': 1883,
                'mqtt_topic_chopper': 'test/polaczenia',
                'mqtt_keepalive': 60,
                'mqtt_username': '',
                'mqtt_password': '',
                'reconnect_interval': 5.0,
            }]
        ),
    ])
