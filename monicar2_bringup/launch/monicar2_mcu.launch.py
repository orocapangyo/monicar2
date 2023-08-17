#!/usr/bin/env python3
# Author: Bishop Pearson
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/esp32Nodemcu"],
            output='screen'
        )
    ])
