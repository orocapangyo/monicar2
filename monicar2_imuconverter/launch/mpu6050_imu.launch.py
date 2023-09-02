#!/usr/bin/env python3
# Author: Bishop Pearson
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='monicar2_imuconverter',
            executable='monicar2_imuconverter',
            name='monicar2_imu_node',
            output='screen',
        )
    ])