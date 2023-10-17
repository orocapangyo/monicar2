#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([

    Node(
        package='monicar2_localization', executable='rviz2ClickTo2d', name='rviz2_click_node',
        output='screen',
        emulate_tty=True,
    ),

  ])