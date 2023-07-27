#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
  monicar2_mcu_parameter = LaunchConfiguration(
    'monicar2_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_bringup'),
      'param/monicar2_mcu.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'monicar2_mcu_parameter',
      default_value=monicar2_mcu_parameter
    ),

    Node(
      package='monicar2_bringup',
      executable='monicar2_mcu_node',
      name='monicar2_mcu_node',
      output='screen',
      emulate_tty=True,
      parameters=[monicar2_mcu_parameter],
      namespace='',
    )
  ])
