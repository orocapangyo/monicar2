#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  ekf_parameter = LaunchConfiguration(
    'ekf_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_localization'),
      'param/ekf.yaml'
    )
  ),

  return LaunchDescription([
    # Start robot localization using an Extended Kalman filter
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_parameter],
        emulate_tty=True,
    ),
  ])
