#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  useImu = LaunchConfiguration('useImu', default='true')
  imu_parameter = LaunchConfiguration(
    'imu_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_localization'),
      'param/robot.yaml'
    )
  )
  
  return LaunchDescription([
    Node(
        package='monicar2_imuconverter', executable='imuconverter', name='imu_node',
        output='screen',
        parameters=[imu_parameter,
                  {'useImu': useImu}],
        emulate_tty=True,        
    ),
  ])
