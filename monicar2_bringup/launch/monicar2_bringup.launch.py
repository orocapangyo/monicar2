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


def generate_launch_description():
  monicar2_mcu_parameter = LaunchConfiguration(
    'monicar2_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_bringup'),
      'param/monicar2_mcu.yaml'
    )
  )

  monicar2_lidar_parameter = LaunchConfiguration(
    'monicar2_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_bringup'),
      'param/monicar2_lidar.yaml'
    )
  )

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  monicar2_description_dir = LaunchConfiguration(
    'monicar2_description_dir',
    default=os.path.join(
      get_package_share_directory('monicar2_description'),
      'launch'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'monicar2_mcu_parameter',
      default_value=monicar2_mcu_parameter
    ),

    DeclareLaunchArgument(
      'monicar2_lidar_parameter',
      default_value=monicar2_lidar_parameter
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/monicar2_mcu.launch.py']),
      launch_arguments={'monicar2_mcu_parameter': monicar2_mcu_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/monicar2_lidar.launch.py']),
      launch_arguments={'monicar2_lidar_parameter': monicar2_lidar_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([monicar2_description_dir, '/monicar2_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
  ])
