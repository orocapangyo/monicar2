#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  imu_parameter = LaunchConfiguration(
    'imu_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_imuconverter'),
      'param/imu.yaml'
    )
  )
  
  return LaunchDescription([
    Node(
        package='monicar2_imuconverter', executable='imuconverter', name='imu_node',
        output='screen',
        parameters=[imu_parameter],
    ),
  ])
