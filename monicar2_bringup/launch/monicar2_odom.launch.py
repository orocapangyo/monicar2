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

def generate_launch_description():
  odom_parameter = LaunchConfiguration(
    'odom_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_bringup'),
      'param/monicar2_odom.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('odomparameter', default_value=odom_parameter
    ),

    Node(
        package='monicar2_bringup', executable='monicar2_odom', name='odom_node',
        output='screen',
        parameters=[odom_parameter],
    ),

  ])
