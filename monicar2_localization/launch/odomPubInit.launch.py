#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  initPose = LaunchConfiguration('initPose', default='true')
  odom_parameter = LaunchConfiguration(
    'odom_parameter',
    default=os.path.join(
      get_package_share_directory('monicar2_localization'),
      'param/robot.yaml'
    )
  ),

  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar2_bringup"), '/launch', '/bringup.launch.py'])
    ),

    Node(
        package='monicar2_localization', executable='odomPublisher', name='odompub_node',
        output='screen',
        parameters=[odom_parameter,
                  {'initPoseRecieved': initPose}],
        emulate_tty=True,
    ),
  ])
