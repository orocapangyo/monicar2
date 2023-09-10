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
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar2_bringup"), '/launch', '/mcu.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar2_imuconverter"), '/launch', '/mpu6050.launch.py'])
    ), 

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar2_bringup"), '/launch', '/rplidar.launch.py'])
    ), 
    
    Node(
        package='tf2_ros', executable='static_transform_publisher', name='baseLkTolaserLk', 
        arguments=['0.065', '0.0', '0.13', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen',
    ),

    Node(
        package='tf2_ros', executable='static_transform_publisher', name='baseFtToBaseLk', 
        arguments=['0.0', '0.0', '0.015', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen',
    ),

  ])
