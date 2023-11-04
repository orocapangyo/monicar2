#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee

# Standard library imports
# https://answers.ros.org/question/382167/is-it-possible-to-have-a-conditional-in-a-launch-file-using-declarelaunchargument/
from os.path import join

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  initPose = LaunchConfiguration('initPose', default='true')
  use_ekf = LaunchConfiguration('use_ekf', default='false')
  use_des = LaunchConfiguration('use_des', default='false')
  use_joy = LaunchConfiguration('use_joy', default='false')

  declare_initPose = DeclareLaunchArgument(
    name='initPose', 
    default_value=initPose, 
    description='Whether initPose is clicked already')
    
  declare_use_ekf = DeclareLaunchArgument(
    name='use_ekf', 
    default_value=use_ekf, 
    description='Whether to start ekfnode')

  declare_use_desc= DeclareLaunchArgument(
    name='use_des', 
    default_value=use_des, 
    description='Whether to start robot description')

  declare_use_joy= DeclareLaunchArgument(
    name='use_joy', 
    default_value=use_joy, 
    description='Whether to start joystick')

  run_odompub0 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      join(get_package_share_directory('monicar2_localization'), 'launch', 'odomPubInit1.launch.py')),
    condition=IfCondition(initPose)
  )

  run_odompub1 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      join(get_package_share_directory('monicar2_localization'), 'launch', 'odomPubInit0.launch.py')),
    condition=UnlessCondition(initPose)
  )

  run_rvizClick2d= IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      join(get_package_share_directory('monicar2_localization'), 'launch', 'rviz2ClickTo2d.launch.py')),
    condition=UnlessCondition(initPose)
  )

  run_ekf = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      join(get_package_share_directory('monicar2_localization'), 'launch', 'ekfnode.launch.py')),
    condition=IfCondition(use_ekf)
  )  

  run_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      join(get_package_share_directory('monicar2_description'), 'launch', 'state_publisher.launch.py')),
    condition=IfCondition(use_des)
  )

  run_joystick = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      join(get_package_share_directory('monicar2_teleop'), 'launch', 'teleop_joy.launch.py')),
    condition=IfCondition(use_joy)
  )

  # Create the launch description and populate
  ld = LaunchDescription()
  ld.add_action(declare_initPose)
  ld.add_action(declare_use_ekf)
  ld.add_action(declare_use_desc)
  ld.add_action(declare_use_joy)

  # Add any actions
  ld.add_action(run_odompub0)
  ld.add_action(run_odompub1)
  ld.add_action(run_rvizClick2d) 
  ld.add_action(run_ekf)
  ld.add_action(run_description)
  ld.add_action(run_joystick)

  return ld
