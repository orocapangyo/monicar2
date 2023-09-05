#!/usr/bin/env python
# Author: ChangeWhan Lee

# euler_from_quaternion, quaternion_from_euler(
# https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434

import math
from math import sin, cos, pi
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Rviz2Click2To2d(Node):

    def __init__(self):
        super().__init__('rviz2_click_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('initialPose', None),
            ])

        print('rvRviz2Click2To2d created')

        # Get parameter values
        #self.initialPose = self.get_parameter_or('initialPose', Parameter('initialPose', Parameter.Type.INTEGER, 1)).get_parameter_value().integer_value
        #print('initialPose: %d' %(self.initialPose) )

        self.goalPub = self.create_publisher(PoseStamped, 'goal_2d', 10)
        self.initPub = self.create_publisher(PoseStamped, 'initial_2d', 10)

        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.handle_goal, 10)
        self.init_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.handle_initial_pose, 10)

         # Initialize the transform broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)
        # Mark current time
        self.last_time = self.get_clock().now()
        self.current_time = self.last_time
        print('Init done')

    def handle_goal(self, msg):
        print('goal clicked')
        rpyGoal =  PoseStamped()
        rpyGoal.header.frame_id = "map"
        rpyGoal.header.stamp = msg.header.stamp
        rpyGoal.pose.position.x = msg.pose.position.x
        rpyGoal.pose.position.y = msg.pose.position.y
        rpyGoal.pose.position.z = 0.0

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = msg.pose.orientation.z
        q.w = msg.pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion(q)

        rpyGoal.pose.orientation.x = 0.0
        rpyGoal.pose.orientation.y = 0.0
        rpyGoal.pose.orientation.z = yaw
        rpyGoal.pose.orientation.w = 0.0
        self.goalPub.publish(rpyGoal)

    def handle_initial_pose(self, msg):
        print('initial pose clicked')
        rpyPose = PoseStamped()
        rpyPose.header.frame_id = "map"
        rpyPose.header.stamp = msg.header.stamp
        rpyPose.pose.position.x = msg.pose.pose.position.x
        rpyPose.pose.position.y = msg.pose.pose.position.y
        rpyPose.pose.position.z = 0.0

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = msg.pose.pose.orientation.z
        q.w = msg.pose.pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion(q)

        rpyPose.pose.orientation.x = 0.0
        rpyPose.pose.orientation.y = 0.0
        rpyPose.pose.orientation.z = yaw
        rpyPose.pose.orientation.w = 0.0
        self.initPub.publish(rpyPose)


def main(args=None):
    rclpy.init(args=args)
    click2dNode =  Rviz2Click2To2d()

    rclpy.spin(click2dNode)
    click2dNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
