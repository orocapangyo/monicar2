#!/usr/bin/env python
# Author: ChangeWhan Lee

#Port below ROS1 to ROS2
#https://answers.ros.org/question/326434/how-to-compute-odometry-from-encoder-ticks/

#Refers below tutorials
#https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

import math
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped, Point, Pose, Quaternion, Twist, Vector3

from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

#Parameters
wheeltrack = 0.165
wheelradius = 0.033
TPR = 1860.0

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class ODOMNode(Node):

    def __init__(self):
        super().__init__('odompub_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('initialPose', None),
            ])

        print('Odom publisher created')

        # Get parameter values
        self.initialPose = self.get_parameter_or('initialPose', Parameter('initialPose', Parameter.Type.INTEGER, 1)).get_parameter_value().integer_value
        print('initialPose: %d' %(self.initialPose) )

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx =  0.0
        self.vy =  0.0
        self.vth =  0.0

        self.qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', self.qos)
        self.left_ticks_sub = self.create_subscription(Int32, 'left_ticks', self.leftTicksCallback, 10)
        self.right_ticks_sub = self.create_subscription(Int32, 'right_ticks', self.rightTicksCallback, 10)
        
        self.timer_tick = 0.05
        self.timer = self.create_timer(self.timer_tick, self.cb_timer)

         # Initialize the transform broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)
        # Mark current time
        self.last_time = self.get_clock().now()
        self.current_time = self.last_time
        print('Init done')

    def leftTicksCallback(msg):
        self.left_ticks = msg.data
        #print('leftTicks: %d' %(self.left_ticks) )

    def rightTicksCallback(msg):
        self.right_ticks = msg.data
        #print('rightTicks: %d' %(self.right_ticks) )

    def cb_timer(self):

        self.current_time = self.get_clock().now()

        delta_L = self.left_ticks - self.last_left_ticks
        delta_R = self.right_ticks - self.last_right_ticks

        dl = 2 * pi * wheelradius * delta_L / TPR
        dr = 2 * pi * wheelradius * delta_R / TPR
        dc = (dl + dr) / 2
        #dt = (self.current_time - self.last_time)
        dt = self.timer_tick
        dth = (dr-dl)/wheeltrack

        if dr==dl:
            dx=dr*cos(self.th)
            dy=dr*sin(self.th)

        else:
            radius=dc/dth

            iccX=x-radius*sin(self.th)
            iccY=y+radius*cos(self.th)

            dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
            dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

        self.x += dx  
        self.y += dy 
        self.th =(self.th+dth) %  (2 * pi)
        
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'odom'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        odom_quat  = quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        # Send the transformation
        self.odom_broadcaster.sendTransform(t)

        # next, we'll publish the odometry message over ROS
        if dt>0:
            vx=dx/dt
            vy=dy/dt
            vth=dth/dt

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.current_time.to_msg()

        # set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    odomNode =  ODOMNode()

    rclpy.spin(odomNode)
    odomNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
