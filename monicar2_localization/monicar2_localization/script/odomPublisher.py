#!/usr/bin/env python
# Author: ChangeWhan Lee

#Port below ROS1 to ROS2
#https://answers.ros.org/question/326434/how-to-compute-odometry-from-encoder-ticks/
#https://answers.ros.org/question/383841/ros2-python-odometry/

#Refer below tutorials
#https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

# euler_from_quaternion, quaternion_from_euler
# https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434

import math
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped, Point, Pose, Quaternion, Twist, PoseStamped

from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

#Parameters
wheeltrack = 0.160
wheelradius = 0.033

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 

class ODOMNode(Node):

    def __init__(self):
        super().__init__('odompub_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('initPoseRecieved', True),
                ('TPR', 0.0),
                ('timer_tick', 0.0),
            ])

        print('Odom publisher created')

        # Get parameter values
        self.initPoseRecieved = self.get_parameter_or('initPoseRecieved', Parameter('initPoseRecieved', Parameter.Type.BOOL, True)).get_parameter_value().bool_value
        self.timer_tick = self.get_parameter_or('timer_tick', Parameter('timer_tick', Parameter.Type.DOUBLE, 0.05)).get_parameter_value().double_value        
        self.TPR = self.get_parameter_or('TPR', Parameter('TPR', Parameter.Type.DOUBLE, 1860.0)).get_parameter_value().double_value
        print('initPoseRecieved: %s, TPR: %s' %(self.initPoseRecieved, self.TPR) )

        #initialzie variable
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
        self.init_sub = self.create_subscription(PoseStamped, 'initial_2d', self.init2dCallback, 10)
        
        self.timer = self.create_timer(self.timer_tick , self.cb_timer)

         # Initialize the transform broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # Mark current time
        self.last_time = self.get_clock().now().to_msg()
        self.current_time = self.last_time
        print('odompub_node done')

    def leftTicksCallback(self, msg):
        self.left_ticks = msg.data
        #print('leftTicks: %d' %(self.left_ticks) )

    def rightTicksCallback(self, msg):
        self.right_ticks = msg.data
        #print('rightTicks: %d' %(self.right_ticks) )

    def init2dCallback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.orientation.z
        self.initPoseRecieved = True
        print("init2dCallback:", self.initPoseRecieved)

    def cb_timer(self):
        if self.initPoseRecieved == False:
                return 1
        
        # Dt calculate
        self.current_time = self.get_clock().now().to_msg()
        ct = self.current_time.sec + (self.current_time.nanosec/1e+9)
        lt = self.last_time.sec + (self.last_time.nanosec/1e+9)
        dt = (ct - lt)
        #Dth calculate
        delta_L = self.left_ticks - self.last_left_ticks
        delta_R = self.right_ticks - self.last_right_ticks
        dl = 2 * pi * wheelradius * delta_L / self.TPR
        dr = 2 * pi * wheelradius * delta_R / self.TPR
        dc = (dl + dr) / 2
        dth = (dr-dl)/wheeltrack

        #dx,dy calculate
        if dr==dl:
            dx=dr*cos(self.th)
            dy=dr*sin(self.th)

        else:
            radius=dc/dth

            iccX=self.x-radius*sin(self.th)
            iccY=self.y+radius*cos(self.th)

            dx = cos(dth) * (self.x-iccX) - sin(dth) * (self.y-iccY) + iccX - self.x
            dy = sin(dth) * (self.x-iccX) + cos(dt) * (self.y-iccY) + iccY - self.y
        
        #x,y,th refresh
        self.x += dx  
        self.y += dy 
        self.th =(self.th+dth) %  (2 * pi)
        
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        odom_quat  = quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = odom_quat.x
        t.transform.rotation.y = odom_quat.y
        t.transform.rotation.z = odom_quat.z
        t.transform.rotation.w = odom_quat.w

        # Send the transformation
        self.odom_broadcaster.sendTransform(t)

        # next, we'll publish the odometry message over ROS
        if dt>0:
            vx=dx/dt
            vy=dy/dt
            vth=dth/dt

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()

        # set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat.x
        odom.pose.pose.orientation.y = odom_quat.y
        odom.pose.pose.orientation.z = odom_quat.z
        odom.pose.pose.orientation.w = odom_quat.w

        # set the velocity
        odom.child_frame_id = "base_footprint"
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
