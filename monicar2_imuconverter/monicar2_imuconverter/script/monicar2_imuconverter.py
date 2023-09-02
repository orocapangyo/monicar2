#!/usr/bin/env python
# Author: ChangeWhan Lee
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

import time
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('monicar2_imu_node')
        # Declare parameters from YAML
        self.get_logger().info("Setting Up the Node...")

        # Set subscriber
        self.quatSub = self.create_subscription(Quaternion, 'quaternion', self.node_callback, 10)       
        self.get_logger().info("quaternion subscriber set") 
        
        # Set publisher
        self.imuPub = self.create_publisher(Imu, 'imu/data', 10)
        self.get_logger().info("Imu publisher set")

        # Set publisher
        self.posePub = self.create_publisher(PoseStamped, 'pose', 10)
        self.get_logger().info("PoseStamped publisher set")

        self.imu_msg = Imu()
        self.pose_msg = PoseStamped()

        self.frame_id = self.declare_parameter('frame_id', "imu_link").value
        self.seq = 0

        linear_acceleration_stdev = 0.06
        angular_velocity_stdev = 0.005
        linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev
        angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev

        # Orientation covariance estimation:
        # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
        # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
        # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
        # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
        # i.e. variance in yaw: 0.0025
        # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
        # static roll/pitch error of 0.8%, owing to gravity orientation sensing
        # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
        # so set all covariances the same.
        self.imu_msg.orientation_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        self.imu_msg.angular_velocity_covariance = [
            0.0, 0.0, 0.0,
            0.0, angular_velocity_cov, 0.0,
            0.0, 0.0, angular_velocity_cov
        ]

        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        self.imu_msg.linear_acceleration_covariance = [
            0.0, 0.0, 0.0,
            0.0, linear_acceleration_cov, 0.0,
            0.0, 0.0, linear_acceleration_cov
        ]

    def node_callback(self, msg):
        #self.get_logger().info("Received a /quaternion message!")
        #self.get_logger().info("Components: [%0.2f, %0.2f]"%(msg.linear.x, msg.angular.z))
        self.imu_msg.header.frame_id = self.frame_id
        self.pose_msg.header.frame_id = self.frame_id
        
        self.imu_msg.orientation = msg
        self.pose_msg.pose.orientation = msg

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()

        self.seq = self.seq + 1

        self.imuPub.publish(self.imu_msg)
        self.posePub.publish(self.pose_msg)

def main(args=None):
    rclpy.init(args=args)
    imuNode = ImuNode()
    rclpy.spin(imuNode)

    imuNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()