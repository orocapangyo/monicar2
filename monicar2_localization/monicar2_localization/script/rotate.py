#!/usr/bin/env python

#thread for seep
#https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
#time calculation
#https://answers.ros.org/question/383841/ros2-python-odometry/

import rclpy
import time
from geometry_msgs.msg import Twist
import threading

PI = 3.1415926535897

def main():

    rclpy.init()
    node = rclpy.create_node('rotate_node')
    velocity_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    vel_msg = Twist()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = float(input("Input your speed (degrees/sec):"))
    angle = float(input("Type your distance (degrees):"))
    clockwise = int(input("Clockwise, 0 or 1:"))

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #Since we are moving just in x-axis
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = abs(angular_speed)
    else:
        vel_msg.angular.z = -abs(angular_speed)

    #Setting the current time for distance calculus
    t0 = node.get_clock().now().to_msg()
    lt = t0.sec + (t0.nanosec/1e+9)
    current_angle = 0.0

    print("Start to rotate")
    try:
        while rclpy.ok():
            rate.sleep()
            #Publish the velocity
            velocity_publisher.publish(vel_msg)

            t1 = node.get_clock().now().to_msg()
            ct = t1.sec + (t1.nanosec/1e+9)

            #Calculates angle
            current_angle= angular_speed*(ct-lt)
            print("ct:", ct-lt)
            if (current_angle > relative_angle):
                break
    except KeyboardInterrupt:
        pass

    print("Stop to move")
    #After the loop, stops the robot
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    main()