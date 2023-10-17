#!/usr/bin/env python

#thread for seep
#https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
#time calculation
#https://answers.ros.org/question/383841/ros2-python-odometry/

import rclpy
import time
from geometry_msgs.msg import Twist
import threading

def main():

    rclpy.init() 
    node = rclpy.create_node('move_node')
    velocity_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    vel_msg = Twist()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    #Receiveing the user's input
    print("Let's move your robot")
    speed = float(input("Input your speed, 0.0~0.2(m/s): "))
    distance = float(input("Type your distance, 0~1(meter): "))
    isForward = int(input("Foward, 0 or 1: "))
    
    #Checking if the movement is forward or backwards
    if isForward == 1:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    #Since we are moving just in x-axis
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    #Setting the current time for distance calculus
    t0 = node.get_clock().now().to_msg()
    lt = t0.sec + (t0.nanosec/1e+9)
    current_distance = 0.0

    print("Start to move")
    try:
        while rclpy.ok():
            rate.sleep()
            #Publish the velocity
            velocity_publisher.publish(vel_msg)

            t1 = node.get_clock().now().to_msg()
            ct = t1.sec + (t1.nanosec/1e+9)

            #Calculates distance
            current_distance= speed*(ct-lt)
            print("ct:", ct-lt)
            if (current_distance > distance):
                break
    except KeyboardInterrupt:
        pass

    print("Stop to move")
    #After the loop, stops the robot
    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    main()