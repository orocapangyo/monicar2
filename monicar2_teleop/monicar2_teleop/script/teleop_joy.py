#!/usr/bin/env python3

#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from tokenize import Double
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile

MAXCOLOR = 5

msg = """
Control Your Robot!
---------------------------
'A' : Change led

once click : left
double clicks : right
three clicks : right & center
four clicks : All ON
five clicks : ALL OF
"""

class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node')
        self.declare_parameters(    # bring the param from yaml file
            namespace='',
            parameters=[ 
                ('max_fwd_m_s', None),
                ('max_rev_m_s', None),
                ('max_deg_s', None),
            ])

        self.timer_inc = 0
        self.auto_mode = False
        self.colorIdx = 0
        self.headlight_on = False
        self.mode_button_last =0
        self.colorIdx = 0           # variable for saving data in ledSub's msg data field
        self.pub_led = self.create_publisher(Int32, 'ledSub',10) 
        print(' Monicar Teleop Joystick controller')
        print(msg)       
      
        self.max_fwd_vel = self.get_parameter_or('max_fwd_m_s', Parameter('max_fwd_m_s', Parameter.Type.DOUBLE, 0.2)).get_parameter_value().double_value
        
        self.max_rev_vel = self.get_parameter_or('max_rev_m_s', Parameter('max_rev_m_s', Parameter.Type.DOUBLE, 0.2)).get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter_or('max_deg_s', Parameter('max_deg_s', Parameter.Type.DOUBLE, 0.2)).get_parameter_value().double_value

        print('Param max fwd: %s m/s, max rev: -%s m/s, max ang: %s dev/s'%
            (self.max_fwd_vel,
            self.max_rev_vel,
            self.max_ang_vel)

        )
        print('CTRL-C to quit')

        self.qos = QoSProfile(depth=10)
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', self.qos) 
        # generate publisher for 'cmd_vel'
        self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, 10) 
        # generate publisher for 'ledSub
        self.timer = self.create_timer(0.05, self.cb_timer)
        self.twist = Twist()  
        #generate variable for Twist type msg

    def cb_joy(self, joymsg):
        if joymsg.buttons[2] == 1 and self.mode_button_last == 0:
            if self.auto_mode == False:
                self.auto_mode = True
                self.get_logger().info("AUTO MODE")
            else:
                self.auto_mode = False
                self.get_logger().info("AUTO MODE OFF")
        self.mode_button_last = joymsg.buttons[2]

        if joymsg.axes[1] > 0.0:
            self.twist.linear.x = joymsg.axes[1] * self.max_fwd_vel
        elif joymsg.axes[1] < 0.0:
            self.twist.linear.x = joymsg.axes[1] * self.max_rev_vel
        else:
            self.twist.linear.x = 0.0
        if joymsg.buttons[0] == 1 and self.mode_button_last == 0:
            self.colorIdx+=1
            if self.colorIdx>MAXCOLOR: 
                self.colorIdx=0

        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = joymsg.axes[0] * self.max_ang_vel
        print('V= %.2f m/s, W= %.2f deg/s'%(self.twist.linear.x, self.twist.angular.z))

    def cb_timer(self):
        self.timer_inc+=1
        if self.auto_mode == False:
            self.pub_twist.publish(self.twist)  #publishing 'cmd_vel'

        led_msg = Int32()                       #generate variable for Int32 type msg
        led_msg.data = self.colorIdx 
        self.pub_led.publish(led_msg)           #publishing 'ledSub'

def main(args=None):
    rclpy.init(args=args) 
    teleop_joy =  TeleopJoyNode() 
    rclpy.spin(teleop_joy) 
    teleop_joy.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
