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

import os
import select
import sys
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node

from geometry_msgs.msg import Twist  # linear speed,angle speed msg type (for x,y,z)
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


MAX_LIN_VEL = 0.2
MAX_ANG_VEL = 0.8
LIN_VEL_STEP_SIZE = 0.03
ANG_VEL_STEP_SIZE = 0.1

MAX_SONG = 5
MAX_ANIM = 3
MAX_COLOR = 6

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

c: Change led
z: play buzzer song
p: OLED animation

CTRL-C to quit
"""

e = """
Communications Failed
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:  # if valid, save to key
        key = sys.stdin.read(1)
    else:       # else, initialize
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:  # if variance is bigger
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)
def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    print('Param max lin: %s m/s, max ang: %s rad/s, lin step: %s m/s ang step: %s rad/s'%
        (MAX_LIN_VEL, MAX_ANG_VEL,
        LIN_VEL_STEP_SIZE, ANG_VEL_STEP_SIZE)
    )

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard_node')        # generate node
    pub = node.create_publisher(Twist, 'cmd_vel', qos)      # generate publisher for 'cmd_vel'
    ledpub = node.create_publisher(Int32, 'ledSub',10)      # generate publisher for 'ledSub'
    songpub = node.create_publisher(Int32, 'songSub',10)    # generate publisher for 'songpub'
    lcdpub = node.create_publisher(Int32, 'lcdSub',10)      # generate publisher for 'lcdpub'

    print('Monicar Teleop Keyboard controller')

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    colorIdx = 0                                        # variable for saving data in ledSub's msg data field
    songIdx = 0                                         # variable for saving data in songSub's msg data field
    lcdIdx = 0
    gMsg = Int32()

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':              # linear speed up
                target_linear_velocity = \
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':            # linear speed down
                target_linear_velocity = \
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':            # left angle spped up
                target_angular_velocity = \
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':            # right angle spped up
                target_angular_velocity = \
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':  # pause
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'c':                # led control
                print('colorIdx: %d'%(colorIdx))
                gMsg.data = colorIdx
                ledpub.publish(gMsg)
                colorIdx += 1
                if colorIdx >= MAX_COLOR:
                    colorIdx = 0

            elif key == 'z':                # play buzzer song
                print('songIdx: %d'%(songIdx))
                gMsg.data = songIdx
                songpub.publish(gMsg)
                songIdx += 1
                if songIdx >= MAX_SONG:
                    songIdx = 0

            elif key == 'p':                # play oled animation
                print('lcdIdx: %d'%(lcdIdx))
                gMsg.data = lcdIdx
                lcdpub.publish(gMsg)
                lcdIdx += 1
                if lcdIdx >= MAX_ANIM:
                    lcdIdx = 0

            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()         #generate variable for Twist type msg

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)      #publishing 'cmd_vel'

    except Exception as e:
        print(e)

    finally:  #
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
