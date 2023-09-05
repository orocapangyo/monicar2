#!/usr/bin/env python3

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist # 선형속도,각속도값 msg(x,y,z로 주어짐)
from rclpy.qos import QoSProfile

if os.name == 'nt': # window 사용시 키입력 모듈
    import msvcrt
else:
    import termios #linux 사용시 키입력 모듈
    import tty

# 기존 jessicar2의 setting값 참조
MAX_LIN_VEL = 1.20
MAX_ANG_VEL = 1.80
LIN_VEL_STEP_SIZE = 0.05 
ANG_VEL_STEP_SIZE = 0.1 

# """은 여러줄의 문자열을 변수에 할당하기 위한 문법
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

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt': 
        return msvcrt.getch().decode('utf-8') 
    tty.setraw(sys.stdin.fileno()) #Terminal 설정을 raw상태로 돌려 키 입력을 즉각 받음
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) #입력값 감시
    if rlist: #유효하면 key에저장
        key = sys.stdin.read(1)
    else: #아니면 초기화
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) #터미널 설정 복구
    return key


def print_vels(target_linear_velocity, target_angular_velocity): #선형,각 속도 출력
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

#제어값 프로파일링(주행을 부드럽게)
def make_simple_profile(output, input, slop): # slop
    if input > output: #변화가 작으면(목표값 미달)
        output = min(input, output + slop) #slop 더해서 부드럽게
    elif input < output: #변화가 크면(목표값 초과)
        output = max(input, output - slop) #slop 뺴서 부드럽게
    else:
        output = input 

    return output


def constrain(input_vel, low_bound, high_bound): #속도,각도(회전) 바운더리 
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
        
    rclpy.init() #rclpy 초기화
    
    qos = QoSProfile(depth=10) #msg 버퍼크기 제한
    node = rclpy.create_node('teleop_keyboard_node') # node생성
    pub = node.create_publisher(Twist, 'cmd_vel', qos)# publisher 생성, cmd_vel 발행

    print('Monicar Teleop Keyboard controller')
    
    
    
    # 주행 알고리즘 변수 초기화. status는 플래그로 사용, 나머지 변수들은 속도,방향 제어로 사용. 
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w': # 선형 속도 증가 
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x': # 선형 속도 감소
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a': # 왼쪽 각속도 증가
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd': #오른쪽 각속도 증가
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's': # 정지(초기화)
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg) # 터미널에 키 조작법 정보가 주행 상태 정보에 묻히지 않기 위해서 20번 키입력때마다 주행 설명서 출력
                status = 0

            twist = Twist() #변수 twist를 Twist 객체로 생성

            control_linear_velocity = make_simple_profile( #선형속도 프로파일링
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0)) #slop(보간)값

            twist.linear.x = control_linear_velocity # 선형속도, x성분만 사용
            twist.linear.y = 0.0 
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(#각속도 프로파일링
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0)) #slop(보간)값

            twist.angular.x = 0.0 #각속도, z성분만 사용
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity 

            pub.publish(twist) #cmd vel에 twist 정보를 담아서 publish

    except Exception as e: #예외처리
        print(e)

    finally: # 프로그램 종료전에 monicar가 수행해야할 작업들(움직임 정지, publish)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # 터미널 설정 복구(linux)


if __name__ == '__main__':
    main()
