#!/usr/bin/env python

from tokenize import Double
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile

class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node') # node 초기화
        self.declare_parameters(#parameter 선언
            namespace='',
            parameters=[
                ('max_fwd_m_s', None),
                ('max_rev_m_s', None),
                ('max_deg_s', None), 
            ])
        self.timer_inc = 0
        self.auto_mode = False
        self.headlight_on = False
        self.mode_button_last = 0

        print(' Monicar Teleop Joystick controller')
       
       # 최대 전진, 후진 회적 속도를 yaml파일에서 config된 parameter을 사용
        self.max_fwd_vel = self.get_parameter_or('max_fwd_m_s', Parameter('max_fwd_m_s', Parameter.Type.DOUBLE, 0.2)).get_parameter_value().double_value
        
        self.max_rev_vel = self.get_parameter_or('max_rev_m_s', Parameter('max_rev_m_s', Parameter.Type.DOUBLE, 0.2)).get_parameter_value().double_value
        
        self.max_ang_vel = self.get_parameter_or('max_deg_s', Parameter('max_deg_s', Parameter.Type.DOUBLE, 0.2)).get_parameter_value().double_value
        
        print('Param max fwd: %s m/s, max rev: -%s m/s, max ang: %s dev/s'%
            (self.max_fwd_vel,
            self.max_rev_vel,
            self.max_ang_vel)
        )
        self.qos = QoSProfile(depth=10) #msg 크기 10으로 제한
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', self.qos) #cmd_vel토픽에 Twist msg를 발행하는 publisher
        self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, 10) #joy msg를 구독하는 subscriber를 생성.
        self.timer = self.create_timer(0.05, self.cb_timer)
        self.twist = Twist()

    def cb_joy(self, joymsg):
        if joymsg.buttons[2] == 1 and self.mode_button_last == 0:
            if self.auto_mode == False:
                self.auto_mode = True
                self.get_logger().info("AUTO MODE ON")
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
        if joymsg.buttons[0] == 1:
            if self.headlight_on == False:
                self.headlight_on = True
                #self.set_headlight_onOff(True)
            else:
                self.headlight_on = False
                #self.set_headlight_onOff(False)

        self.twist.linear.y = 0.0; 
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0; 
        self.twist.angular.y = 0.0; 
        self.twist.angular.z = joymsg.axes[0] * self.max_ang_vel
        print('V= %.2f m/s, W= %.2f deg/s'%(self.twist.linear.x, self.twist.angular.z))

    def cb_timer(self):
        self.timer_inc+=1
        if self.auto_mode == False:
            self.pub_twist.publish(self.twist)

def main(args=None):
    rclpy.init(args=args) #ros2 node 초기화
    teleop_joy =  TeleopJoyNode() # node 시작
    rclpy.spin(teleop_joy) #node loop 시작
    teleop_joy.destroy_node() #node 종료
    rclpy.shutdown() #ros2종료

if __name__ == '__main__':
    main()
