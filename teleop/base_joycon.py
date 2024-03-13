#!/usr/bin/env python3
#-*- coding:utf-8 -*-

#--------------------------------------------------------------------
# Title: ジョイスティックで足回り制御をするROSノード
# Author: Issei Iida
# Date: 2021/06/17
# Memo: dualshock4 コントローラに対応したキー配置設定
#--------------------------------------------------------------------

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.node import Node

class JoyCtrMegarover(Node):
    def __init__(self):
        super().__init__('base_joycon')
        # Publisher
        #仮想環境で行う際は /vmegarover/cmd_vel にする
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Subscriber
        self.create_subscription(Joy, "joy", self.joyCB, qos_profile=10)
        # Value
        self.twist = Twist()
        self.linear = 1
        self.angular = 0
        #三角ボタンを押している間、スティックで動作
        self.safety = 2
        self.declare_parameter('l_scale', 0.6)
        self.declare_parameter('a_scale', 0.8)
        self.l_scale = self.get_parameter("l_scale").value
        self.a_scale = self.get_parameter("a_scale").value

    def joyCB(self, joy):
        if joy.buttons[self.safety]:
            self.twist.angular.z = self.a_scale * joy.axes[self.angular]
            self.twist.linear.x = self.l_scale * joy.axes[self.linear]
        else:
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
        self.vel_pub.publish(self.twist)

def main():
    rclpy.init()
    base_joycon = JoyCtrMegarover()
    rclpy.spin(base_joycon)
    rclpy.shutdown()
