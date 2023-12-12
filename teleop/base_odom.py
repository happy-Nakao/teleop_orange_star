#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#--------------------------------------------------------------
# Title: 足回り制御を行うPythonモジュール
# Author: Issei Iida, Yusuke Kanazawa
# Date: 2021/10/25
# Memo: [rotateAngle()]はOdometryを使っています
#--------------------------------------------------------------

import rclpy
import math
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node


class BaseControl(Node):
    def __init__(self):
        # Subscriber
        super().__init__('base_odom')
        self.create_subscription(Odometry, '/vmegarover/odom', self.odomCB, qos_profile=10)
        # Value
        #self.quaternion = (0.0, 0.0, 0.0, 0.0)   # クォータニオン (x, y, z, w)
        #self.current_euler = []   # オイラー角 [roll: x, Pitch: Y, Yaw: z]
        #self.current_deg = 0.0   # 現在の角度（度数法）
        
    def odomCB(self, receive_msg):
        x = receive_msg.pose.pose.orientation.x                                                                                          
        y = receive_msg.pose.pose.orientation.y                                                                                          
        z = receive_msg.pose.pose.orientation.z                                                                                          
        w = receive_msg.pose.pose.orientation.w                                                                                          
                                                                                                                  
        sinr_cosp = 2 * (w * x + y * z)                                                                           
        cosr_cosp = 1 - 2 * (x * x + y * y)                                                                       
        roll      = np.arctan2(sinr_cosp, cosr_cosp)                                                              
                                                                                                                  
        sinp  = 2 * (w * y - z * x)                                                                               
        pitch = np.arcsin(sinp)                                                                                   
                                                                                                                  
        siny_cosp = 2 * (w * z + x * y)                                                                           
        cosy_cosp = 1 - 2 * (y * y + z * z)                                                                       
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.current_euler = [roll, pitch, yaw]     
        #print(self.current_euler)
        self.current_deg = math.degrees(self.current_euler[2])
        if self.current_deg < 0.0:
            sub_deg = 180 - abs(self.current_deg)
            self.current_deg = 180 + sub_deg
        else:
            pass
        print(self.current_deg)


def main():
    rclpy.init()
    base_control = BaseControl()
    rclpy.spin(base_control)