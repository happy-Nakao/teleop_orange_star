#!/usr/bin/env python3
#-*- coding:utf-8 -*-


import rclpy
import time
import math
import numpy
import threading
import matplotlib.pyplot as plot 
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile


class BaseControl(Node):
    def __init__(self):
        super().__init__('base_teleop')
        self.rate = self.create_rate(10, self.get_clock())
        # qos_profile = rclpy.qos.QoSProfile(
        #     reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        #     history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        #qos_profile=qos_profile
        self.twist_pub = self.create_publisher(Twist, '/vmegarover/cmd_vel', 10)
        # Subscriber
        self.create_subscription(Odometry, '/vmegarover/odom', self.odomCB, qos_profile=10)
        # Value
        self.twist_value = Twist()
        self.target_time = 0.0
        #self.rate = rclpy.Rate(1.0)
        self.anglar_vel = 0.0  # 角速度
        self.quaternion = (0.0, 0.0, 0.0, 0.0)   # クォータニオン (x, y, z, w)
        self.current_euler = []   # オイラー角 [roll: x, Pitch: Y, Yaw: z]
        self.current_deg = 0.0   # 現在の角度（度数法）
        self.target_deg = 0.0   # 目標の角度（度数法）
        self.remain_deg = 0.0
        self.sub_target_deg = 0.0
        self.judg_deg = 999.0
        self.tf_buffer = tf2_ros.Buffer()
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def debag(self):
        self.create_subscription(Float64, '/teleop/translate', self.translateDist,  qos_profile=10)
        self.create_subscription(Float64, '/teleop/rotate', self.rotateAngle,  qos_profile=10)

    def odomCB(self, receive_msg):
        #transform_stamped = self.tf_buffer.lookup_transform("target_frame", "source_frame", rclpy.Time(0), rclpy.Duration(1.0))
        self.anglar_vel = receive_msg.twist.twist.angular.z
        # self.quaternion = (
        #     receive_msg.pose.pose.orientation.x,
        #     receive_msg.pose.pose.orientation.y,
        #     receive_msg.pose.pose.orientation.z,
        #     receive_msg.pose.pose.orientation.w)
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
        #quaternion = transform_stamped.transform.rotation
        #print(self.quaternion)
        #self.current_euler = self.tf_buffer.transformations.euler_from_quaternion(self.quaternion)
        #print(self.current_euler)
        self.current_deg = math.degrees(self.current_euler[2])
        if self.current_deg < 0.0:
            sub_deg = 180 - abs(self.current_deg)
            self.current_deg = 180 + sub_deg
        else:
            pass
        #print(self.current_deg)

    def roundDown(self, value, down_num=1):
        return math.floor(value * (10**down_num)) / (10**down_num)

    def publishLinerX(self):
        start_time = time.time()
        end_time = time.time() + 0.15 # 誤差をカバーする0.15
        while end_time - start_time <= self.target_time:
            #print(end_time - start_time)
            self.twist_pub.publish(self.twist_value)
            #print(self.twist_value)
            end_time = time.time()
            time.sleep(2.0)
        self.twist_value.linear.x = 0.0
        self.twist_pub.publish(self.twist_value)
        print("========== FINISH translateDist ==========")

    def publishAnglarZ(self, max_speed, precision, time_out):
        over_flg = False
        time_list = []
        deg_list = []
        integral_value = 0.0
        vel_z = 0.0
        vel_max = max_speed
        self.judg_deg = 999.0
        kp = 0.11  # 0.11
        ki = 0.022  # 0.022
        kd = 1.0  # 1.0
        print("rotateAngle")
        start_time = time.time()
        start_plot = time.time()
        while self.roundDown(self.judg_deg, precision) != self.roundDown(self.current_deg, precision) and not rclpy.shutdown():
            plot_time = time.time() - start_plot
            delta_time = time.time() - start_time
            # 0度をまたがないとき
            if self.sub_target_deg == 0.0:
                self.judg_deg = self.target_deg
                if not over_flg and self.target_deg < 20 and self.current_deg > 260:
                    over_flg = True
                    vel_z = 0.3
                    integral_value = 0.0
                elif not over_flg and self.target_deg > 340 and self.current_deg < 100:
                    over_flg = True
                    vel_z = -0.3
                    integral_value = 0.0
                elif over_flg and self.target_deg < 20 and self.current_deg < 20:
                    over_flg = False
                elif over_flg and self.target_deg > 340 and self.current_deg > 340:
                    over_flg = False
                else:
                    pass
                if abs(self.judg_deg - self.current_deg) < 5:
                    integral_value += delta_time*(self.target_deg - self.current_deg)
                else:
                    integral_value = 0
                if not over_flg:
                    vel_z = kp*(self.target_deg - self.current_deg) + ki*integral_value - kd*self.anglar_vel
                else:
                    pass
            # 0度を時計回りにまたぐとき
            elif self.sub_target_deg > 180:
                self.judg_deg = self.sub_target_deg
                # 終点が0度通過直後(10度以内)かつフィードバック値が20度未満のとき
                if abs(self.judg_deg - self.current_deg) < 5:
                    integral_value += delta_time*(self.sub_target_deg - self.current_deg)
                else:
                    integral_value = 0
                if self.current_deg > 180:
                    over_flg = True
                if abs(360 - self.sub_target_deg) < 10.0 and 20 - self.current_deg > 0:
                    vel_max = 0.3
                elif self.current_deg < 180 and not over_flg:  # 0度より左側のとき
                    vel_z = -vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg) + ki*integral_value - kd*self.anglar_vel
            # 0度を反時計回りにまたぐとき
            else:
                self.judg_deg = self.sub_target_deg
                # 終点が0度通過直後のときかつフィードバック値が340度以上のとき
                if abs(self.judg_deg - self.current_deg) < 5:
                    integral_value += delta_time*(self.sub_target_deg - self.current_deg)
                else:
                    integral_value = 0
                if self.current_deg < 180:
                    over_flg = True
                if abs(0 - self.sub_target_deg) < 10.0 and 360 - self.current_deg < 20:
                    vel_max = 0.3
                elif self.current_deg > 180 and not over_flg:  # 0度より右側のとき
                    vel_z = vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg) + ki*integral_value - kd*self.anglar_vel
            time.sleep(0.05)
            if plot_time > time_out:
                print("Time out !!!")
                break
            if abs(vel_z) > vel_max:
                vel_z = numpy.sign(vel_z)*vel_max
            self.twist_value.angular.z = vel_z
            self.twist_pub.publisher_.publish(self.twist_value)
            # グラフプロット用リスト
            time_list.append(plot_time)
            deg_list.append(self.current_deg)
            start_time = time.time()
            time.sleep(0.05)
        #print(round(self.judg_deg, 1), round(self.current_deg, 1))
        self.twist_value.angular.z = 0.0
        self.twist_pub.publish(self.twist_value)
        self.sub_target_deg = 0.0
        print(f"Finish deg: {self.roundDown(self.current_deg, 1)}")
        print("========== FINISH rotateAngle ==========\n")
        return time_list, deg_list
        
    def translateDist(self, dist, speed = 0.2):
        try:
            dist = dist.data
        except AttributeError:
            pass
        speed = abs(speed)
        self.target_time = abs(dist / speed)
        self.twist_value.linear.x = dist/abs(dist)*speed
        #print(self.twist_value)
        print("\n========== START translateDist ==========")
        self.publishLinerX()

    def rotateAngle(self, deg, precision=0, speed=0.7, time_out=20):
        try:
            deg = deg.data
        except AttributeError:
            pass
        time.sleep(0.2)
        if deg >= 0.0:
            self.target_deg = self.current_deg + deg
            if self.target_deg >= 360:
                self.remain_deg = self.target_deg - 360
                self.sub_target_deg = self.remain_deg
            else:
                pass
        else:
            self.target_deg = self.current_deg + deg
            if self.target_deg < 0.0:
                self.remain_deg = 360 + self.target_deg
                self.sub_target_deg = self.remain_deg
            else:
                pass
        self.current_deg = self.roundDown(self.current_deg, precision)
        self.target_deg = self.roundDown(self.target_deg, precision)
        self.sub_target_deg = self.roundDown(self.sub_target_deg, precision)
        print("\n========== START rotateAngle ==========")
        print(f"current deg: {self.current_deg}")
        print(f"target deg: {self.target_deg}")
        print(f"sub_target deg: {self.sub_target_deg}")
        return self.publishAnglarZ(speed, precision, time_out)

    # ゲイン調整のときに使う
    def odomPlot(self, deg, precision=0, speed=0.5, time_out=10):
        time_x = []
        deg_y = []
        plot_time = 0.0
        # グラフの描画とrotateAngleを並列処理する
        rotate_thread = threading.Thread(target=self.rotateAngle, args=(deg, precision, speed, time_out))
        rotate_thread.start()
        time.sleep(0.05)
        
        start_time = time.time()
        while not rclpy.is_shutdown():
            if len(threading.enumerate()) - 8 == 0 and time_out < plot_time:
                break
            plot_time = time.time() - start_time
            # グラフデータ作成
            time_x.append(plot_time)
            deg_y.append(self.current_deg)
            # 軸ラベル
            plot.xlabel("Time [sec]")
            plot.ylabel("Degree [deg]")
            # プロット範囲
            plot.xlim(0, time_out)
            plot.ylim(min(deg_y), self.judg_deg + 10)
            # プロット
            plot.plot(time_x, deg_y, color='blue', label='Odom data')
            plot.hlines(self.judg_deg, 0, time_out, color='red', linestyles='dotted', label='Target')
            # 凡例表示
            plot.legend()
            # 0.1秒間隔で更新
            plot.pause(0.1)
            # 画面初期化
            plot.clf()

        print("All Plottted!")
        plot.show()

def main():
    rclpy.init()
    base_teleop = BaseControl()
    base_teleop.debag()
    rclpy.spin(base_teleop)
    rclpy.shutdown()
    
# if __name__ == '__main__':
#     base_control = BaseControl()
#     base_control.debag()
#     rclpy.spin()
#     #base_control.odomPlot(30, 1, 0.7, 10)  # ゲイン調整用
