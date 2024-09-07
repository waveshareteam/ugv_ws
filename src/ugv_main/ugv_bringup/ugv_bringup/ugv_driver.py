#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial  
import json  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32,Float32MultiArray
import subprocess
import time

# 打开串口  
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)  

class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        # 创建订阅者
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState,'ugv/joint_states',self.joint_states_callback,10)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray,'ugv/led_ctrl',self.led_ctrl_callback,10)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)
        
    def cmd_vel_callback(self, msg):
        # 收到速度消息时发送速度数据给串口
#        print("msg.linear.x: ", msg.linear.x)
#        print("msg.angular.z: ", msg.angular.z)
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        # 防止原地转向时速度太小无法转动
        if linear_velocity == 0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2
        data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity}) + "\n"
        ser.write(data.encode())
        
    def joint_states_callback(self, msg):

        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }
        
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180*x_rad)/3.1415926
        y_degree = (180*y_rad)/3.1415926
        
        joint_data = json.dumps({
            'T': 134, 
            'X': x_degree, 
            'Y': y_degree, 
            "SX":600,
            "SY":600,
        }) + "\n"
                
        ser.write(joint_data.encode())

    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        
        led_ctrl_data = json.dumps({
            'T': 132, 
            "IO4":IO4,
            "IO5":IO5,
        }) + "\n"
                
        ser.write(led_ctrl_data.encode())
        
    def voltage_callback(self, msg):
        voltage_value = msg.data
        if 0.1< voltage_value < 9 : 
            subprocess.run(['aplay', '-D', 'plughw:4,0', '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
            time.sleep(5)
            #print("Low battery - Voltage is below 9: ", voltage_value)
        #else:
            #print("Received voltage data:", voltage_value)
       
def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = UgvDriver("ugv_driver")  # 新建一个节点

    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    # 关闭rclpy
    rclpy.shutdown()
    # 关闭串口  
    ser.close()

if __name__ == '__main__':
    main()
