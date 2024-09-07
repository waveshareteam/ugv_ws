import serial
import json
import queue
import threading
import rclpy
from rclpy.node import Node
import logging
import time
from std_msgs.msg import Header,Float32MultiArray,Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField
import math

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    def clear_buffer(self):
        self.s.reset_input_buffer()

class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger('BaseController')
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)
        self.rl = ReadLine(self.ser)
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}
    def feedback_data(self):
        try:
            line = self.rl.readline().decode('utf-8')
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            return self.base_data
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()

    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))
        return data_read

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField,"imu/mag",100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)  
        self.base_controller = BaseController('/dev/ttyAMA0', 115200)
        self.feedback_timer = self.create_timer(0.001, self.feedback_loop)
#        self.get_logger().info("ugv_bringup initialized and timer started.")

    def feedback_loop(self):
        self.base_controller.feedback_data()
        if self.base_controller.base_data["T"] == 1001:
           # print(self.base_controller.base_data)
            self.publish_imu_data_raw()
            self.publish_imu_mag()
            self.publish_odom_raw()
            self.publish_voltage()
#            self.get_logger().info("Feedback loop executed and data published.")

    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        msg.linear_acceleration.x = 9.8*float(imu_raw_data["ax"])/8192
        msg.linear_acceleration.y = 9.8*float(imu_raw_data["ay"])/8192
        msg.linear_acceleration.z = 9.8*float(imu_raw_data["az"])/8192
        
        msg.angular_velocity.x = 3.1415926*float(imu_raw_data["gx"])/(16.4*180)
        msg.angular_velocity.y = 3.1415926*float(imu_raw_data["gy"])/(16.4*180)
        msg.angular_velocity.z = 3.1415926*float(imu_raw_data["gz"])/(16.4*180)
              
        self.imu_data_raw_publisher_.publish(msg)
        
    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        msg.magnetic_field.x = float(imu_raw_data["mx"])*0.15
        msg.magnetic_field.y = float(imu_raw_data["my"])*0.15
        msg.magnetic_field.z = float(imu_raw_data["mz"])*0.15
              
        self.imu_mag_publisher_.publish(msg)     

    def publish_odom_raw(self):
        odom_raw_data = self.base_controller.base_data
        array = [odom_raw_data["odl"],odom_raw_data["odr"]]
        msg = Float32MultiArray(data = array)
        self.odom_publisher_.publish(msg)

    def publish_voltage(self):
        voltage_data = self.base_controller.base_data
        msg = Float32()
        msg.data = float(voltage_data["v"])
        self.voltage_publisher_.publish(msg)
                        
def main(args=None):
    rclpy.init(args=args)
    node = ugv_bringup()
    rclpy.spin(node)
    #node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
