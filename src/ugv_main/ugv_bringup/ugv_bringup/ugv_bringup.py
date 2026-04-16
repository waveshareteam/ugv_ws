import serial
import json
import queue
import threading
import subprocess
import time
import signal
import rclpy
from rclpy.node import Node
import logging
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
import math
import os

def is_jetson():
    """Detect Jetson by reading the device-tree model string — no filesystem walk."""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            return 'jetson' in f.read().lower()
    except OSError:
        return False

if is_jetson():
    serial_port = '/dev/ttyTHS1'
else:
    serial_port = '/dev/ttyAMA0'

# Helper class for reading lines from a serial port
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()  # Buffer to store incoming data
        self.s = s  # Serial object

    # Read a line of data from the serial input
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))  # Read from serial buffer
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    # Clear the buffer
    def clear_buffer(self):
        self.s.reset_input_buffer()

# Base controller class for managing UART communication and processing commands
class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger('BaseController')  # Logger setup
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)  # Open serial connection
        self.rl = ReadLine(self.ser)  # Initialize ReadLine helper
        self.command_queue = queue.Queue()  # Command queue for sending data
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)  # Start a separate thread for processing commands
        self.command_thread.start()
        self.data_buffer = None  # Buffer for holding received data
        # Base data structure to hold sensor values
        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}
    
    # Function to read and return feedback data from the serial input.
    # Non-blocking: returns the last known base_data immediately if no bytes
    # are waiting.  This prevents the ROS timer callback from stalling the
    # executor when the ESP32 is slow or silent.
    def feedback_data(self):
        if self.rl.s.in_waiting == 0:
            return self.base_data
        line = None
        try:
            # Drain any backlog so we don't fall behind, keeping only the latest packet
            while self.rl.s.in_waiting > 0:
                candidate = json.loads(self.rl.readline().decode('utf-8'))
                if 'T' in candidate:
                    self.base_data = candidate
            self.rl.clear_buffer()
            return self.base_data
        except UnicodeDecodeError:
            # UART framing/noise error — discard corrupted bytes and resync
            self.rl.clear_buffer()
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()

    # Receive and decode data from the serial connection
    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))  # Read and parse JSON data
        return data_read

    # Add a command to the queue to be sent via UART
    def send_command(self, data):
        self.command_queue.put(data)

    # Thread function to process and send commands from the queue
    def process_commands(self):
        while True:
            data = self.command_queue.get()  # Get command from the queue
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))  # Send command as JSON over UART

    # Send control data as JSON via UART
    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

    # Write motor-stop and feedback-disable directly to the serial port,
    # bypassing the command queue.  Safe to call from signal handlers and
    # destroy_node() where daemon threads may already be dying.
    def emergency_stop(self):
        try:
            self.ser.write((json.dumps({'T': 13, 'X': 0.0, 'Z': 0.0}) + '\n').encode('utf-8'))
            self.ser.write((json.dumps({'T': 131, 'cmd': 0}) + '\n').encode('utf-8'))
            self.ser.flush()
        except Exception:
            pass

# ROS node class for bringing up the UGV system and publishing sensor data
class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        # Publishers for IMU data, magnetic field data, odometry, and voltage
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)
        # Initialize the base controller with the UART port and baud rate
        self.base_controller = BaseController(serial_port, 115200)
        # Configure ESP32: enable serial feedback flow, set 50ms interval (20 Hz),
        # disable command echo, and select module type
        self.base_controller.base_json_ctrl({'T': 131, 'cmd': 1})
        self.base_controller.base_json_ctrl({'T': 142, 'cmd': 50})
        self.base_controller.base_json_ctrl({'T': 143, 'cmd': 0})   # echo off
        self.base_controller.base_json_ctrl({'T': 4, 'cmd': 0})     # module: 0=none
        # Home camera/gimbal to forward-facing default position
        self.base_controller.base_json_ctrl({'T': 133, 'X': 0, 'Y': 0, 'SPD': 200, 'ACC': 10})
        # Timer to periodically execute the feedback loop
        self.feedback_timer = self.create_timer(0.05, self.feedback_loop)  # 20 Hz — matches ESP32 feedback interval
        # Subscribers for forwarding commands to the base over UART
        self.cmd_vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('ugv_bringup ready')
        self.joint_states_sub_ = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)
        self.led_ctrl_sub_ = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)
        self._low_battery_playing = False
        # Watchdog: track last cmd_vel so we can stop motors if commands stop arriving
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_cmd_vel_nonzero = False
        self._CMD_VEL_TIMEOUT = 0.5  # seconds

    # Main loop for reading sensor feedback and publishing it to ROS topics
    def feedback_loop(self):
        # Watchdog: stop motors if no cmd_vel has arrived within the timeout
        if self._last_cmd_vel_nonzero:
            age = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
            if age > self._CMD_VEL_TIMEOUT:
                self.base_controller.emergency_stop()
                self._last_cmd_vel_nonzero = False
                self.get_logger().warn('cmd_vel watchdog: no command received, motors stopped')

        self.base_controller.feedback_data()
        if self.base_controller.base_data["T"] == 1001:  # Check if the feedback type is correct
            self.publish_imu_data_raw()  # Publish IMU raw data
            self.publish_imu_mag()  # Publish magnetic field data
            self.publish_odom_raw()  # Publish odometry data
            self.publish_voltage()  # Publish voltage data

    # Publish IMU data to the ROS topic "imu/data_raw"
    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        # Populate the linear acceleration and angular velocity fields
        msg.linear_acceleration.x = 9.8 * float(imu_raw_data["ax"]) / 8192
        msg.linear_acceleration.y = 9.8 * float(imu_raw_data["ay"]) / 8192
        msg.linear_acceleration.z = 9.8 * float(imu_raw_data["az"]) / 8192
        
        msg.angular_velocity.x = 3.1415926 * float(imu_raw_data["gx"]) / (16.4 * 180)
        msg.angular_velocity.y = 3.1415926 * float(imu_raw_data["gy"]) / (16.4 * 180)
        msg.angular_velocity.z = 3.1415926 * float(imu_raw_data["gz"]) / (16.4 * 180)
              
        self.imu_data_raw_publisher_.publish(msg)  # Publish the IMU data
        
    # Publish magnetic field data to the ROS topic "imu/mag"
    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        # Populate the magnetic field data
        msg.magnetic_field.x = float(imu_raw_data["mx"]) * 0.15
        msg.magnetic_field.y = float(imu_raw_data["my"]) * 0.15
        msg.magnetic_field.z = float(imu_raw_data["mz"]) * 0.15
              
        self.imu_mag_publisher_.publish(msg)  # Publish the magnetic field data

    # Publish odometry data to the ROS topic "odom/odom_raw"
    def publish_odom_raw(self):
        odom_raw_data = self.base_controller.base_data
        array = [odom_raw_data["odl"]/100, odom_raw_data["odr"]/100]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)  # Publish the odometry data

    # Publish voltage data to the ROS topic "voltage"
    def publish_voltage(self):
        voltage_data = self.base_controller.base_data
        msg = Float32()
        msg.data = float(voltage_data["v"])/100
        self.voltage_publisher_.publish(msg)  # Publish the voltage data
        if 0.1 < msg.data < 9 and not self._low_battery_playing:
            self._low_battery_playing = True
            threading.Thread(target=self._play_low_battery_warning, daemon=True).start()

    def _play_low_battery_warning(self):
        subprocess.run(['aplay', '-D', 'plughw:3,0',
                        '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
        time.sleep(5)
        self._low_battery_playing = False

    # Forward /cmd_vel to the base platform over UART
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        if linear_velocity == 0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_cmd_vel_nonzero = (linear_velocity != 0.0 or angular_velocity != 0.0)
        self.base_controller.base_json_ctrl({'T': 13, 'X': linear_velocity, 'Z': angular_velocity})

    # Forward /ugv/joint_states pan/tilt commands to the base over UART
    def joint_states_callback(self, msg):
        name = msg.name
        position = msg.position
        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]
        x_degree = (180 * x_rad) / math.pi
        y_degree = (180 * y_rad) / math.pi
        self.base_controller.base_json_ctrl({'T': 134, 'X': x_degree, 'Y': y_degree, 'SX': 600, 'SY': 600})

    # Forward /ugv/led_ctrl commands to the base over UART
    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        self.base_controller.base_json_ctrl({'T': 132, 'IO4': IO4, 'IO5': IO5})

    def destroy_node(self):
        # Stop motors and disable serial feedback — write directly to serial
        # (bypass the queue) so this is guaranteed to flush before the process exits
        self.base_controller.emergency_stop()
        super().destroy_node()

# Main function to initialize the ROS node and start spinning
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS
    node = ugv_bringup()  # Create the UGV bringup node

    # Ensure motors stop and feedback is disabled when the process is killed
    # by docker stop, systemd, or the OOM killer (SIGTERM).
    def _sigterm_handler(signum, frame):
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()