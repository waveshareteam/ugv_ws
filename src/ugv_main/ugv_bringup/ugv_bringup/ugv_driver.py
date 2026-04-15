#!/usr/bin/env python3
# ugv_driver.py — UART access has been consolidated into ugv_bringup.py.
# This node is retained as a no-op stub so that any launch file that still
# references the 'ugv_driver' executable does not crash.  It subscribes to
# the same topics as before but takes no action.  Remove it from launch files
# and use ugv_bringup for all base-platform communication.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray


class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().warn(
            'ugv_driver is a no-op stub. '
            'UART commands are now handled by ugv_bringup. '
            'Remove ugv_driver from your launch files.'
        )
        self.create_subscription(Twist, 'cmd_vel', lambda msg: None, 10)
        self.create_subscription(JointState, 'ugv/joint_states', lambda msg: None, 10)
        self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', lambda msg: None, 10)
        self.create_subscription(Float32, 'voltage', lambda msg: None, 10)


def main(args=None):
    rclpy.init(args=args)
    node = UgvDriver('ugv_driver')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
