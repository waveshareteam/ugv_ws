import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros import TransformStamped
from std_msgs.msg import Int8

import json
import math
from ugv_interface.action import Behavior
from rclpy.action import ActionClient

class TransformProcessor(Node):
    def __init__(self):
        super().__init__('transform_processor')

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Client
        self._action_client = ActionClient(self, Behavior, 'behavior')

        # Timer to periodically check for transforms
        self.track_sub = self.create_subscription(Int8, '/apriltag/track', self.check_transform, 10)
        # Time of the last received transform
        self.last_transform_time = None

    def check_transform(self,msg):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform('base_footprint', 'dock_frame', now)
            print(trans.transform.translation)
            # Extract position and orientation from the transform
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            distance = math.sqrt(x**2 + y**2)
            angle_radians = math.atan2(abs(x), abs(y))
            angle_degrees = math.degrees(angle_radians)

            if y < 0:
                angle_degrees = -90 + angle_degrees
            else:
                angle_degrees = 90 - angle_degrees

            self.get_logger().info(f'Sent spin goal with angle {angle_degrees}')

            data = [
                {"T": 1, "type": "spin", "data": angle_degrees},
                {"T": 1, "type": "drive_on_heading", "data": distance}
            ]
            self.send_goal(json.dumps(data))
            self.get_logger().info(f'Sent drive goal with distance {distance}')

        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')

    def send_goal(self, command):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = Behavior.Goal()
        goal_msg.command = command

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        
def main(args=None):
    rclpy.init(args=args)
    node = TransformProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

