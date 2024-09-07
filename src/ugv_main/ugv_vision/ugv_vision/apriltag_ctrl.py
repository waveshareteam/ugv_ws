import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.action import ActionClient
from ugv_interface.action import Behavior
from rclpy.duration import Duration
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
from apriltag import apriltag

class ApriltagCtrl(Node):
    def __init__(self):
        super().__init__('apriltag_ctrl')
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        self._action_client = ActionClient(self, Behavior, 'behavior')  
        self.apriltag_ctrl_publisher = self.create_publisher(Image, '/apriltag_ctrl/result', 10)
        self.bridge = CvBridge()
        self.detector = apriltag("tag36h11")
        
    def send_goal(self, command):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = Behavior.Goal()
        goal_msg.command = command

        self.get_logger().info('Sending goal...')
        self._action_client.wait_for_server()
        
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
        self.get_logger().info('Result: {0}'.format(result.result))
   
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(gray)
        #print(results)
        for r in results:
            corners = r['lb-rb-rt-lt'].astype(int)
        
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        
            center_x, center_y = int(r['center'][0]), int(r['center'][1])
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')
        
            if r['id'] == 1:
                print("turn right")
                data = [{"T": 1, "type": "spin", "data": -1}]
            elif r['id'] == 2:
                print("turn left")
                data = [{"T": 1, "type": "spin", "data": 1}]
            elif r['id'] == 3:
                print("move forward")
                data = [{"T": 1, "type": "drive_on_heading", "data": 0.01}]
            elif r['id'] == 4:
                print("move back")
                data = [{"T": 1, "type": "back_up", "data": 0.01}]
            else:
                print("stop")
                data = [{"T": 1, "type": "stop", "data": 0}]
        
            json_str = json.dumps(data)
            self.send_goal(json_str)
        
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.apriltag_ctrl_publisher.publish(result_img_msg)
        cv2.imshow('ctrled Image', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    apriltag_ctrl = ApriltagCtrl()
    rclpy.spin(apriltag_ctrl)
    apriltag_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

