import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ugv_interface.action import Behavior
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
from apriltag import apriltag

class ApriltagTracker(Node):
    def __init__(self):
        super().__init__('apriltag_tracker')
        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create an action client to send goals to the behavior action server
        self._action_client = ActionClient(self, Behavior, 'behavior')  
        # Create a publisher to publish the tracked image
        self.apriltag_track_publisher = self.create_publisher(Image, '/apriltag_track/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        # Create an apriltag detector
        self.detector = apriltag("tag36h11")
        
    def send_goal(self, command):
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        # Create a goal message
        goal_msg = Behavior.Goal()
        goal_msg.command = command

        self.get_logger().info('Sending goal...')
        # Wait for the action server to be available
        self._action_client.wait_for_server()
        
        # Send the goal message
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Add a callback to the goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Get the goal handle
        goal_handle = future.result()
        # Check if the goal was accepted
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted')

        # Get the result of the goal
        self._get_result_future = goal_handle.get_result_async()
        # Add a callback to the result
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Get the result
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
   
    def image_callback(self, msg):

        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect apriltags in the image
        results = self.detector.detect(gray)

        # Loop through the detected apriltags
        for r in results:

            # Get the corners of the apriltag
            corners = r['lb-rb-rt-lt'].astype(int)
        
            # Draw a polygon around the apriltag
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        
            # Get the center of the apriltag
            center_x, center_y = int(r['center'][0]), int(r['center'][1])
            # Draw a circle at the center of the apriltag
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Print the tag ID and center coordinates
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')

            # Determine the command to send based on the center coordinates
            if (center_x - 320) > 30: 
                print("turn right")
                data = [{"T": 1, "type": "spin", "data": -1}]
            elif (320 - center_x) > 30: 
                print("turn left")
                data = [{"T": 1, "type": "spin", "data": 1}]
            else:
                if (240 - center_y) > 30:
                    print("move forward")
                    data = [{"T": 1, "type": "drive_on_heading", "data": 0.01}]
                elif (center_y - 240) > 30:
                    print("move back")
                    data = [{"T": 1, "type": "back_up", "data": 0.01}]
                else:
                    print("stop")
                    data = [{"T": 1, "type": "stop", "data": 0}]

            # Convert the command to a JSON string
            json_str = json.dumps(data)
            # Send the goal
            self.send_goal(json_str)

        # Convert the OpenCV image to a ROS Image message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the tracked image
        self.apriltag_track_publisher.publish(result_img_msg)
        # Show the tracked image
        cv2.imshow('Tracked Image', frame)
        # Wait for a key press
        cv2.waitKey(1)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of the ApriltagTracker class
    apriltag_tracker = ApriltagTracker()
    # Spin the ROS client library
    rclpy.spin(apriltag_tracker)
    # Destroy the ApriltagTracker instance
    apriltag_tracker.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()

