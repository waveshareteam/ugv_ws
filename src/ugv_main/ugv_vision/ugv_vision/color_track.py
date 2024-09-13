import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ugv_interface.action import Behavior
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge

import cv2
import numpy as np
import json

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create an action client to send goals to the behavior action server
        self._action_client = ActionClient(self, Behavior, 'behavior')
        # Create a publisher to publish the tracked image to the color_track/result topic
        self.color_track_publisher = self.create_publisher(Image, '/color_track/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        #self.lower_color = np.array([0, 115, 0]) 
        #self.upper_color = np.array([5, 255, 255])  
        # Declare parameters for lower and upper hue, saturation, and value
        self.declare_parameter("lower_hue", 0, ParameterDescriptor(description="Lower Hue"))
        self.declare_parameter("lower_saturation", 115, ParameterDescriptor(description="Lower Saturation"))
        self.declare_parameter("lower_value", 0, ParameterDescriptor(description="Lower Value"))
        
        self.declare_parameter("upper_hue", 5, ParameterDescriptor(description="Upper Hue"))
        self.declare_parameter("upper_saturation", 255, ParameterDescriptor(description="Upper Saturation"))
        self.declare_parameter("upper_value", 255, ParameterDescriptor(description="Upper Value"))

        # Initialize the lower and upper color arrays with the parameter values
        self.lower_color = np.array([self.get_parameter("lower_hue").value, 
                                     self.get_parameter("lower_saturation").value, 
                                     self.get_parameter("lower_value").value])
        self.upper_color = np.array([self.get_parameter("upper_hue").value, 
                                     self.get_parameter("upper_saturation").value, 
                                     self.get_parameter("upper_value").value])
        
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
        # Convert the image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Get the parameter values for lower and upper hue, saturation, and value
        lower_hue = self.get_parameter("lower_hue").value
        lower_saturation = self.get_parameter("lower_saturation").value
        lower_value = self.get_parameter("lower_value").value
        
        upper_hue = self.get_parameter("upper_hue").value
        upper_saturation = self.get_parameter("upper_saturation").value
        upper_value = self.get_parameter("upper_value").value

        # Update the lower and upper color arrays with the parameter values
        self.lower_color = np.array([lower_hue, lower_saturation, lower_value])
        self.upper_color = np.array([upper_hue, upper_saturation, upper_value])

        
        # Create a mask for the color range
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        # Find the contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through the contours
        for contour in contours:
            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # Check if the width and height of the rectangle are within a certain range
            if 30 < w < 300 and 10 < h < 300:
                # Draw a rectangle around the contour
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # Calculate the center of the rectangle
                cx = x + w // 2
                cy = y + h // 2
                # Draw a circle at the center of the rectangle
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                self.get_logger().info(f'Tracking block at ({cx}, {cy})')
                
                # Check if the center of the rectangle is within a certain range of the center of the image
                if (cx - 320) > 30: 
                    print("turn right")
                    data = [{"T": 1, "type": "spin", "data": -1}]
                elif (320 - cx) > 30: 
                    print("turn left")
                    data = [{"T": 1, "type": "spin", "data": 1}]
                else:
                    # Check if the center of the rectangle is within a certain range of the center of the image
                    if (240 - cy) > 30:
                        print("move forward")
                        data = [{"T": 1, "type": "drive_on_heading", "data": 0.01}]
                    elif (cy - 240) > 30:
                        print("move back")
                        data = [{"T": 1, "type": "back_up", "data": 0.01}]
                    else:
                        print("stop")
                        data = [{"T": 1, "type": "stop", "data": 0}]

                # Convert the data to a JSON string
                json_str = json.dumps(data)
                # Send the goal
                self.send_goal(json_str)
        # Convert the OpenCV image to an image message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the image message
        self.color_track_publisher.publish(result_img_msg)
        # Show the image
        cv2.imshow('Tracked Image', frame)
        # Wait for a key press
        cv2.waitKey(1)

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    # Create a ColorTracker node
    color_tracker = ColorTracker()
    # Spin the node
    rclpy.spin(color_tracker)
    # Destroy the node
    color_tracker.destroy_node()
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()

