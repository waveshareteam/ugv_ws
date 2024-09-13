import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
from apriltag import apriltag

class ApriltagCtrl(Node):
    def __init__(self):
        super().__init__('apriltag_ctrl')
        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create a publisher to the apriltag_ctrl/result topic
        self.apriltag_ctrl_publisher = self.create_publisher(Image, '/apriltag_ctrl/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        # Create an apriltag detector object
        self.detector = apriltag("tag36h11")
        
    def detect_apritag(self, frame):

        return type
   
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
        
            # Print the ID and center of the apriltag
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')

        # Convert the OpenCV image back to a ROS Image message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the result image message
        self.apriltag_ctrl_publisher.publish(result_img_msg)
        # Show the result image
        cv2.imshow('ctrled Image', frame)
        # Wait for 1 millisecond
        cv2.waitKey(1)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    apriltag_ctrl = Apriltagctrl()
    # Create an instance of the ApriltagCtrl node
    apriltag_ctrl = ApriltagCtrl()
    # Spin the node
    rclpy.spin(apriltag_ctrl)
    # Destroy the node
    apriltag_ctrl.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()

