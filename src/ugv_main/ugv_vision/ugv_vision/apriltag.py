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
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        self.apriltag_ctrl_publisher = self.create_publisher(Image, '/apriltag_ctrl/result', 10)
        self.bridge = CvBridge()
        self.detector = apriltag("tag36h11")
        
    def detect_apritag(self, frame):

        return type
   
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(gray)

        for r in results:
            corners = r['lb-rb-rt-lt'].astype(int)
        
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        
            center_x, center_y = int(r['center'][0]), int(r['center'][1])
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')

        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        self.apriltag_ctrl_publisher.publish(result_img_msg)
        cv2.imshow('ctrled Image', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    apriltag_ctrl = Apriltagctrl()
    rclpy.spin(apriltag_ctrl)
    apriltag_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

