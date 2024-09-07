import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVAdjuster(Node):
    def __init__(self):
        super().__init__('hsv_adjuster')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Initial HSV values
        self.h_min = 0
        self.s_min = 0
        self.v_min = 0
        self.h_max = 179
        self.s_max = 255
        self.v_max = 255

        # Create a window with trackbars
        cv2.namedWindow('HSV Adjuster')
        cv2.createTrackbar('H Min', 'HSV Adjuster', 0, 179, self.nothing)
        cv2.createTrackbar('S Min', 'HSV Adjuster', 0, 255, self.nothing)
        cv2.createTrackbar('V Min', 'HSV Adjuster', 0, 255, self.nothing)
        cv2.createTrackbar('H Max', 'HSV Adjuster', 179, 179, self.nothing)
        cv2.createTrackbar('S Max', 'HSV Adjuster', 255, 255, self.nothing)
        cv2.createTrackbar('V Max', 'HSV Adjuster', 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get current positions of the trackbars
        self.h_min = cv2.getTrackbarPos('H Min', 'HSV Adjuster')
        self.s_min = cv2.getTrackbarPos('S Min', 'HSV Adjuster')
        self.v_min = cv2.getTrackbarPos('V Min', 'HSV Adjuster')
        self.h_max = cv2.getTrackbarPos('H Max', 'HSV Adjuster')
        self.s_max = cv2.getTrackbarPos('S Max', 'HSV Adjuster')
        self.v_max = cv2.getTrackbarPos('V Max', 'HSV Adjuster')

        # Convert the image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create a mask based on the current HSV trackbar values
        lower_hsv = np.array([self.h_min, self.s_min, self.v_min])
        upper_hsv = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)

        # Apply the mask to the original image
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Show the result
        cv2.imshow('HSV Adjuster', result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    hsv_adjuster = HSVAdjuster()

    try:
        rclpy.spin(hsv_adjuster)
    except KeyboardInterrupt:
        pass

    hsv_adjuster.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

