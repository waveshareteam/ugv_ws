# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import mediapipe as mp

# Initialize the mediapipe hands module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils


# Define the GestureCtrl class
class GestureCtrl(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('gesture_ctrl')
        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create a publisher to the gesture_ctrl/result topic
        self.gesture_ctrl_publisher = self.create_publisher(Image, '/gesture_ctrl/result', 10)
        # Create a CvBridge object
        self.bridge = CvBridge()
      
    # Define the function to detect the gesture
    def detect_gesture(self, hand_landmarks):
        lmlist=[]
        tipids=[4,8,12,16,20]

        # Loop through the hand landmarks
        for id,lm in enumerate(hand_landmarks.landmark):
          
          h,w,c= 480,640,3
          # Calculate the center of the landmark
          cx,cy=int(lm.x * w) , int(lm.y * h)
          # Append the landmark to the list
          lmlist.append([id,cx,cy])
          # If the list is not empty and has 21 landmarks
          if len(lmlist) != 0 and len(lmlist)==21:
              fingerlist=[]
              
              #thumb and dealing with flipping of hands
              # If the x-coordinate of the 12th landmark is greater than the x-coordinate of the 20th landmark
              if lmlist[12][1] > lmlist[20][1]:
                  # If the x-coordinate of the 4th landmark is greater than the x-coordinate of the 3rd landmark
                  if lmlist[tipids[0]][1] > lmlist[tipids[0]-1][1]:
                      fingerlist.append(1)
                  else:
                      fingerlist.append(0)
              # If the x-coordinate of the 12th landmark is less than the x-coordinate of the 20th landmark
              else:
                  # If the x-coordinate of the 4th landmark is less than the x-coordinate of the 3rd landmark
                  if lmlist[tipids[0]][1] < lmlist[tipids[0]-1][1]:
                      fingerlist.append(1)
                  else:
                      fingerlist.append(0)
              
              #others
              # Loop through the tipids list
              for id in range (1,5):
                  # If the y-coordinate of the current landmark is less than the y-coordinate of the previous landmark
                  if lmlist[tipids[id]][2] < lmlist[tipids[id]-2][2]:
                      fingerlist.append(1)
                  else:
                      fingerlist.append(0)
              
              
              # If the fingerlist is not empty
              if len(fingerlist)!=0:
                  # Count the number of 1s in the fingerlist
                  fingercount=fingerlist.count(1)
                  # Return the fingercount
                  return fingercount
                   
    # Define the callback function for the image_raw topic
    def image_callback(self, msg):
        # Convert the image message to a cv2 image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the image to RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Process the image using the mediapipe hands module
        results = hands.process(image_rgb)   
  
        # If there are hand landmarks
        if results.multi_hand_landmarks:
            # Loop through the hand landmarks
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw the hand landmarks on the image
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                # Detect the gesture
                gesture_type = self.detect_gesture(hand_landmarks)        
                # Print the gesture type
                print(gesture_type)
            
        # Convert the image back to a message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the message
        self.gesture_ctrl_publisher.publish(result_img_msg)
        # Show the image
        cv2.imshow('Tracked Image', frame)
        # Wait for a key press
        cv2.waitKey(1)

# Define the main function
def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    # Create an instance of the GestureCtrl class
    gesture_ctrl = GestureCtrl()
    # Spin the node
    rclpy.spin(gesture_ctrl)
    # Destroy the node
    gesture_ctrl.destroy_node()
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

# If the script is run directly
if __name__ == '__main__':
    # Run the main function
    main()

