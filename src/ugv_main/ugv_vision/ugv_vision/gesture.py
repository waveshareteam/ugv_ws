import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils


class GestureCtrl(Node):
    def __init__(self):
        super().__init__('gesture_ctrl')
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        self.gesture_ctrl_publisher = self.create_publisher(Image, '/gesture_ctrl/result', 10)
        self.bridge = CvBridge()
      
    def detect_gesture(self, hand_landmarks):
        lmlist=[]
        tipids=[4,8,12,16,20]

        for id,lm in enumerate(hand_landmarks.landmark):
          
          h,w,c= 480,640,3
          cx,cy=int(lm.x * w) , int(lm.y * h)
          lmlist.append([id,cx,cy])
          if len(lmlist) != 0 and len(lmlist)==21:
              fingerlist=[]
              
              #thumb and dealing with flipping of hands
              if lmlist[12][1] > lmlist[20][1]:
                  if lmlist[tipids[0]][1] > lmlist[tipids[0]-1][1]:
                      fingerlist.append(1)
                  else:
                      fingerlist.append(0)
              else:
                  if lmlist[tipids[0]][1] < lmlist[tipids[0]-1][1]:
                      fingerlist.append(1)
                  else:
                      fingerlist.append(0)
              
              #others
              for id in range (1,5):
                  if lmlist[tipids[id]][2] < lmlist[tipids[id]-2][2]:
                      fingerlist.append(1)
                  else:
                      fingerlist.append(0)
              
              
              if len(fingerlist)!=0:
                  fingercount=fingerlist.count(1)
                  return fingercount
                   
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)   
  
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                gesture_type = self.detect_gesture(hand_landmarks)        
                print(gesture_type)
            
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        self.gesture_ctrl_publisher.publish(result_img_msg)
        cv2.imshow('Tracked Image', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    gesture_ctrl = GestureCtrl()
    rclpy.spin(gesture_ctrl)
    gesture_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

