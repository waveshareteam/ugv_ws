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
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils


class GestureCtrl(Node):
    def __init__(self):
        super().__init__('gesture_ctrl')
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        self._action_client = ActionClient(self, Behavior, 'behavior') 
        self.gesture_ctrl_publisher = self.create_publisher(Image, '/gesture_ctrl/result', 10)
        self.bridge = CvBridge()

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
                
                if gesture_type == 1: 
                    print("turn right")
                    data = [{"T": 1, "type": "spin", "data": -1}]
                elif gesture_type == 2: 
                    print("turn left")
                    data = [{"T": 1, "type": "spin", "data": 1}]
                else:
                    if gesture_type == 3:
                        print("move forward")
                        data = [{"T": 1, "type": "drive_on_heading", "data": 0.01}]
                    elif gesture_type == 4:
                        print("move back")
                        data = [{"T": 1, "type": "back_up", "data": 0.01}]
                    else:
                        print("stop")
                        data = [{"T": 1, "type": "stop", "data": 0}]

                json_str = json.dumps(data)
                self.send_goal(json_str)
            
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

