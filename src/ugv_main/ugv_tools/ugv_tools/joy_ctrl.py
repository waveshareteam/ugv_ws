#!/usr/bin/env python
# encoding: utf-8

#public lib
import os
import time
import getpass
import threading
from time import sleep

#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool

class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.Joy_active = False
		self.Buzzer_active = False
		self.RGBLight_index = 0
		self.cancel_time = time.time()
		self.user_name = getpass.getuser()
		self.linear_Gear = 1
		self.angular_Gear = 1
		
		#create pub
		self.pub_goal = self.create_publisher(GoalID,"move_base/cancel",10)
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		self.pub_Buzzer = self.create_publisher(Bool,"Buzzer",  1)
		self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
		self.pub_RGBLight = self.create_publisher(Int32,"RGBLight" , 10)
		
		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)
		
		#declare parameter and get the value
		self.declare_parameter('xspeed_limit',0.5)
		self.declare_parameter('yspeed_limit',0.5)
		self.declare_parameter('angular_speed_limit',1.0)
		self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
		self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
		
		
	def buttonCallback(self,joy_data):
		if not isinstance(joy_data, Joy): return
		if self.user_name == "root": self.user_jetson(joy_data)
		else: self.user_pc(joy_data)
		
	def user_jetson(self, joy_data):
		#cancel nav
		if joy_data.buttons[9] == 1: self.cancel_nav()
		#RGBLight
		if joy_data.buttons[7] == 1:
                    RGBLight_ctrl = Int32()
                    if self.RGBLight_index < 6:
                        RGBLight_ctrl.data = self.RGBLight_index
                        for i in range(3): self.pub_RGBLight.publish(RGBLight_ctrl)
                    else: self.RGBLight_index = 0
                    self.RGBLight_index += 1
		#Buzzer
		if joy_data.buttons[11] == 1:
			Buzzer_ctrl = Bool() 
			self.Buzzer_active=not self.Buzzer_active
			Buzzer_ctrl.data =self.Buzzer_active
			for i in range(3): self.pub_Buzzer.publish(Buzzer_ctrl)
        #linear Gear control
		if joy_data.buttons[13] == 1:
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        # angular Gear control
		if joy_data.buttons[14] == 1:
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
		xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        #ylinear_speed = self.filter_data(joy_data.axes[2]) * self.yspeed_limit * self.linear_Gear
		ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
		angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		if self.Joy_active == True:
			print("joy control now")
			for i in range(3): self.pub_cmdVel.publish(twist)
        
	def user_pc(self, joy_data):

        # 取消 Cancel
		if joy_data.axes[5] == -1: self.cancel_nav()
		if joy_data.buttons[5] == 1:
			if self.RGBLight_index < 6:
				self.pub_RGBLight.publish(self.RGBLight_index)
                # print ("pub RGBLight success")
			else: self.RGBLight_index = 0
			self.RGBLight_index += 1
		if joy_data.buttons[7] == 1:
			self.Buzzer_active=not self.Buzzer_active
            # print "self.Buzzer_active: ", self.Buzzer_active
			self.pub_Buzzer.publish(self.Buzzer_active)
        # 档位控制 Gear control
		if joy_data.buttons[9] == 1:
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
		if joy_data.buttons[10] == 1:
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
		xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
		ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
		angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		for i in range(3): self.pub_cmdVel.publish(twist)
        
	def filter_data(self, value):
		if abs(value) < 0.2: value = 0
		return value
		
	def cancel_nav(self):
		now_time = time.time()
		if now_time - self.cancel_time > 1:
			Joy_ctrl = Bool()
			self.Joy_active = not self.Joy_active
			Joy_ctrl.data = self.Joy_active
			for i in range(3):
				self.pub_JoyState.publish(Joy_ctrl)
				#self.pub_goal.publish(GoalID())
				self.pub_cmdVel.publish(Twist())
			self.cancel_time = now_time
			
def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('joy_ctrl')
	rclpy.spin(joy_ctrl)	
main()		
