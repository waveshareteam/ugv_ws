#!/usr/bin/env python
# encoding: utf-8
import sys, select, termios, tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Car!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

speedBindings = {
    'Q': (1.1, 1.1),
    'Z': (.9, .9),
    'W': (1.1, 1),
    'X': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}



class ugv_Keyboard(Node):
	def __init__(self,name):
		# Initialize the node
		super().__init__(name)
		# Create a publisher to publish the Twist message
		self.pub = self.create_publisher(Twist,'cmd_vel',1)
		# Declare parameters for linear and angular speed limits
		self.declare_parameter("linear_speed_limit",1.0)
		self.declare_parameter("angular_speed_limit",1.0)
		# Get the parameter values
		self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		# Get the terminal settings
		self.settings = termios.tcgetattr(sys.stdin)
	def getKey(self):
		# Set the terminal to raw mode
		tty.setraw(sys.stdin.fileno())
		# Wait for input
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		# If there is input, read it
		if rlist: key = sys.stdin.read(1)
		# If there is no input, set key to empty string
		else: key = ''
		# Set the terminal back to normal mode
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		# Return the key
		return key
	def vels(self, speed, turn):
		# Return the current speed and turn
		return "currently:\tspeed %s\tturn %s " % (speed,turn)		
	
def main():
	# Initialize the ROS 2 Python client library
	rclpy.init()
	# Create an instance of the ugv_Keyboard class
	ugv_keyboard = ugv_Keyboard("keyboard_ctrl")
	# Set the initial speed and turn
	xspeed_switch = True
	(speed, turn) = (0.2, 0.5)
	# Set the initial position
	(x, th) = (0, 0)
	# Set the initial status
	status = 0
	# Set the initial stop state
	stop = False
	# Set the initial count
	count = 0
	# Create a Twist message
	twist = Twist()
	try:
		# Print the message
		print(msg)
		# Print the initial speed and turn
		print(ugv_keyboard.vels(speed, turn))
		# Loop forever
		while (1):
			# Get the key
			key = ugv_keyboard.getKey()
			# If the key is t or T, switch the x and y speed
			if key=="t" or key == "T": xspeed_switch = not xspeed_switch
			# If the key is s or S, stop the keyboard control
			elif key == "s" or key == "S":
				print ("stop keyboard control: {}".format(not stop))
				stop = not stop
			# If the key is in the moveBindings dictionary, set the x and y position
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
				count = 0	
			# If the key is in the speedBindings dictionary, set the speed and turn
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				count = 0
				# If the speed is greater than the linear speed limit, set it to the limit
				if speed > ugv_keyboard.linenar_speed_limit: 
					speed = ugv_keyboard.linenar_speed_limit
					print("Linear speed limit reached!")
				# If the turn is greater than the angular speed limit, set it to the limit
				if turn > ugv_keyboard.angular_speed_limit: 
					turn = ugv_keyboard.angular_speed_limit
					print("Angular speed limit reached!")
				# Print the current speed and turn
				print(ugv_keyboard.vels(speed, turn))
				# If the status is 14, print the message
				if (status == 14): print(msg)
				# Increment the status
				status = (status + 1) % 15
			# If the key is space or k, set the x and y position to 0
			elif key == ' ': (x, th) = (0, 0)
			# If the key is not in the moveBindings or speedBindings dictionaries, increment the count
			else:
				count = count + 1
				# If the count is greater than 4, set the x and y position to 0
				if count > 4: (x, th) = (0, 0)
				# If the key is ctrl-c, break the loop
				if (key == '\x03'): break
			# If the xspeed_switch is true, set the x speed
			if xspeed_switch: twist.linear.x = speed * x
			# If the xspeed_switch is false, set the y speed
			else: twist.linear.y = speed * x
			# Set the turn
			twist.angular.z = turn * th
			# If the stop state is false, publish the Twist message
			if not stop: ugv_keyboard.pub.publish(twist)
			# If the stop state is true, publish an empty Twist message
			if stop:ugv_keyboard.pub.publish(Twist())
	except Exception as e: print(e)
	finally: ugv_keyboard.pub.publish(Twist())
	# Set the terminal back to normal mode
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ugv_keyboard.settings)
	# Destroy the node
	ugv_keyboard.destroy_node()
	# Shutdown the ROS 2 Python client library
	rclpy.shutdown()
