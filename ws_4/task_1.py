#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Workshop4 Task 1

"""
Note: ok, turns out this did work. I just did not publish (step .3, below) in a different terminal.
I was trying to remap instead, like a fool.

For anyone else who might see this:

1. Terminal 1: roslaunch uol_turtlebot_simulator simple.launch
2. Terminal 2: run your python code however
3. Terminal 3: publish with rostopic pub /wheel_vel_left std_msgs/Float32 "data: 1.0" -r 10 (what I missed)
4. Profit.

(I apologise for my attention span and not reading the question properly)
"""

import rospy
import sys
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import mean
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

wheel_radius = 0.03 # 3cm?
robot_radius = 0.17 # radius of the robot. So, the length to the wheel from the robot's centre (top side or bottom, whichever).

class ws4_task1():

	def __init__(self):
		self.node_name = "ws4_task1"
		print("Running") # Just the word running to let you know it is working. Not really that functional.
		rospy.init_node(self.node_name)

		# This topic, wheel_vel_left, is just of our own making. It would say wheel_vel_right and it would still command the left.
  		# The calculations, completed in the functional blocks (forward, inverse kinetmatics etc) define this action of the left wheel turning. Not the word.
		self.sub_left_cmd = rospy.Subscriber("/wheel_vel_left", Float32, self.callbackRun)

		# This will publish velocity commands. It does it via a multiplexer, that is what mux doesdatetime A combination of a date and a time. Attributes: ()

		"""
		http://wiki.ros.org/cmd_vel_mux
		-------------------------------------
  		subscribers:
		- name:        "Joystick control"
			topic:       "/joystick_cmd_vel"
			timeout:     1.0
			priority:    1

		- name:        "Navigation stack"
			topic:       "/move_base_cmd_vel"
			timeout:     1.0
			priority:    0

		publisher:       "output/cmd_vel"

		-------------------------------------
		This is what a mux config looks like
		Just a yaml file which determines
		controller priority. Or any device.

		"""

  		self.pub_cmd_vel= rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1)

  		# I believe this will also work (see below)
  		# self.pub_cmd_vel= rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
		# self.pub_cmd_vel= rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

		# instance of Twist.
		self.twist = Twist()

	# computing the forward kinematics for a differential drive
	def forward_kinematics(self, w_l, w_r):

  		w_l = 1.0 # left wheel
		w_r = 0.0 # right wheel

		# multiply the left/right wheel by the radius of the TB2 wheel. This gives the circumference.
		c_l = wheel_radius * w_l 		# left wheel circumference
		c_r = wheel_radius * w_r 		# right wheel circumference
		v = (c_l + c_r) / 2			# The sum of both, left and right, wheels gives the linear velocity (v). Divded by 2 (per wheel)
		a = (c_r - c_l) / (2 * robot_radius)	# The angular velocity (a) is the difference between the right and left wheel. Multiplied by the robot radius (it turns in rad/s) for each wheel.

		return (v, a) # return the output for linear and angular velocities.

		"""
		__________________________________________________
		|	 FORWARD	|	INVERSE		 |
		__________________________________________________
		|  a. wheel_values	|  c. lin & ang vel	 |
		|  b. circumference 	|  b. circumference      |
		|  c. lin & ang vel	|  a. wheel_values       |
		|			|			 |
		__________________________________________________

		"""

	# computing the inverse kinematics for a differential drive
	# This is, essentially, forward kinematics (but working backwards)
	def inverse_kinematics(self, v, a):

  		c_l = v - (robot_radius * a)
		c_r = v + (robot_radius * a)
  		w_l = c_l / wheel_radius
		w_r = c_r / wheel_radius

		return (w_l, w_r) # Almost mirrored from above

		# w_l = 1.0
		# w_r = 0.0

	# inverse kinematics from a Twist message (This is what a ROS robot has to do)
	def inverse_kinematics_from_twist(self,t):
		return self.inverse_kinematics(t.linear.x, t.angular.z)


	def callbackRun(self, msg):
		(v, a) = self.forward_kinematics(msg.data, 0) # use data from forward kinematics to populate values for v (linear) and a (angular)
		print("v = %f, a = %f" % (v, a)) # print lin and ang velocity as floats

		left_cmd = Twist() 	# instance of twist, then apply v and a to velocity commands
		left_cmd.linear.x = v 	# linear velocity, produced from forward kinematics
		left_cmd.angular.z = a 	# angular velocity, produced from forward kinematics

		self.pub_cmd_vel.publish(left_cmd) # send this output to the publisher and move the left wheel of the TB2

if __name__ == '__main__':
	ws4_task1() 
	rospy.spin() # keep doing this
