#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
	"""Driving my robot
	"""

	def __init__(self):
		rospy.loginfo("Starting node")
		self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist) # 

	# A function to send velocities until the node is killed
	def send_velocities(self):
		r = rospy.Rate(10) # Setting a rate (hz) at which to publish
		while not rospy.is_shutdown(): # Running until KILLED
			rospy.loginfo("Sending commands")
			twist_msg = Twist() # Creating a new message to send to the robot

			twist_msg.linear.x = 0.25 # expand the circle (Max of 0.6 velocity plz)
			twist_msg.angular.z = 0.5 # just rotate. Without velocity it would just spin.

			# rotate on the spot or drive in a circle?

			self.pub.publish(twist_msg) # Sending the message via our publisher
			r.sleep() # Calling sleep to ensure the rate we set above

if __name__ == '__main__':
	rospy.init_node("command_velocity")
	cv = CommandVelocity()
	cv.send_velocities() # Calling the function
	rospy.spin()
