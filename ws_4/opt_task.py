#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Note: I plan on referencing this very soon (tomorrow?)

I will comment the heck out of this so it has a clear understanding.

1. Terminal 1: roslaunch uol_turtlebot_simulator simple.launch
2. Terminal 2: run your python code
3. Drop coloured objects. I recommend "Textured shapes".

"""

import cv2		# OpenCV library
import numpy as np 	# Used for image processing/matrices
import sys			
import rospy

from cv_bridge import CvBridge, CvBridgeError	# Bridge (for converting ROS image to OpenCV) and error log
from geometry_msgs.msg import Twist	
from numpy import mean				# Can be used to find the average pixel value in an image 
from std_msgs.msg import String
from sensor_msgs.msg import Image

class pursue_colour():

	def __init__(self):

		self.node_name = "pursue_colour" 
		rospy.init_node(self.node_name) 

		self.cv_window_name = self.node_name 
		self.bridge = CvBridge() 

		# Publishes a custom string. It can say whatever you want.
		self.pub = rospy.Publisher("/result_topic", String, queue_size=1)
		# Subscribes to the image, which has just been converted into OpenCV from ROS Image
		self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows
		# Publishes velocity commands. Like the name says. 
		self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		
		# An instance of Twist.
  		self.twist = Twist()

	# Exactly what the name states. A functional block to open windows to view images.
	def open_windows(self,event):
		try:
			
   			# Just some naming conventions for each window. "WINDOW_NORMAL" makes the window more versatile (snaps to sides, enlarges etc)
			cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL) 
			cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)
			
			# "Cammy" is just the default RGB (BGR) camera view.
			cv2.imshow("Cammy",self.cam_view)
			# "Slice" displays the segmented colour (HSV) set within 'color_slice'. 
			cv2.imshow("Slice",self.processed_image)	
   
			# This is not really needed, but meh. Just something to give a bit of pause.
			cv2.waitKey(3)
		except:
			pass # bad exception handling. It works. Screw it.
			
	def image_callback(self, data):
		try:
			# calling the class variable, bridge [CvBridge()] 
 			# This takes the ROS Image messages and convert them into OpenCV format.
			# "bgr8" is and 8-bit RGB Image. 
 			# The reason it is backwards (RGB) is because the image is perceived by the camera as inverse matrices. I think?
			self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError ("e"): # try and catch called from Cv2
			print ("e") # print whatever did/did not happen (error)
			pass #...

		cam_view = np.array(self.cam_view, dtype = np.uint8)
		self.processed_image = self.color_slice(cam_view)

	def color_slice(self, cam_view):
		hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
	

		# define range of green color in HSV (upper and lower boundary)
		lower_green = np.array([30,30,30])
		upper_green = np.array([100,255,255])
		
		mask = cv2.inRange(hsv, lower_green, upper_green) # create a mask based on those boundaries (view shows hsv colours in range)
		masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=mask) # mask the camera view to segment out the colour green
		
		h, w, d = cam_view.shape # height, width, depth of the camera display (for the mean stuff)
		M = cv2.moments(mask)
	
		# I will explain this properly tomorrow, but you probably get it
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(cam_view, (cx, cy), 20, (0, 0, 255), -1)
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			
			self.cmd_vel_pub.publish(self.twist)
			#cv2.imshow("window", cam_view)
			# cv2.waitKey(3)
		
		return masked


		"""
		Example 1: Get the mean
		print np.mean(hsv[:, :, 0]) #H?
				print np.mean(hsv[:, :, 1]) #S?
				print np.mean(hsv[:, :, 2]) #V?
		print mean(hsv)
		#-----------------------------------------
		Example 2: Get the mean
		# the shape gives you the dimensions
		h = img3.shape[0]
		w = img3.shape[1]
		# loop over the image, pixel by pixel
		count = 0
		# a slow way to iterate over the pixels
		for y in range(0, h):
			for x in range(0, w):
			
		# threshold the pixel
				if img3[y, x] > 0:
					count += 1
		print('count edge pixels: %d' % count)
		"""

if __name__ == '__main__':
	cv2.startWindowThread()
	pursue_colour()
	rospy.spin()
