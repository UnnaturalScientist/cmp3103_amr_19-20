#!/usr/bin/env python

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import mean
import numpy as np

class colorContour():

	def __init__(self):

		self.node_name = "color_contour"
		rospy.init_node(self.node_name)

		self.cv_window_name = self.node_name
		self.bridge = CvBridge()

		# output is then published as a string. 
		# queue size 10, because...
		self.pub = rospy.Publisher("/result_topic", String, queue_size=10)
		# basically:
		# [class instance = client library.node subscription("topic", ROS Message Type, call for method)]
		self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows

	# function to display images
	def open_windows(self,event):
		try:

			# "namedWindow" is exactly that. A title/header for the window
			# cv2, means a normal fixed window. There are alternatives where the sizeis more dynamic.
			# Names the window "Cammy", not Canny. 
			cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL)
			# Names the window Slice. 
			cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)

			# imshow (Image Show). Shows..the..image.
			cv2.imshow("Cammy",self.cam_view)
			# Presents the "Processed Image" with the colour sliced out.
			cv2.imshow("Slice",self.processed_image)

			# Unsure, check in simulation.
			cv2.waitKey(3)
	
		# not good practice, but it works.
		# really should have better exception handling, but whatever.
		except:
			pass

	# function to 
	def image_callback(self, data):
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
			# This takes the data from 
			self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e: # try and catch called from Cv2
			print e
			pass

		# takes the numpy array of the camera view
		# it uses an unsigned 8-bit integer 
		# this has a max value of 255 and this is perect for RGB.
		cam_view = np.array(self.cam_view, dtype = np.uint8)
		self.processed_image = self.color_slice(cam_view)

	def color_slice(self, cam_view):
		hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
		lower_blue = np.array([, 100, 100]) # HSV not RGB
		upper_blue = np.array([255, 255, 250])
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=mask)
		
		"""
		HSV = Hue, Saturation and Value
		
		HUE
		____
		
		Red falls between 0 and 60 degrees
		Yellow falls between 61 and 120 degrees
		Green falls between 121-180 degrees
		Cyan falls between 181-240 degrees
		Blue falls between 241-300 degrees
		Magenta falls between 301-360

		SATURATION
		__________

		Describes the level of gray in a colour.
		Measured from 0 to 100% (zero being most gray)
		Can also appear as 0-1 (one is the primary colour)

		VALUE (Brightness)
		__________________

		Works in conjunction with saturation.
		Is brightness of colour 0-100%
		0 = black
		100 = the brightest form of that chosen colour
		
		QUICK COLOURS
		______________

		"""
		self.pub.publish(str(np.mean(hsv[:, :, 0]))) # Hue layer
		self.pub.publish(str(np.mean(hsv[:, :, 1]))) # Saturation
		self.pub.publish(str(np.mean(hsv[:, :, 2]))) # Value/Brightness
	#print mean(hsv)
	
		return masked
	

if __name__ == '__main__':

		# Executes the class "colourContours"
		colorContour()
		# Spin to Win
		rospy.spin()

# (Quigley et al., 2015, 193-208)
# (Lentin, 2018, 176-189)

"""
Quigley, M., Gerkey, B. and Smart, W.D. (2015) Programming Robots with ROS.
Sebastopol, USA: O'Reilly Media, Inc.

Lentin, J. (2018) Learning Robotics sing Python, 2nd Edition.
Birmingham, UK: Packt Publishing Ltd.

https://en.wikipedia.org/wiki/HSL_and_HSV

"""
