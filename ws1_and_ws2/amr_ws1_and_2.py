
"""
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Workshop: Week 1 - Simulation Only (30/1/2020 @ 12:29)

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Tasks 1. to 3. ROS install instructions etc.

Task 4. Linux Basics (Link): 
			__________________________________________________________________________________
			https://help.ubuntu.com/community/UsingTheTerminal
			__________________________________________________________________________________

----------------------------------------------------------------------------------------------

Task 5. ROS Tutorials 1 to 6 (Link): http://wiki.ros.org/ROS/Tutorials

Also, 'First Turtlebot Tutorial': 

	Note: If roscore is not already running, a roscore node is included in the  
	launch of 'roslaunch'.

	5a. Terminal 1: 
			__________________________________________________________________________________
			roslaunch uol_turtlebot_simulator simple.launch (TB2 in Gazebo)
			__________________________________________________________________________________


	5b. Terminal 2: 
			__________________________________________________________________________________
			roslaunch uol_turtlebot_simulator keyop.launch (Move it around)
			__________________________________________________________________________________


	5c. Gazebo > Insert Tab > Add Gazebo model of choice 

	5d. Terminal 3: 
			__________________________________________________________________________________
			roslaunch uol_turtlebot_simulator turtlebot-rviz.launch (RViz)
			__________________________________________________________________________________


	5e. RViz > Tick 'Registered PointCloud' > Expand > Select Topic: /camera/depth/points

	5f. The model selected in 5c should be detected by the TB2 in RViz.

----------------------------------------------------------------------------------------------

Task 6. How can you view topics and how many ROS components are running?

	6a. Use 'rostopic list' to see which topics are active during the launch.

	6b. Components = Nodes. Use 'rosnode list'.

----------------------------------------------------------------------------------------------

Task 7. Print the robot odometry with the following commands:

	7a. rostopic echo /odom

	7b. Note: This can be done for any topic to see if it is present 
		(e.g  for LaserScan  use the topic '/scan')

----------------------------------------------------------------------------------------------

Opt 1. Make the TB2 move with topics:

	opt1a. Either or. The second is the same on one line.
			__________________________________________________________________________________
			user@hostname$ rostopic pub -1 cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'
			__________________________________________________________________________________

			--- Some notes ---

			The topic cmd_vel is published directly from the terminal.
			The TB2 subscribes to this topic.
			
			Well, may have to use cmd_vel_mux/input/navi. Still, the same outcome.

			The TB2 uses a multiplexer (mux), because it uses so many components.
			Warning: This 'mux' observation is an assumption. May not be right, but seems it.

			-1 means that this command should be executed once. 
			linear (x) is measured in metres (SI) per second
			angular (z) is measured in radians (SI) per second

			SI is what ROS uses to measure (Internation System of Units) 
			e.g metres, kg, radians etc.

----------------------------------------------------------------------------------------------

Opt 2. Generate the maze. Use teleop. Do stuff. 
			__________________________________________________________________________________
			opt2a. Maze (https://github.com/LCAS/teaching/tree/kinetic/uol_turtlebot_simulator/worlds)
			__________________________________________________________________________________


----------------------------------------------------------------------------------------------

Opt 3. Familiarise yourself with IDE: 

	opt3a. MS Visual Studio - launch with 'code' in a terminal.

	opt3b. Sypder via Anaconda pkg (link): https://www.anaconda.com/distribution/#linux
		   (Use Python 2.7 version) <-- my preference.

----------------------------------------------------------------------------------------------

Opt 4. Look at Github Repo for module. 

	opt4a. Ok. (link): https://github.com/LCAS/teaching

	opt4b: I put this into my workspace for these workshops 
		   My filepath/workspace being: 
			__________________________________________________________________________________
			jason/home/amr_ws/src[also contains devel and build]/teaching-kinect
			__________________________________________________________________________________

			(See Prep 4 WS2 for environment building)

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



Workshop: Week 2 - ROS Programming (30/1/2020 @ 12:58)

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Prep 1: Some tutorials.

	P1a: Command line tools: 
			__________________________________________________________________________________
			https://youtu.be/_2VmTefxCBk
			__________________________________________________________________________________

	P1b: Python Programming: 
			__________________________________________________________________________________
			https://youtu.be/nHPDBJZWZqo
			__________________________________________________________________________________

----------------------------------------------------------------------------------------------

Prep 2: Setting up the Turtlebot.

	P2a: Get TurtleBot2 from demonstrators. Keep it out of sunlight, do not get it wet and do 
		 not feed it after midnight. No. Matter. What.

	P2b: TB2 Guide (Link): 
			__________________________________________________________________________________
			https://github.com/LCAS/teaching/wiki/Turtlebots#connecting-to-the-robot-from-your-browser
			__________________________________________________________________________________


	P2c: Get TB2 IP (Link): 
			__________________________________________________________________________________
			https://docs.google.com/spreadsheets/d/e/2PACX-1vTfVDcI2Xwud7KN7wBpYEdFRzQcxuVWz6cW01zJrRe6InaBayX4VfgCsUzrAgGoyVPeJXlR7Hq5vmDy/pubhtml?gid=291043266&single=true
			__________________________________________________________________________________
			

	P2d: Open IP in browser.

	P2e: Using TMule Control > click 'All windows' > launch 'roscore' > launch 'turtlebot'

----------------------------------------------------------------------------------------------

Prep 3: VPN

	Note:There is a firewall between the TB2 and ROS. A VPN bypasses that.

	p3a: Access the Robot Web Page: The web page populated by the TB2 IP (See P2c)

	p3b: click 'Dowload VPN client config' > click 'rpi.ovpn' > Store in Downloads.

	p3c: In Terminal: 
			__________________________________________________________________________________
			gedit ~/Downloads/rpi.ovpn
			__________________________________________________________________________________


	p3d: change this: 
			__________________________________________________________________________________
			'remote [enter your VPN address here] 1194'
			__________________________________________________________________________________


	p3e: In terminal: 
			__________________________________________________________________________________
			sudo openvpn ~/[Folder used to save file]/rpi.ovpn
			__________________________________________________________________________________

			e.g  sudo openvpn ~/Downloads/rpi.ovpn

	
	p3f: The client needs to keep running to maintain a connection to the TB2.

	p3g: Local Client has new VPN address.

	p3h: Turtlebot uses IP: 192.168.2.1

	p3i: Use the roscore on the robot, not a local one (So, via TMule?)

----------------------------------------------------------------------------------------------

Recommence Workshop: Week 2 (6/2/2020 @ 11:56)
----------------------------------------------------------------------------------------------

Prep 4: Setting up the ROS environment (Skip p4a and p4b if 'ros-network.sh' is already there)

	p4a: Use the following bash script..
			__________________________________________________________________________________
			#!bin/bash

			# This script configures the ROS environment variables according to the route
			# to the ROS_MASTER. ROS_MASTER can either be defined as an evironment variable
			# itself or given as first argument to this script. The ROS_IP and ROS_HOSTNAME
			# are set according to the IP that is sitting on the route to this master. 
			# The ROS_MASTER_URI is also set, using port 11311. ROS_MASTER needs to be defined
			# as a numeric IP address, not a hostname.

			if [ "$1" ]; then
				ROS_MASTER="$1"
			fi

			if [ -z "$ROS_MASTER" ]; then
				ROS_MASTER=127.0.0.1
			fi

			echo "ROS_MASTER:     $ROS_MASTER"

			export ROS_IP=`ip route get $ROS_MASTER | grep "src" | sed 's/.*src \([0-9\.]*\).*/\1/'`
			export ROS_HOSTNAME=$ROS_IP
			export ROS_MASTER_URI="http://$ROS_MASTER:11311/"

			echo "ROS_IP:         $ROS_IP"
			echo "ROS_HOSTNAME:   $ROS_HOSTNAME"
			echo "ROS_MASTER_URI: $ROS_MASTER_URI"
			__________________________________________________________________________________
	

	p4b: ALTERNATIVELY use this one line command.. 
			__________________________________________________________________________________
			wget https://raw.githubusercontent.com/marc-hanheide/network-scripts/master/ros-network.sh -P ~/
			__________________________________________________________________________________

			
	p4c: This needs to be pasted into each terminal communicating with the TB2 roscore.
		 This will include those terminals in the workspace.
			__________________________________________________________________________________
			source ~/ros-network.sh 192.168.2.1
			__________________________________________________________________________________
			
			Note: The workspace for the TB2 has now been established. Meaning, we have told ROS to use 
			the 'roscore' for the robot at the address 192.168.2.1 (not the local one).

----------------------------------------------------------------------------------------------
	
Prep 5: Testing topics and visualisation (RViz) of the Turtlbot 2

	p5a: Using 'rostopic list', in the correctly set-up environment, should show the following:
			__________________________________________________________________________________
			/camera/depth/camera_info
			/camera/depth/image_raw
			/camera/depth/points
			/camera/parameter_descriptions
			/camera/parameter_updates
			/camera/rgb/camera_info
			/camera/rgb/image_raw
			/camera/rgb/image_raw/compressed
			/camera/rgb/image_raw/compressed/parameter_descriptions
			/camera/rgb/image_raw/compressed/parameter_updates
			/camera/rgb/image_raw/compressedDepth
			/camera/rgb/image_raw/compressedDepth/parameter_descriptions
			/camera/rgb/image_raw/compressedDepth/parameter_updates
			/camera/rgb/image_raw/theora
			/camera/rgb/image_raw/theora/parameter_descriptions
			/camera/rgb/image_raw/theora/parameter_updates
			/clock
			/cmd_vel_mux/active
			/cmd_vel_mux/input/navi
			/cmd_vel_mux/input/safety_controller
			/cmd_vel_mux/input/switch
			/cmd_vel_mux/input/teleop
			/cmd_vel_mux/parameter_descriptions
			/cmd_vel_mux/parameter_updates
			/depthimage_to_laserscan/parameter_descriptions
			/depthimage_to_laserscan/parameter_updates
			/gazebo/link_states
			/gazebo/model_states
			/gazebo/parameter_descriptions
			/gazebo/parameter_updates
			/gazebo/set_link_state
			/gazebo/set_model_state
			/gazebo_gui/parameter_descriptions
			/gazebo_gui/parameter_updates
			/joint_states
			/laserscan_nodelet_manager/bond
			/mobile_base/commands/motor_power
			/mobile_base/commands/reset_odometry
			/mobile_base/commands/velocity
			/mobile_base/events/bumper
			/mobile_base/events/cliff
			/mobile_base/sensors/bumper_pointcloud
			/mobile_base/sensors/core
			/mobile_base/sensors/imu_data
			/mobile_base_nodelet_manager/bond
			/odom
			/rosout
			/rosout_agg
			/scan
			/tf
			/tf_static
			__________________________________________________________________________________


	p5b: Launch the RViz demo in Turtlebot:
			__________________________________________________________________________________
			'roslaunch uol_turtlebot_simulator turtlebot-rviz.launch'
			__________________________________________________________________________________


	p5c: You can use code to bring up MS Visual Studio.
		 Make the Turtlebot do stuff.

		 Spyder works the same way. Write your script. Hit play.

----------------------------------------------------------------------------------------------	

Task 1: Make your robot move using the command line 
		(both, in simulation and real robot, see above)

	1a. Discuss.. rostopic list.
		Discuss.. rostopic echo /odom.

		Note: Using Rviz to see where the origin of the robot is and then moving it to 
		see if the odometry has been tracked or not is a good visual representation.

	1b. Use this command in the terminal. The robot will rotate.
			__________________________________________________________________________________
			user@hostname$ rostopic pub -1 cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'
			__________________________________________________________________________________

	1c. Do this in the simulation:

	1d. Do this optional one if you want. If we have time.


----------------------------------------------------------------------------------------------	

Task 2: Python programming

	2a. Link to the ROS tutorial for Publishers and Subscribers..
			__________________________________________________________________________________
			http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
			__________________________________________________________________________________

	2b. Refer to the 'rostopic list' for the Turtlebot. There are several commands which
		receive cmd_vel_mux (Mux is a multiplexer. It controls which, well.. control method 
		has priority at an given time. We have no need for that now, but it is just the format
		for velocity commands when using a Turtlbot. It switches between many components.
		Including bumpers, cliff sensors, moving limbs).


		TL:DR;

		Use one of those topics in the code below, either should work.
			__________________________________________________________________________________			
			/cmd_vel_mux/input/navi
			/cmd_vel_mux/input/teleop
			__________________________________________________________________________________
		

		Navi = Autonomous movement/navigation stack
		Teleop = Xbox controllers and using keys to move the TB2.


	2c. Below, is the "First Turtlebot coding"

"""

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
	"""Driving my robot
	"""

	def __init__(self):
		rospy.loginfo("Starting node")
		self.pub = rospy.Publisher("...", Twist) # <-- This is where you put2b !!!!!!
		
	# A function to send velocities until the node is killed
	def send_velocities(self):
		r = rospy.Rate(10) # Setting a rate (hz) at which to publish
		while not rospy.is_shutdown(): # Runnin until killed
			rospy.loginfo("Sending commands")
			twist_msg = Twist() # Creating a new message to send to the robot

			# ... put something relevant into your message

			self.pub.publish(twist_msg) # Sending the message via our publisher
			r.sleep() # Calling sleep to ensure the rate we set above

if __name__ == '__main__':
	rospy.init_node("command_velocity")
	cv = CommandVelocity()
	cv.send_velocities() # Calling the function
	rospy.spin()

"""

	2d. This code example helps fill in the blanks (Marc's lecture code).

"""


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Chatter:

	def __init__(self):
		rospy.init_node('chatter')
		self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

	def laser_cb(self, laser_msg):
		if laser_msg.ranges[320] < 1.0:
			t = Twist()
			t.angular.z = 1.0
			self.publisher.publish(t)
		else:
			t = Twist()
			t.linear.x = 1.0
			self.publisher.publish(t)

	def run(self):
		rospy.spin()

c = Chatter()
c.run()

"""

	2e. The complete code is below.

"""


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy #import the rospy client library
from geometry_msgs.msg import Twist # Import Twist from the geomertry_msgs.msg dependancy/package

class CommandVelocity():
	"""Driving my robot
	"""

	def __init__(self):
		rospy.loginfo("Starting node")
		self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist) 

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


"""

	2f. Do this on the Turtlebot.

	2g. Use VPN or Jupytr. Whichever.


To be marked off for this week, do this:

	A. Do both the tasks

	B. Make the robot move in reality and sim. 
	   Do this from:

	   i. Command line

	   ii. Python script

	   Note: Circle does not have to be a specific size
`	

"""

	
