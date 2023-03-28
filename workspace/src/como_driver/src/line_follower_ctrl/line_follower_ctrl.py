#!/usr/bin/env python
'''
we subscribe to the steering angle topic /ecu/line_follow/str and publish to /ecu a steering and a velocity command.

This script also serves as a sample script for what other COMO higher-level control scripts can look like.

Author:
Sleiman Safaoui

Github:
The-SS

Email:
snsafaoui@gmail.com

Date:
July 30, 2018
'''

import roslib
import rospy
from barc.msg import ECU, Encoder
from image_processing.msg import LineData
from numpy import pi
from std_msgs.msg import Float64


class StrSub:
	'''
	Subscribes to the steering angle topic
	'''
	def __init__(self):
		self.str_cmd = 0.0
		self.str_sub = rospy.Subscriber("/ecu/line_follower/servo", Float64, self.callback, queue_size =1)
		
	def callback(self, data):
		self.str_cmd = data.data
		
	def get_str(self):
		return self.str_cmd

class ECUPub:
	'''
	Publishes an ECU command to /ecu topic
	'''
	def __init__(self):
		self.ecu = ECU(0.,0.)
		self.ecu_pub = rospy.Publisher('/ecu', ECU, queue_size = 1)
		
	def set_ecu(self, motor, servo):
		self.ecu = ECU(float(motor), float(servo)*pi/180.0)
	
	def publish_ecu(self):
		self.ecu_pub.publish(self.ecu)
		
def main():
	rospy.init_node("line_follower_ctrl") # initialize ROS node
	rate = rospy.Rate(30)
	str_sub = StrSub()
	ecu_pub = ECUPub()
	
	motor_cmd = 4.5
	while not rospy.is_shutdown():
		str_cmd = str_sub.get_str() # get steering command
		ecu_pub.set_ecu(motor_cmd, str_cmd) # update publisher with new command
		ecu_pub.publish_ecu() # publish new command
		
		rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down line_follower_ctrl node")
		pass
		
