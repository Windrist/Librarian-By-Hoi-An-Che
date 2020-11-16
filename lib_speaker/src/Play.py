#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
import subprocess

class Play(object):
	def callback(self, msg):
		print("Playing: " + msg.data)
		cmd = "mpg123 -q " + self.file + "/../file/{}".format(msg.data)
		subprocess.run(cmd, shell=True)
		self.main.data = "Done"
		self.pub_state.publish(self.main)
	
	def __init__(self):

		rospy.init_node('Play', anonymous=True)
		self.pub_state = rospy.Publisher("/Main_state", String, queue_size=100)
		rospy.Subscriber("/Play_file", String, self.callback)
		self.rate = rospy.Rate(10)

		self.file = os.path.dirname(__file__)

		self.main = String()

	def spin(self):
		while not rospy.is_shutdown():
			self.rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	Play = Play()
	Play.spin()
