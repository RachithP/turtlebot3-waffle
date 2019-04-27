#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
import numpy as np

class turtlebot3_controller():

	def __init__(self):

		self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		self.move = Twist()

		# Define the rate at which messages are published. This does not matter for this simulation as we are not considering it for publishing purposes.
		self.rate = rospy.Rate(100)

	def publish(self, params, path):

		print('length of params:', len(params))
		print('length of path:', len(path))

		rospy.sleep(1)	# wait for sometime for observer to get adjusted to the scene

		for param in params:

			#convert param into appropriate format
			param = np.squeeze(np.array(param))

			# linear velocity in the x-axis, this x-axis is the axis of the robot in the forward direction.
			self.move.linear.x = param[0]/100.0 # converting cm/s to m/s

			# angular velocity in the z-axis around the robot
			self.move.angular.z = param[1]

			print('linear velocity:', self.move.linear.x)
			print('angular velocity:', self.move.angular.z)
			print('-------------------------------------------')

			# publish command
			endTime = rospy.Time.now() + rospy.Duration(1.02)

			while rospy.Time.now() < endTime:
				self.velocity_pub.publish(self.move)

			self.rate.sleep()