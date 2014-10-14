#!/usr/bin/env python  
import roslib
import rospy
import tf
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
import math
import numpy as np

class Robot():
	"""Base class for Neato robot"""
	def __init__(self,name, pos_type):
		# Initializes Robot instance with robot name and position type (leader, follower), and creates publisher to /cmd_vel
		self.name = name
		self.pos_type = pos_type

		self.pub = rospy.Publisher('/{}/cmd_vel'.format(self.name), Twist, queue_size=10)

	def drive(self, lin_vel, ang_vel):
		"""Publishes specified linear and angular command velocities"""
		# Note: for the Neato platforms, only x-linear and z-rotational motion is possible
		twist = Twist()

		twist.linear = Vector3(lin_vel,0,0)
		twist.angular = Vector3(0,0,ang_vel)
		self.pub.publish(twist.linear, twist.angular)

class Leader(Robot):
	"""Class for leader subset of Neato robots"""
	# This class is empty in this implementation, but is included for future expansion/use as a platform
	def __init__(self, name):
		Robot.__init__(self, name, "leader")

class Follower(Robot):
	"""Class for follower subset of Neato robots"""
	def __init__(self, name):
		Robot.__init__(self, name, "follower")

		self.tf_listener = tf.TransformListener()
		self.initialized = False

		# Initializes last valid translational (x) and rotational (x) differences
		self.valid_rot = 0
		self.valid_trans_x = 0
		self.trans_y = 0

	def has_drifted(self):
		"""Returns True if the robot has drifted more than 0.5m horizontally"""
		if abs(self.trans_y) > 0.5:
			return True
		else:
			return False

	def correct_drift(self):
		"""Placeholder code for what would actually move robot to re-align stuff"""
		print("HELP -- I'VE DRIFTED OFF COURSE! \n PLEASE CORRECT ME")

	def scrub_inputs(self, trans, rot):
		"""Removes invalid translational or rotational differences from consideration"""
		trans_x = -0.9 - self.trans[0]
		trans_y = trans[1]
		rot_z = np.degrees(rot[2])

		if self.initialized:
		# If newest translational and rotational difference values are within specified error, assume valid
			if abs(trans_x - self.valid_trans_x) < 0.3:
				self.valid_trans_x = trans_x
			if abs(rot_z - self.valid_rot) < 1.0:
				self.valid_rot = rot_z
		else:
			self.valid_trans_x = trans_x
			self.valid_rot = rot_z
			self.initialized = True

	def listen_to_transform(self):
		print("{}.tf_listener initiated".format(self.name))

		# Looks up coordinate frame transforms between /world /base_link frames
		(trans_f, rot_f) = self.tf_listener.lookupTransform('/world', '/{}/base_link'.format(self.name), rospy.Time(0))
		(trans_l, rot_l) = self.tf_listener.lookupTransform('/world', '/leader/base_link', rospy.Time(0))

		angles_f = np.array(euler_from_quaternion(rot_f))
		angles_l = np.array(euler_from_quaternion(rot_l))

		# Casts to numpy array for ease of working
		trans_f = np.array(trans_f)
		trans_l = np.array(trans_l)

		trans = trans_f - trans_l
		angles = angles_f - angles_l

		self.scrub_inputs(trans, angles)
		self.follow_the_leader()

	def follow_the_leader(self):
		"""Implements leader-following behavior by sending appropriate command velocities to follower robot"""
		twist = Twist()
		ang_vel = 0
		lin_vel = 0

		# If robot has drifted off course, correct drift before doing anything else
		if self.has_drifted():
			self.correct_drift()

		# Rotational re-alignment takes precedence if robots are more than 2 degrees apart
		if abs(self.valid_rot) > 2:
			print("{} is rotating".format(self.name))
			ang_vel = -0.01*self.valid_rot	
		
		# If follower is within 0.9m of leader robot, set translational difference to 0
		if 1 - abs(self.valid_trans_x) < 0.1: 
			self.valid_trans_x = 0

		# Sets linear velocity proportional to transational difference between robots
		lin_vel = 0.5*self.valid_trans_x
		self.drive(lin_vel, ang_vel)


if __name__ == '__main__':
	rospy.init_node('test_horde')
	# Instantiates one instance of Robot class for each Neato
	leader = Leader('leader', 'leader-type')
	follower = Follower('follower', 'follower-type')

	rate = rospy.Rate(5.0)
	while not rospy.is_shutdown():
		try:
			follower.listen_to_transform()
			rate.sleep()
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		finally:	
			follower.drive(0,0)

	rospy.spin() 







