#!/usr/bin/env python  
import roslib
import rospy
import tf
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist
from tf.transformations import euler_from_quaternion
import math
import numpy as np

class Robot:
	"""Neato robot class -- [insert better comment here]"""
	def __init__(self, name):
		self.name = name

		self.tf_broadcaster = tf.TransformBroadcaster()
		self.tf_listener = tf.TransformListener()
		self.initialized = False
		self.prev_translation = [0,0,0]
		self.prev_rotation = [0,0,0]

	def update_world_loc(self, msg):
		"""Returns current robot location [x,y] with respect to world frame"""
		# Note: currently not accounting for offset of odom -> world static transform
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		# print(self.name)
		# print('x: ', self.x)
		# print('y: ', self.y)
		# print('---------------------------------------------------')

	def update_world_rot(self, msg):
		"""Returns robot rotational orientation [] with respect to world frame"""

	def broadcast_pose(self, msg):
		"""Broadcasts the transform between /world and /base_link"""

		self.update_world_loc(msg)
		self.update_world_rot(msg)


		############################################################################################################################
		# I suspect that this broadcaster isn't actually necessary, since we declare the odom -> world transforms in the launch -- but who knows
		############################################################################################################################
		# self.tf_broadcaster.sendTransform((self.x, self.y, msg.pose.pose.position.z),
  #                    tf.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.z),
  #                    rospy.Time.now(),
  #                    '{}/odom'.format(self.name),
  #                    '/world') 

	def listen_to_pose(self):
		"""Listens to transform between /world and /base_link"""


		# This is currently yielding odom -> base_link conversion (regardless of what I set frame_id to...)
		p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='base_link'), pose=Pose())
		# temp_pose = self.tf_listener.transformPose('odom',p)

		# temp = self.tf_listener.lookupTransform('/world', '/follower/base_link', rospy.Time(0))

		print p

		# print("=======POSE STAMPED=====")
		# print(p)
		# print("TRANSFORMED POSE TO ODOM?")
		# print temp_pose
		print("==============================")

	def correctdrift(self, translation_y):
		print "CORRECT Y AXIS DRIFT PLEASE"

	def scrubinputs(self, trans, rot):
		translation_x = -0.9 - trans[0]
		translation_y = trans[1]
		rotation_z = np.degrees(rot[2])
		print "translation x", translation_x
		print "rotation Z", rotation_z

		if self.initialized:
			if abs(translation_x - self.prev_translation[0]) < 0.3:
				self.prev_translation[0] = translation_x
			if abs(int(rotation_z - self.prev_rotation[2])) < 1:
				self.prev_rotation[2] = rotation_z
			if abs(translation_y) > 0.5:
				self.correctdrift(translation_y)
		else:
			self.prev_translation[0] = translation_x
			self.prev_rotation[2] = rotation_z
			self.initialized = True
		self.prev_rotation[2] = rotation_z

	def follow(self):
		"""Sends a cmd_vel to follower robots so they'll follow leader robot"""
		angular_z = 0
		twist = Twist()
		print ("Translation distance: ", self.prev_translation[0])
		print ("Testing crap: ", 1 - abs(self.prev_translation[0]))
		if abs(self.prev_rotation[2]) > 2: #rotation takes priority over translation
			print "rotating"
			#self.prev_translation[0] = 0
			angular_z = self.prev_rotation[2]
		if 1 - abs(self.prev_translation[0]) < 0.1: #so we're operating over a range
			self.prev_translation[0] = 0
		
		print "TRANSLATE MOVE: ", 0.5*self.prev_translation[0]
		print "ANGULAR MOVE: ", 0.2*angular_z
		twist.linear.x = 0.5*self.prev_translation[0]; twist.linear.y = 0; twist.linear.z = 0 #basically we move on 2 axis, x-linear, z-angular
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -0.01*angular_z
		pub.publish(twist)


def listener():
	listener = tf.TransformListener()
	print "tf listener initiated"
	rate = rospy.Rate(5.0)
	while not rospy.is_shutdown():
		try:
			(trans_f,rot_f) = listener.lookupTransform('/world', '/follower/base_link', rospy.Time(0))
			(trans_l,rot_l) = listener.lookupTransform('/world', '/leader/base_link', rospy.Time(0))
			#(trans,rot) = listener.lookupTransform('/leader/base_link', '/follower/base_link', rospy.Time(0))
			angles_f = np.array(euler_from_quaternion(rot_f))
			angles_l = np.array(euler_from_quaternion(rot_l))
        	
        	# Casts to numpy array for easier working with
			trans_f = np.array(trans_f)
			trans_l = np.array(trans_l)

			trans = trans_f - trans_l
			angles = angles_f - angles_l

			print('-------------------------------------------------------------')
			print('TRANSLATION', trans)
			print 'ROTATION:',[(180.0/math.pi)*i for i in angles]
			print('-------------------------------------------------------------')

			follower.scrubinputs(trans, angles)
			follower.follow()
			rate.sleep()

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		finally:
			twist = Twist()
			twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
			pub.publish(twist)


if __name__ == '__main__':
	# Instantiates one instance of Robot class for each Neato
	leader = Robot('leader')
	follower = Robot('follower')
	rospy.init_node('test_broadcaster')
	print('TEST RUNNING')
	# rospy.Subscriber('follower/odom', Odometry, follower.broadcast_pose)
	# rospy.Subscriber('leader/odom', Odometry, leader.broadcast_pose)
	pub = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=10)
	# Because we want to continually be listening, even when we aren't moving 	
	listener()
	# follower.listen_to_pose()
	rospy.spin() 







