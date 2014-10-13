#!/usr/bin/env python  
import roslib
import rospy
import tf
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist
import math

class Robot:
	"""Neato robot class -- [insert better comment here]"""
	def __init__(self, name):
		self.name = name

		self.tf_broadcaster = tf.TransformBroadcaster()
		self.tf_listener = tf.TransformListener()


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

def listener():
	listener = tf.TransformListener()

	rate = rospy.Rate(5.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/world', '/follower/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print('-------------------------------------------------------------')
		print('TRANSLATION', trans)
		print('ROTATION:', rot)
		print('-------------------------------------------------------------')

		rate.sleep()


if __name__ == '__main__':
	# Instantiates one instance of Robot class for each Neato
	leader = Robot('leader')
	follower = Robot('follower')

	rospy.init_node('test_broadcaster')
	print('TEST RUNNING')
	rospy.Subscriber('follower/odom', Odometry, follower.broadcast_pose)

	# Because we want to continually be listening, even when we aren't moving 	
	listener()
	# follower.listen_to_pose()


	rospy.spin() 







