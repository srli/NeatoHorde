#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import Pose

def handle_neato_pose(msg, neato_name):
	"""Helper method to handle the neato robot's pose"""
	br = tf.TransformBroadcaster()
	br.sendTransform((msg.x, msg.y, 0),
						tf.transformations.quaternion_from_euler(0.0, 0.0, msg.theta),
						rospy.Time.now(),
						neato_name,
						"world")

if __name__ == '__main__':
	# Initiates as 'neato_tf_broadcaster' node
	rospy.init_node('neato_tf_broadcaster')
	# Note: just set for one robot right now (should switch to param)
	neato_name = 'bigbird'
	sub = rospy.Subscriber('/{}/pose'.format(neato_name), Pose, handle_neato_pose, neato_name)

	rospy.spin()

