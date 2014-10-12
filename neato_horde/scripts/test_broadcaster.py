#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry
import inspect

def handle_turtle_pose(msg, neato_name):
    #print 'hello'
    # br = tf.TransformBroadcaster()
    # br.sendTransform(msg.pose,
    #                  tf.transformations.quaternion_from_euler(msg.pose),
    #                  rospy.Time.now(),
    #                  neato_name,
    #                  "world")
    offset = 0
    if neato_name == 'bigbird':
        offset = 3
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     tf.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.z),
                     rospy.Time.now(),
                     '{}/odom'.format(neato_name),
                     "world")
    


if __name__ == '__main__':
    neatos = ['oscar', 'bigbird']
    rospy.init_node('turtle_tf_broadcaster')
    print('TEST_BROADCASTER RUNNING')
    #turtlename = rospy.get_param('~turtle')
    for neato in neatos:
        #print '/%s/odom' % neato
        rospy.Subscriber('/%s/odom' % neato,
                         Odometry,
                         handle_turtle_pose, neato)
    rospy.spin()