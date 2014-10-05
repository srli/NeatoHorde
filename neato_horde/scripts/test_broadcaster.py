#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_turtle_pose(msg, neato_name):
    #print 'hello'
    br = tf.TransformBroadcaster()
    br.sendTransform(msg.pose,
                     tf.transformations.quaternion_from_euler(msg.pose),
                     rospy.Time.now(),
                     neato_name,
                     "world")
    # br = tf.TransformBroadcaster()
    # br.sendTransform((msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
    #                  tf.transformations.quaternion_from_euler(0, 0, msg.twist.angular.z),
    #                  rospy.Time.now(),
    #                  neato_name,
    #                  "world")
    # print "sent!"
    i = msg
    print i



if __name__ == '__main__':
    neatos = ['oscar', 'bigbird']
    rospy.init_node('turtle_tf_broadcaster')
    #turtlename = rospy.get_param('~turtle')
    for neato in neatos:
        #print '/%s/odom' % neato
        rospy.Subscriber('/%s/odom' % neato,
                         Odometry,
                         handle_turtle_pose, neato)
    rospy.spin()