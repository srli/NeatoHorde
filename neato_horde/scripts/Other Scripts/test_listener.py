#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf_neato')
    print('TEST_LISTENER RUNNING')

    listener = tf.TransformListener()

    follower_vel = rospy.Publisher('bigbird/cmd_vel', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/follower/base_link', rospy.Time(0))
            # (trans2, rot2) = listener.lookupTransform('/bigbird/odom', '/oscar/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print('-------------------------------------------------------------')
        print('TRANSLATION', trans)
        # print('TRANSLATION ODOM', trans2)
        print('ROTATION:', rot)
        # print('ROTATION ODOM:', rot2)
        print('-------------------------------------------------------------')

        # x_trans = trans[0]
        # if x_trans > 0:
        #     linear = 0.2*x_trans
        # else:
        #     linear = -0.2*x_trans
        # angular = 0 #0.2 * math.atan2(trans[1], trans[0])
        # #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # follower_vel.publish(cmd)

        rate.sleep()
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0
    follower_vel.publish(cmd)

