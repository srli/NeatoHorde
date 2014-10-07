#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_neato')

    listener = tf.TransformListener()

    follower_vel = rospy.Publisher('bigbird/cmd', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/bigbird', '/oscar', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print('Translation', trans)
        print('Rotational:', rot)


        x_trans = trans[0]
        if x_trans > 0:
            linear = 0.2*x_trans
        else:
            linear = -0.2*x_trans
        angular = 0 #0.2 * math.atan2(trans[1], trans[0])
        #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        follower_vel.publish(cmd)

        rate.sleep()
    cmd.linear.x = 0
    cmd.angular.z = 0
    follower_vel.publish(cmd)

