#!/usr/bin/env python

import rospy
import random as rnd
import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    # This function is called when a new position message is received
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    (roll,pitch,yaw) = euler_from_quaternion(msg.pose.pose.orientation)
    rospy.loginfo( "X: " + x + ", Y: " + y + ", Yaw: " + yaw )

def random_path():
 
    # initialize a subscriber to receive the estimated pose
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)

    # initialize a publisher for the message
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

    # initialize the ROS node 
    rospy.init_node('random_path', anonymous=True)
    rate = rospy.Rate(1) # 1 hz

    # while ROS is still running
    while not rospy.is_shutdown():

        msg = Twist();

        # movement forward (X-axis only)
        msg.linear.x = rnd.uniform( 0, 0.3 )
        msg.linear.y = 0
        msg.linear.z = 0

        # and some rotation
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = rnd.uniform( -0.5, 0.5 )

        ## Uncomment this line to write the message to the command-line
	# rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        random_path()
    except rospy.ROSInterruptException:
        pass

