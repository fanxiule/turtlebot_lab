#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from math import sin
from math import cos
from math import sqrt

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from ellipse import EllipPublisher

lin_vel = 0.0  # linear control input
rot_vel = 0.0  # rotational control input

x = 0.0  # x from odometry
y = 0.0  # y from odometry
yaw = 0.0  # yaw from odometry

# previous state estimation
x_upd_pre = np.array([[0], [0], [0]], dtype='float')
# previous covariance estimation
err_upd_pre = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], dtype='float')
iter = 0  # iteration of the EKF

sample_time = 1/30


def ekf(lin_vel, rot_vel, x, y, yaw):
    global x_upd_pre
    global err_upd_pre
    global sample_time
    global iter

    sensor_var = np.array([[0.05], [0.05], [0.05]])  # variance of odometry
    Q = np.array([[sensor_var[0], 0, 0], [
                 0, sensor_var[1], 0], [0, 0, sensor_var[2]]], dtype='float')  # Q, sensor covariance
    # R, motion model covariance
    R = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
    G_lin = np.array([[1, 0, -lin_vel*sin(x_upd_pre[2, 0])],
                      [0, 1, lin_vel*cos(x_upd_pre[2, 0])], [0, 0, 1]])  # linearized motion model
    # rospy.loginfo("Sensor: " + str(sensor_var) + " Q: " + str(Q) +
    #              " R: " + str(R) + " G_linearized: " + str(G_lin))

    if iter == 0:  # in the first iteration, use guess
        x_pred = np.array([[0], [0], [1]], dtype='float')  # initial guess
        err_pred = np.array([[0.05, 0, 0], [0, 0.05, 0], [
                            0, 0, 0.05]], dtype='float')
    else:
        x_pred = np.array([[x_upd_pre[0, 0]+lin_vel*cos(x_upd_pre[2, 0])*sample_time],
                           [x_upd_pre[1, 0]+lin_vel *
                               sin(x_upd_pre[2, 0])*sample_time],
                           [x_upd_pre[2, 0]+rot_vel*sample_time]])  # state predition
        err_pred = np.dot(np.dot(G_lin, err_upd_pre),
                          np.transpose(G_lin)) + R  # error prediction

    # rospy.loginfo("x_pred: " + str(x_pred) + " err_pred" + str(err_pred))
    H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                 dtype='float')  # sensor model
    K = np.dot(np.dot(err_pred, np.transpose(H)), np.linalg.inv(
        np.dot(np.dot(H, err_pred), np.transpose(H))+Q))  # Kalman gain
    mea = np.array([[x], [y], [yaw]])
    mea_pred = x_pred  # since measurement is odometry, measurement based on prediction is essentially the state prediction
    # rospy.loginfo("H: " + str(H) + " K" + str(K) + " mea: " + str(mea))
    x_upd = x_pred+np.dot(K, (mea-x_pred))
    err_upd = np.dot((np.eye(3) - np.dot(K, H)), err_pred)
    # rospy.loginfo("x_upd: " + str(x_upd) + " err_upd: " + str(err_upd))
    x_upd_pre = x_upd
    err_upd_pre = err_upd
    iter = iter+1
    return x_upd, err_upd


def getPoseFromIPS(args):
    pose_msg = PoseStamped()
    pose_msg = args

    x = pose_msg.pose.position.x
    y = pose_msg.pose.position.y
    explicit_quat = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                     pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)
    # rospy.loginfo("X: " + str(x) + ", Y: " + str(y) + ", Yaw: " + str(yaw))

# callback for odometry reading


def odom_callback(msg):
    global x
    global y
    global yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)


def teleop_callback(msg):
    global lin_vel
    global rot_vel
    lin_vel = msg.linear.x
    rot_vel = msg.angular.z


def pose_callback(msg, args):
    # This function is called when a new position message is received

    # You'll probably need to check which pose in gazebo represents the robot and not some
    # other (static) object
    # for i in range(len(msg.pose)):
    #    print("Pose {}: {},{},{}".format(i, msg.pose[i].position.x,
    #                                     msg.pose[i].position.y, msg.pose[i].position.z))

    # republish pose for rviz
    publisher = args
    curpose = PoseStamped()
    curpose.pose = msg.pose[1]
    curpose.header.frame_id = "/map"
    # curpose.header.frame_id = "/base_link"
    getPoseFromIPS(curpose)
    publisher.publish(curpose)


def main():

    # initialize the ROS model
    rospy.init_node('extended_KF', anonymous=True)

    pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=1)
    ellip_pub = EllipPublisher()
    # initialize a subscriber to receive the estimated pose and the map
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.Subscriber("/gazebo/model_states", ModelStates,
                     pose_callback, pose_pub)

    rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, teleop_callback)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        x_upd, err_upd = ekf(lin_vel, rot_vel, x, y, yaw)
        e_val, e_vec = np.linalg.eig(err_upd)
        rospy.loginfo("Estimated X: " + str(x_upd[0])+" Estimated Y: " + str(
            x_upd[1])+" Estimated Yaw: " + str(x_upd[2]))
        ellip_pub.publish(e_val[0], e_val[1])
        rate.sleep()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
