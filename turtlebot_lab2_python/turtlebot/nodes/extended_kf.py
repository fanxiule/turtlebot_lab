#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt

from math import sin
from math import cos
from math import sqrt

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

from ellipse import EllipPublisher

lin_vel = 0.0  # linear control input
rot_vel = 0.0  # rotational control input

x = 0.0  # ground true x
y = 0.0  # ground true y
yaw = 0.0  # ground true yaw

x_mea = 0.0  # x corrupted by noise
y_mea = 0.0  # y corrupted by noise
yaw_mea = 0.0  # yaw corruputed by noise

x_var = 0.01  # variance for x measurement from IPS
y_var = 0.01  # variance for y measurement from IPS
yaw_var = 0.01  # variance for yaw measurement from IPS

# previous state estimation
x_upd_pre = np.array([[0], [0], [0]], dtype='float')
# previous covariance estimation
err_upd_pre = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], dtype='float')

iter = 0  # iteration of the EKF
sample_time = 1  # sample_time
real_path = Path()  # real trajectory
ekf_path = Path()  # trajectory generated by ekf
mea_path = Path()  # trajectory based on corrupted measurments


def ekf(lin_vel, rot_vel, x, y, yaw):  # ekf algorithm
    global x_upd_pre
    global err_upd_pre
    global sample_time
    global iter
    global x_var
    global y_var
    global yaw_var

    # R, motion model covariance
    R = np.array([[x_var, 0, 0], [0, y_var, 0], [0, 0, yaw_var]])
    # Q, measurement model covariance
    Q = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.0]])
    G_lin = np.array([[1, 0, -lin_vel*sin(x_upd_pre[2, 0])],
                      [0, 1, lin_vel*cos(x_upd_pre[2, 0])], [0, 0, 1]])  # linearized motion model
    # rospy.loginfo("Sensor: " + str(sensor_var) + " Q: " + str(Q) +
    #              " R: " + str(R) + " G_linearized: " + str(G_lin))

    if iter == 0:  # first iteration
        # initial belief of state
        x_upd = np.array([[6.2], [-2.5], [0]], dtype='float')
        # initial belief of error covariance
        err_upd = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]])
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
        # sensor measurement from IPS
        mea = np.array([[x_mea], [y_mea], [yaw_mea]])
        # rospy.loginfo("H: " + str(H) + " K" + str(K) + " mea: " + str(mea))
        x_upd = x_pred+np.dot(K, (mea-x_pred))  # state update
        # error covariance update
        err_upd = np.dot((np.eye(3) - np.dot(K, H)), err_pred)
        # rospy.loginfo("x_upd: " + str(x_upd) + " err_upd: " + str(err_upd))
    x_upd_pre = x_upd
    err_upd_pre = err_upd
    iter = iter+1
    return x_upd, err_upd


def IPS_callback(msg, pubs):
    global x
    global y
    global yaw
    global x_mea
    global y_mea
    global yaw_mea
    global x_var
    global y_var
    global yaw_var
    global real_path
    global mea_path

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    # convert orientation from quaternion to euler
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)
    # rospy.loginfo("Ground True X: " + str(x) + ", Y: " +
    #               str(y) + ", Yaw: " + str(yaw))

    # corrupt true states with noise
    x_mea = x + np.random.normal(0, sqrt(x_var))
    y_mea = y + np.random.normal(0, sqrt(y_var))
    yaw_mea = yaw + np.random.normal(0, sqrt(yaw_var))

    # broadcast a global frame /world, origin of this frame is the origin of IPS frame
    tf_broadcaster = pubs[0]
    tf_broadcaster.sendTransform((x, y, 0),
                                 explicit_quat,
                                 rospy.Time.now(),
                                 "base_link",
                                 "world")
    # rospy.loginfo("Measurement X: " + str(x_mea) + ", Y: " +
    #               str(y_mea) + ", Yaw: " + str(yaw_mea))

    # publish the real path of the robot
    path_publisher = pubs[1]
    curpose = PoseStamped()
    curpose.pose = msg.pose.pose
    curpose.header.frame_id = "/world"
    real_path.poses.append(curpose)
    real_path.header.frame_id = "/world"
    path_publisher.publish(real_path)

    # publish the path based on corrupted measurement only
    mea_publisher = pubs[2]
    curpose = PoseStamped()
    curpose.pose.position.x = x_mea
    curpose.pose.position.y = y_mea
    curpose.header.frame_id = "/world"
    mea_path.poses.append(curpose)
    mea_path.header.frame_id = "/world"
    mea_publisher.publish(mea_path)


def teleop_callback(msg):  # obtain control action
    global lin_vel
    global rot_vel
    lin_vel = msg.linear.x
    rot_vel = msg.angular.z


def main():
    # initialize the ROS model
    rospy.init_node('extended_KF', anonymous=True)

    # pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=1)
    frame_br = tf.TransformBroadcaster()  # broadcaster for a global /world frame
    # publisher for real robot path
    true_path_pub = rospy.Publisher("/robot_path", Path, queue_size=1)
    # publisher for path based on corrupted measurements
    mea_path_pub = rospy.Publisher("/mea_path", Path, queue_size=1)
    ekf_path_pub = rospy.Publisher(
        "/ekf_path", Path, queue_size=1)  # publisher for ekf path
    publishers = [frame_br, true_path_pub, mea_path_pub]
    rospy.Subscriber("/indoor_pos", PoseWithCovarianceStamped,
                     IPS_callback, publishers)  # subscriber to obtain IPS data
    ellip_pub = EllipPublisher()  # publisher for ekf covariance ellipse
    # subscriber to obtain control action
    rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, teleop_callback)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            x_upd, err_upd = ekf(lin_vel, rot_vel, x_mea, y_mea, yaw_mea)
            # calculate eigenvalues and eigenvectors of the error
            e_val, e_vec = np.linalg.eig(err_upd)
            rospy.loginfo("Estimated X: " + str(x_upd[0])+" Estimated Y: " + str(
                x_upd[1])+" Estimated Yaw: " + str(x_upd[2]))
            # publish covariance ellipse
            ellip_pub.publish(x_upd[0], x_upd[1], e_val[0], e_val[1])

            # publish trajecotry generated by EKF
            ekf_pose = PoseStamped()
            ekf_pose.pose.position.x = x_upd[0]
            ekf_pose.pose.position.y = x_upd[1]
            ekf_pose.header.frame_id = "/world"
            ekf_path.poses.append(ekf_pose)
            ekf_path.header.frame_id = "/world"
            ekf_path_pub.publish(ekf_path)

            # plot true path and ekf path in a figure
            fig1 = plt.figure(1)
            plt.plot(x, y, 'yo')
            plt.plot(x_upd[0, 0], x_upd[1, 0], 'ro')
            plt.plot(x_mea, y_mea, 'bo')
            plt.legend(['Ground Truth', 'EKF', 'Measurements with Noise'])
            plt.title('Trajectories from IPS, EKF and Corrupted Measurements')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.grid()
            fig1.canvas.draw_idle()

            # plot difference between two paths over time
            # difference between ekf and true
            error_ekf = sqrt((x-x_upd[0, 0])**2+(y-x_upd[1, 0])**2)
            # difference between measurements and true
            error_mea = sqrt((x-x_mea)**2+(y-y_mea)**2)
            fig2 = plt.figure(2)
            plt.plot(iter, error_ekf, 'ro')
            plt.plot(iter, error_mea, 'bo')
            plt.title('Distance Between Two Trajectories over Time')
            plt.legend(['EKF vs True', 'Measurements vs True'])
            plt.xlabel('Time Intervals')
            plt.ylabel('Difference (m)')
            plt.grid()
            fig2.canvas.draw_idle()

            # plot difference between two errors
            error_diff = error_mea - error_ekf
            fig3 = plt.figure(3)
            plt.plot(iter, error_diff, 'ro')
            plt.title("Improvement of State Estimates with EKF")
            plt.xlabel("Time Intervals")
            plt.ylabel("Difference (m)")
            plt.grid()
            fig3.canvas.draw_idle()
            plt.pause(0.001)
            rate.sleep()

        except rospy.exceptions.ROSInternalException:
            rospy.logerr("ROS interrupt exception")
        except rospy.exceptions.ROSTimeMovedBackwardsException:  # sometimes occur while using rosbag
            rospy.logerr("ROS time moves backwards ")


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
