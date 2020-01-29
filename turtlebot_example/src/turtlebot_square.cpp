//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various
// inputs and outputs.
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

double X;
double Y;
double Yaw;
bool isTurning = false;
double initial_yaw;
double yaw_target;
double initial_x;

//Callback function for the Position topic
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    //This function is called when a new position message is received

    X = msg->pose.pose.position.x;                // Robot X psotition
    Y = msg->pose.pose.position.y;                // Robot Y psotition
    Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
}

int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate
    loop_rate.sleep();       //Maintain the loop rate
    ros::spinOnce();         //Check for new messages
    initial_x = X;
    initial_yaw = Yaw;

    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
        ROS_INFO_NAMED("turtlebot", "X: %f", X);
        ROS_INFO_NAMED("turtlebot", "Initial X: %f", initial_x);
        ROS_INFO_NAMED("turtlebot", "Yaw: %f", Yaw);
        //Main loop code goes here:
        if (isTurning)
        {
            ROS_INFO_NAMED("turtlebot", "Turning");
            vel.linear.x = 0;
            vel.angular.z = 0.2;
            if (yaw_target > M_PI)
            {
                yaw_target -= 2 * M_PI;
            }

            if (Yaw > yaw_target)
            {
                isTurning = false;
                initial_x = X;
            }
        }
        if (!isTurning)
        {
            vel.linear.x = 0.1;
            vel.angular.z = 0;
            ROS_INFO_STREAM("abs(X - initial_x)=" << fabs(X - initial_x));
            if (fabs(X - initial_x) > 0.1)
            {
                isTurning = true;
                initial_yaw = Yaw;
                yaw_target = M_PI / 2 + initial_yaw;
                ROS_INFO_NAMED("turtlebot", "Change to turning");
            }
        }

        //vel.linear.x = 0.2; // set linear speed
        //vel.angular.z = 0.2; // set angular speed

        velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
