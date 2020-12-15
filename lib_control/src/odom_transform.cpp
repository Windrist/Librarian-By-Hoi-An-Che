#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

geometry_msgs::Twist twist;
double x = 0, y = 0, theta = 0;

void velocityCallback(const geometry_msgs::Twist &msg)
{
    /* Subscribe velocity of robot */
    twist = msg;
    twist.linear.x = twist.linear.x * 2.2;
    twist.angular.z = twist.angular.z;
}

// void imuCallback(const sensor_msgs::Imu &msg)
// {
//     /* Subscribe orientation of robot */
//     theta = tf::getYaw(msg.orientation);
// }

// void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
// {
//     /* Subscribe init pose of robot */
//     x = msg.pose.pose.position.x;
//     y = msg.pose.pose.position.y;
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 10, velocityCallback);
    // ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
    // ros::Subscriber pos_sub = n.subscribe("/initialpose", 10, poseCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
 
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(10.0);

    double last_theta = 0.0;

    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        // double dt = (current_time - last_time).toSec();
        double dt = 0.1;
        x += (twist.linear.x * cos(theta) - twist.linear.y * sin(theta)) * dt;
        y += (twist.linear.x * sin(theta) + twist.linear.y * cos(theta)) * dt;
        double delta_theta = twist.angular.z * dt;
        // double delta_theta = theta - last_theta;

        // x += twist.linear.x * dt * cos(theta + (delta_theta / 2.0));
        // y += twist.linear.x * dt * sin(theta + (delta_theta / 2.0));
        theta += delta_theta;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
        
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = twist.linear.x;
        odom.twist.twist.linear.y = twist.linear.y;
        odom.twist.twist.angular.z = twist.angular.z;
        odom.twist.covariance[0] = 1e-4;
        odom.twist.covariance[7] = 1e-4;
        odom.twist.covariance[35] = 1e-4;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        last_theta = theta;
        r.sleep();
    }
}