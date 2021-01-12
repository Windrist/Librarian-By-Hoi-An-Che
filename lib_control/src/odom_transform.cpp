#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

// Init Global Variable
geometry_msgs::Twist twist;
double x = 0, y = 0, theta = 0;

void velocityCallback(const geometry_msgs::Twist &msg)
{
    /* Subscribe Velocity of robot */
    twist = msg;
    // Calibrate Velocity
    twist.linear.x = twist.linear.x * 2.2;
    twist.angular.z = twist.angular.z;
}

int main(int argc, char** argv)
{
    // Init ROS
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 10, velocityCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate r(10.0);

    // Init Time to Calculate Odometry
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(n.ok()){

        ros::spinOnce();
        current_time = ros::Time::now();

        // Compute Odometry in a typical way given the Velocities of the robot
        double dt = (current_time - last_time).toSec();
        x += (twist.linear.x * cos(theta) - twist.linear.y * sin(theta)) * dt;
        y += (twist.linear.x * sin(theta) + twist.linear.y * cos(theta)) * dt;
        theta += twist.angular.z * dt;

        // Since all Odometry is 6DOF we'll need a Quaternion created from Yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // First, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // Send the transform
        odom_broadcaster.sendTransform(odom_trans);
        
        // Next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        // Set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // Set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = twist.linear.x;
        odom.twist.twist.linear.y = twist.linear.y;
        odom.twist.twist.angular.z = twist.angular.z;
        odom.twist.covariance[0] = 1e-4;
        odom.twist.covariance[7] = 1e-4;
        odom.twist.covariance[35] = 1e-4;

        // Publish the message
        odom_pub.publish(odom);

        // Reformat Time
        last_time = current_time;
        // last_theta = theta;
        r.sleep();
    }
}