#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

const float PI = 3.14159265;
const float R = 0.05;
const float L = 0.15;
geometry_msgs::Twist twist;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    /* Subscribe velocity of robot */
    twist = *msg;
}

void convert2Speed(const geometry_msgs::Twist& twist,
        std_msgs::Float32& w1, std_msgs::Float32& w2, std_msgs::Float32& w3)
{
    /* Inverse kinematic */

    float vx = twist.linear.x;
    float vy = twist.linear.y;
    float w = twist.angular.z;

    w1.data = (-L*w/R) * 16;
    w2.data = (20*sqrt(3)*vx - L*w/R) * 16;
    w3.data = (-L*w/R - 20*sqrt(3)*vx) * 16;
}

int main(int argc, char** argv)
{
     // Init ROS node
    ros::init(argc, argv, "convert");
    ros::NodeHandle nh;

    // Init publisher and subscriber
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, velocityCallback);
    ros::Publisher w1_pub = nh.advertise<std_msgs::Float32>("/librarian/front_wheel_controller/command", 10);
    ros::Publisher w2_pub = nh.advertise<std_msgs::Float32>("/librarian/left_wheel_controller/command", 10);
    ros::Publisher w3_pub = nh.advertise<std_msgs::Float32>("/librarian/right_wheel_controller/command", 10);

    // Create Twist message
    std_msgs::Float32 w1, w2, w3;

    ros::Rate r(10);
    while(nh.ok())
    {
        // Update speed each wheel
        convert2Speed(twist, w1, w2, w3);

        // Publish angular velocity of each wheel
        w1_pub.publish(w1);
        w2_pub.publish(w2);
        w3_pub.publish(w3);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}