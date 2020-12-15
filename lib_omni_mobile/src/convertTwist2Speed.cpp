#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

const float PI = 3.14159265;
const float R = 0.053;
const float L = 0.14256;
geometry_msgs::Twist twist;

void velocityCallback(const geometry_msgs::Twist &msg)
{
    /* Subscribe velocity of robot */
    twist = msg;
}

void convert2Speed(const geometry_msgs::Twist& twist,
        std_msgs::Float64& w1, std_msgs::Float64& w2, std_msgs::Float64& w3)
{
    /* Inverse kinematic */

    float vx = twist.linear.x;
    float vy = twist.linear.y;
    float w = twist.angular.z;

    w1.data = (2*vy + L*w) / R;
    w2.data = (L*w - sqrt(3)*vx - vy) / R;
    w3.data = (L*w + sqrt(3)*vx - vy) / R;
}

int main(int argc, char** argv)
{
     // Init ROS node
    ros::init(argc, argv, "convert");
    ros::NodeHandle nh;

    // Init publisher and subscriber
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, velocityCallback);
    ros::Publisher w1_pub = nh.advertise<std_msgs::Float64>("/librarian/front_wheel_controller/command", 10);
    ros::Publisher w2_pub = nh.advertise<std_msgs::Float64>("/librarian/left_wheel_controller/command", 10);
    ros::Publisher w3_pub = nh.advertise<std_msgs::Float64>("/librarian/right_wheel_controller/command", 10);
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/librarian/joint_states", 10);

    // Create Twist message
    std_msgs::Float64 w1, w2, w3;
    sensor_msgs::JointState joint_states;
    joint_states.position = {0, 0, 0, 0, 0, 0};
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10);
    while(nh.ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        // Update speed each wheel
        convert2Speed(twist, w1, w2, w3);

        // joint_states.header.stamp = ros::Time::now();
        // joint_states.header.frame_id = "base_link";
        // joint_states.name = {(char*)"joint_left_wheel", (char*)"roller_8_left_wheel", (char*)"roller_7_left_wheel", (char*)"roller_6_left_wheel",
        //                     (char*)"roller_5_left_wheel", (char*)"roller_4_left_wheel", (char*)"roller_3_left_wheel", (char*)"roller_2_left_wheel",
        //                     (char*)"roller_1_left_wheel", (char*)"roller_16_left_wheel", (char*)"roller_15_left_wheel", (char*)"roller_14_left_wheel",
        //                     (char*)"roller_13_left_wheel", (char*)"roller_12_left_wheel", (char*)"roller_11_left_wheel", (char*)"roller_10_left_wheel",
        //                     (char*)"roller_9_left_wheel", (char*)"joint_right_wheel", (char*)"roller_8_right_wheel", (char*)"roller_7_right_wheel",
        //                     (char*)"roller_6_right_wheel", (char*)"roller_5_right_wheel", (char*)"roller_4_right_wheel", (char*)"roller_3_right_wheel",
        //                     (char*)"roller_2_right_wheel", (char*)"roller_1_right_wheel", (char*)"roller_16_right_wheel", (char*)"roller_15_right_wheel",
        //                     (char*)"roller_14_right_wheel", (char*)"roller_13_right_wheel", (char*)"roller_12_right_wheel", (char*)"roller_11_right_wheel",
        //                     (char*)"roller_10_right_wheel", (char*)"roller_9_right_wheel", (char*)"joint_front_wheel", (char*)"roller_8_front_wheel",
        //                     (char*)"roller_7_front_wheel", (char*)"roller_6_front_wheel", (char*)"roller_5_front_wheel", (char*)"roller_4_front_wheel",
        //                     (char*)"roller_3_front_wheel", (char*)"roller_2_front_wheel", (char*)"roller_1_front_wheel", (char*)"roller_16_front_wheel",
        //                     (char*)"roller_15_front_wheel", (char*)"roller_14_front_wheel", (char*)"roller_13_front_wheel", (char*)"roller_12_front_wheel",
        //                     (char*)"roller_11_front_wheel", (char*)"roller_10_front_wheel", (char*)"roller_9_front_wheel",
        //                     (char*)"joint_1", (char*)"joint_2", (char*)"joint_3"};
        
        // joint_states.position = {joint_states.position[0] + w1.data * dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //                         joint_states.position[1] + w2.data * dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //                         joint_states.position[2] + w3.data * dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //                         0, 0, 0};
        // joint_states.velocity = {w1.data, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //                         w2.data, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //                         w3.data, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //                         0, 0, 0};

        // joint_states_pub.publish(joint_states);

        // Publish angular velocity of each wheel
        w1_pub.publish(w1);
        w2_pub.publish(w2);
        w3_pub.publish(w3);
        r.sleep();
    }
    return 0;
}