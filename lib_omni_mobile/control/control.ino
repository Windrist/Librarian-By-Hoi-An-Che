// ROS lib
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Stepper lib
#include <AccelStepper.h>

#define PI 3.14159265

// define Step pins
#define MotorInterfaceType 1
#define step1_pin1 1
#define step1_pin2 2
#define step2_pin1 3
#define step2_pin2 4
#define step3_pin1 5
#define step3_pin12 6
AccelStepper Step1(MotorInterfaceType, step1_pin1, step1_pin2);
AccelStepper Step2(MotorInterfaceType, step2_pin1, step2_pin2);
AccelStepper Step3(MotorInterfaceType, step3_pin1, step3_pin2);

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Float32> w1_sub("/omni/speed_1", &speed1StateCallback);
ros::Subscriber<std_msgs::Float32> w2_sub("/omni/speed_1", &speed2StateCallback);
ros::Subscriber<std_msgs::Float32> w3_sub("/omni/speed_1", &speed3StateCallback);

void speed1StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 1 velocity call back (rad/s) adn control wheel 1*/
    speed = convertVelocity2StepSpeed(msg->data);
}

void speed2StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 2 velocity call back (rad/s) adn control wheel 2*/
}

void speed3StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 3 velocity call back (rad/s) adn control wheel 3*/
}

int convertVelocity2StepSpeed(vel)
{
    /*Convert angular velocity of wheel to step per sec to control Stepper*/
}

void setup()
{
    // ros init arduino
    nh.initNode();
    nh.subscribe(joint_sub);   
}

void loop()
{
    // ros spin
    nh.spinOnce();
    delay(100)
}