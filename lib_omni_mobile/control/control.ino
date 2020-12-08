// ROS lib
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Stepper lib
#include <AccelStepper.h>

#define PI 3.14159265

// define Step pins
#define MotorInterfaceType 1
#define step1_pin 1
#define step1_dir 2
#define step2_pin 3
#define step2_dir 4
#define step3_pin 5
#define step3_dir 6
AccelStepper Step1(MotorInterfaceType, step1_pin, step1_dir);
AccelStepper Step2(MotorInterfaceType, step2_pin, step2_dir);
AccelStepper Step3(MotorInterfaceType, step3_pin, step3_dir;

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Float32> w1_sub("/omni/speed_1", &speed1StateCallback);
ros::Subscriber<std_msgs::Float32> w2_sub("/omni/speed_1", &speed2StateCallback);
ros::Subscriber<std_msgs::Float32> w3_sub("/omni/speed_1", &speed3StateCallback);

void speed1StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 1 velocity call back (rad/s) adn control wheel 1*/
    speed = convertVelocity2StepSpeed(msg->data);
    Step1.setSpeed(speed);
    Step1.runSpeed();
}

void speed2StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 2 velocity call back (rad/s) adn control wheel 2*/
    speed = convertVelocity2StepSpeed(msg->data);
    Step2.setSpeed(speed);
    Step2.runSpeed();
}

void speed3StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 3 velocity call back (rad/s) adn control wheel 3*/
    speed = convertVelocity2StepSpeed(msg->data);
    Step3.setSpeed(speed);
    Step3.runSpeed();
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

    // Set max Speed for each stepper
    Step1.setMaxSpeed(10000);
    Step2.setMaxSpeed(10000);
    Step3.setMaxSpeed(10000);
}

void loop()
{
    // ros spin
    nh.spinOnce();
    delay(100)
}