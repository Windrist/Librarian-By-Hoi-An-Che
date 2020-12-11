// ROS lib
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Stepper lib
#include <AccelStepper.h>

#define PI 3.14159265
#define MAX_SPEED 10000
#define MAX_ACCEL 10

// define Step pins
#define MotorInterfaceType 1
#define step1_pin A0
#define step1_dir A1
#define step1_en 38
#define step2_pin A6
#define step2_dir A7
#define step2_en A2
#define step3_pin 46
#define step3_dir 48
#define step3_en A8
AccelStepper Step1(MotorInterfaceType, step1_pin, step1_dir);
AccelStepper Step2(MotorInterfaceType, step2_pin, step2_dir);
AccelStepper Step3(MotorInterfaceType, step3_pin, step3_dir);

ros::NodeHandle nh;

int convertVelocity2StepSpeed(float vel)
{
    /*Convert angular velocity (rad/s) of wheel to step/s to control Stepper*/
    int speed = int(vel/(2*PI)*200);
    return speed;
}

void speed1StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 1 velocity call back (rad/s) and control wheel 1*/
    int speed = convertVelocity2StepSpeed(msg.data);
    Step1.setSpeed(speed);
//    Step1.runSpeed();
}

void speed2StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 2 velocity call back (rad/s) and control wheel 2*/
    int speed = convertVelocity2StepSpeed(msg.data);
    Step2.setSpeed(speed);
//    Step2.runSpeed();
}

void speed3StateCallback(const std_msgs::Float32& msg)
{
    /*Wheel 3 velocity call back (rad/s) and control wheel 3*/
    int speed = convertVelocity2StepSpeed(msg.data);
    Step3.setSpeed(speed);
//    Step3.runSpeed();
}

ros::Subscriber<std_msgs::Float32> w1_sub("/omni/speed_1", &speed1StateCallback);
ros::Subscriber<std_msgs::Float32> w2_sub("/omni/speed_2", &speed2StateCallback);
ros::Subscriber<std_msgs::Float32> w3_sub("/omni/speed_3", &speed3StateCallback);

void setup()
{
    Serial.begin(57600);
    
    // ros init arduino
    nh.initNode();
    nh.subscribe(w1_sub);   
    nh.subscribe(w2_sub);
    nh.subscribe(w3_sub);
    // Set max Speed for each stepper
    Step1.setMaxSpeed(MAX_SPEED);
    Step2.setMaxSpeed(MAX_SPEED);
    Step3.setMaxSpeed(MAX_SPEED);
    Step1.setAcceleration(MAX_ACCEL);
    Step2.setAcceleration(MAX_ACCEL);
    Step3.setAcceleration(MAX_ACCEL);
}

void loop()
{
    // ros spin
    Step1.runSpeed();
    Step2.runSpeed();
    Step3.runSpeed();
    nh.spinOnce();
//    delay(100);
}
