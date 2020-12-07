// ROS lib
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

// Arduino lib
#include <Servo.h>
#include <AccelStepper.h>

#define PI 3.14159265

// Declare all PIN used
#define j1_step_pin1 1
#define j1_step_pin2 2
#define j2_servo_pin 3
#define j3_servo_pin 4
#define grip_servo_pin 5

// Create objects
#define MotorInterfaceType 1
AccelStepper J1Stepper(MotorInterfaceType, j1_step_pin1, j1_step_pin2);
Servo J2Servo;
Servo J3Servo;
Servo GripServo;

ros::NodeHandle nh;

float theta_1, theta_2, theta_3;
bool is_grip = false;

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    /*
     * Get angle of each joints, get from robot
     * Output: 3 angle of theta 1,2,3 (angle)
     */
    sensor_msgs::JointState joint_state = msg;
    theta_1 = joint_state.position[0]/PI*180;
    theta_2 = joint_state.position[1]/PI*180;
    theta_3 = joint_state.position[2]/PI*180;
}

void gripStateCallback(const std_msgs::Bool& msg)
{
    /*
     * Get grip robot state (pick up/droff off) andf control gripper
     */
    if(msg.data != is_grip)
    {
        is_grip = !is_grip;
        if(is_grip)
        {
            // Droff off
            GripServo.write(180);
        }
        else
        {
            // Pick up
            GripServo.write(0);
        }
    }

}

void controlJoint1(float theta)
{
    /*
     * Control joint 1 - Stepper motor
     */
    
}

void controlJoint2(float theta)
{
    /*
     * Control joint 2 - Servo motor
     */
    J2Servo.write(theta);
}

void controlJoint3(float theta)
{
    /*
     * Control joint 3 - Servo motor
     */
    J3Servo.write(theta);
}


// Subscribe joint state data
ros::Subscriber<sensor_msgs::JointState> joint_sub("/joint_states", &jointStateCallback);
ros::Subscriber<std_msgs::Bool> joint_sub("/grip_state", &gripStateCallback);

void setup()
{
    // ros init arduino
    nh.initNode();
    nh.subscribe(joint_sub);

    // Declare Plarnar Arm 3DOF params
    J1Stepper.setmaxSpeed(1000);
    J2Servo.attach(j2_servo_pin);   // Joint 2
    J3Servo.attach(j3_servo_pin);   // Joint 3
    GripServo.attach(grip_servo_pin);   // Grip
    
}

void loop()
{
    // Code here
    controlJoint1(theta_1);
    controlJoint2(theta_2);
    controlJoint3(theta_3);

    // ros spin
    nh.spinOnce();
    delay(100);
}