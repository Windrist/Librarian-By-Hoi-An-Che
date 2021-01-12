
// ROS lib
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

// Arduino lib
#include <Servo.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

/**************************
* OMNI WHEELS CONTROLLER *
**************************/
#define MAX_SPEED 10000
#define MAX_ACCEL 10

// Declare all PIN used
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

// Create Steppers
AccelStepper Step1(MotorInterfaceType, step1_pin, step1_dir);
AccelStepper Step2(MotorInterfaceType, step2_pin, step2_dir);
AccelStepper Step3(MotorInterfaceType, step3_pin, step3_dir);

// Get Stepper Variable
int convertVelocity2StepSpeed(float vel)
{
   /*Convert angular velocity (rad/s) of wheel to steps/s to control Stepper*/
   int speed = int(vel / (2*PI) * 200) * 16;
   return speed;
}

void frontCallback(const std_msgs::Float64& msg)
{
   /*Front Wheel velocity call back (rad/s) and control Front Wheel*/
   int speed = convertVelocity2StepSpeed(msg.data);
   Step1.setSpeed(speed);
}

void leftCallback(const std_msgs::Float64& msg)
{
   /*Left Wheel velocity call back (rad/s) and control Front Wheel*/
   int speed = convertVelocity2StepSpeed(msg.data);
   Step2.setSpeed(speed);
}

void rightCallback(const std_msgs::Float64& msg)
{
   /*Right Wheel velocity call back (rad/s) and control Right Wheel*/
   int speed = convertVelocity2StepSpeed(msg.data);
   Step3.setSpeed(speed);
}

// Subscribe to Stepper Speed Message
ros::Subscriber<std_msgs::Float64> fw_sub("/librarian/front_wheel_controller/command", &frontCallback);
ros::Subscriber<std_msgs::Float64> lw_sub("/librarian/left_wheel_controller/command", &leftCallback);
ros::Subscriber<std_msgs::Float64> rw_sub("/librarian/right_wheel_controller/command", &rightCallback);

/******************************
 * PLANAR ARM 3DOF CONTROLLER *
******************************/
// Declare all PIN used
#define j1_step_pin 26
#define j1_step_dir 28
#define j2_servo_pin 4
#define j3_servo_pin 5
#define grip_servo_pin 6

// Create Joints
#define MotorInterfaceType 1
AccelStepper J1Stepper(MotorInterfaceType, j1_step_pin, j1_step_dir);
Servo J2Servo;
Servo J3Servo;
Servo GripServo;

// Call ROS for Grip State Message
std_msgs::Int16 theta_msg;
ros::Subscriber<std_msgs::Int16> state_sub("/Grip_state", &stateCallback);

float theta_1 = 0, theta_2 = 0, theta_3 = 0;
bool is_grip = false;

void moveToGoal()
{
  
  GripServo.write(60);
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((60-0)/15*i);
    J3Servo.write((180-0)/15*i);
    delay(100);
  }
  delay(1000);
  GripServo.write(10);
  delay(1000);
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((0-60)/15*i+60);
    J3Servo.write((0-180)/15*i+180);
    delay(150);
  }
  delay(1000);
}

void droff()
{
  
  GripServo.write(10);
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((60-0)/15*i);
    J3Servo.write((180-0)/15*i);
    delay(100);
  }
  delay(1000);
  GripServo.write(60);
  delay(1000);
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((0-60)/15*i+60);
    J3Servo.write((0-180)/15*i+180);
    delay(150);
  }
  delay(1000);
}

void joint1Callback(const std_msgs::Float64& msg)
{
    /*
     * Control joint 1 - Stepper motor
     */
    // Convert angular to step number
    float theta = msg.data/PI*180;
    static float last_step = 0;
    float step = 1.8; // 1 step = 1.8 degree
    int curr_step = round(theta/step);  // compute current step position need move to
    int speed = round((curr_step-last_step)*10);    // compute speed of stepper by differention
    J1Stepper.setSpeed(speed);          // set speed for stepper
    J1Stepper.runToNewPosition(curr_step*49);     // move to current position (gear ratio 1:49)
    last_step = curr_step;              // upda     te last step position
}

const int check_pin = 10;

int lastState = 0;

void stateCallback(const std_msgs::Int16& msg)
{
    int cmd = msg.data;
    if (lastState != cmd)
    {
        switch (cmd)
        {
            case 1:
            {
                moveToGoal();
                break;
            }
            case 2:
            {
                break;
            }
            case 3:
            {
                droff();
                break;
            }
        }
    }
}

void setup()
{    
    // ROS Init Arduino
    nh.initNode();
    nh.getHardware()->setBaud(57600);

    /************************
    * PLANAR ARM 3DOF SETUP *
    ************************/
    // Subscribe to Grip State Message
    nh.subscribe(state_sub);

    // Declare Checked pin
    pinMode(check_pin, INPUT);

    // Declare Plarnar Arm 3DOF params
    J1Stepper.setMaxSpeed(10000);
    J2Servo.attach(j2_servo_pin);   // Joint 2
    J3Servo.attach(j3_servo_pin);   // Joint 3
    GripServo.attach(grip_servo_pin);   // Grip
    
    // Calib Robot Arm
    J2Servo.write(0);
    J3Servo.write(0);
    GripServo.write(60);

    /********************
    * OMNI WHEELS SETUP *
    ********************/
    // Subscribe to Wheels Velocity Message
    nh.subscribe(fw_sub);   // Front Wheel
    nh.subscribe(lw_sub);   // Left Wheel
    nh.subscribe(rw_sub);   // Right Wheel

    // Set Value for each stepper
    Step1.setMaxSpeed(MAX_SPEED);
    Step2.setMaxSpeed(MAX_SPEED);
    Step3.setMaxSpeed(MAX_SPEED);
    Step1.setAcceleration(MAX_ACCEL);
    Step2.setAcceleration(MAX_ACCEL);
    Step3.setAcceleration(MAX_ACCEL);
}

void loop()
{
    /*Omni Wheel Send Command*/
    Step1.runSpeed();
    Step2.runSpeed();
    Step3.runSpeed();

    // ROS Spin
    nh.spinOnce();
}
