
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

#define PI 3.14159265
#define USE_USBCON
ros::NodeHandle nh;

/******************************
 * PLANAR ARM 3DOF CONTROLLER *
******************************/
// Declare all PIN used
#define j1_step_pin 26
#define j1_step_dir 28
#define j2_servo_pin 4
#define j3_servo_pin 5
#define grip_servo_pin 6

// Create objects
#define MotorInterfaceType 1
AccelStepper J1Stepper(MotorInterfaceType, j1_step_pin, j1_step_dir);
Servo J2Servo;
Servo J3Servo;
Servo GripServo;

std_msgs::Float64 theta_msg;
ros::Publisher chatter("chatter", &theta_msg);

float theta_1 = 0, theta_2 = 0, theta_3 = 0;
bool is_grip = false;
//
//void jointStateCallback(const sensor_msgs::JointState& msg)
//{
//    /*
//     * Get angle of each joints, get from robot
//     * Output: 3 angle of theta 1,2,3 (angle)
//     */
//     theta_msg.data = 30;
//    chatter.publish( &theta_msg );
//    sensor_msgs::JointState joint_state = msg;
//    theta_1 = joint_state.position[3]/PI*180;
//    theta_2 = joint_state.position[4]/PI*180;
//    theta_3 = joint_state.position[5]/PI*180;
//}

void moveToGoal()
{
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((60-0)/15*i);
    J3Servo.write((180-0)/15*i);
    delay(100);
  }
  delay(1000);
  GripServo.write(10);
}

void droff()
{
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((60-0)/15*i);
    J3Servo.write((180-0)/15*i);
    delay(100);
  }
  delay(1000);
  GripServo.write(60);
}

void comeBack()
{
  for(int i = 0; i < 15; i++)
  {
    J2Servo.write((0-60)/15*i+60);
    J3Servo.write((0-180)/15*i+180);
    delay(150);
  }
  delay(1000);
  GripServo.write(10);
}

void gripStateCallback(const std_msgs::Bool& msg)
{
    /*
     * Get grip robot state (pick up/droff off) andf control gripper
     */
//    int cmd = Serial.read();
//    if(cmd == 'true')
//    {
//        moveToGoal();
//    }
//    else if (cmd == 'false')
//    {
//        comeBack();
//    }

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

void joint2Callback(const std_msgs::Float64& msg)
{
    /*
     * Control joint 2 - Servo motor
     */
    // float theta = msg.data/PI*180;
    // J2Servo.write(theta);
}

void joint3Callback(const std_msgs::Float64& msg)
{
    /*
     * Control joint 3 - Servo motor
     */
    // float theta = msg.data/PI*180;
    // J3Servo.write(theta);
}

// Subscribe joint state data
//ros::Subscriber<sensor_msgs::JointState> joint_sub("/librarian/joint_states", &jointStateCallback);
ros::Subscriber<std_msgs::Bool> grip_sub("/grip_state", &gripStateCallback);

ros::Subscriber<std_msgs::Float64> j1_sub("/joint_1", &joint1Callback);
ros::Subscriber<std_msgs::Float64> j2_sub("/joint_2", &joint2Callback);
ros::Subscriber<std_msgs::Float64> j3_sub("/joint_3", &joint3Callback);

const int check_pin = 10;

//void calibrate()
//{
//    // Calib joint 1
//    int checked = 0;
//    while(!checked)
//    {
//        checked = digitalRead(check_pin);
//        J1Stepper.setSpeed(-500);
//        J1Stepper.run();
//    }
////    setCurrentPosition(25*49); // ~45degree
//    // calib joint 2+3
//    J2Servo.write(0);
//    J3Servo.write(0);
//    delay(5000); // delay 5 seconds
//}
//
///**************************
// * OMNI WHEELS CONTROLLER *
//**************************/
//#define MAX_SPEED 10000
//#define MAX_ACCEL 10
//
//// define Step pins
//#define MotorInterfaceType 1
//#define step1_pin A0
//#define step1_dir A1
//#define step1_en 38msg
//#define step2_pin A6
//#define step2_dir A7
//#define step2_en A2
//#define step3_pin 46
//#define step3_dir 48
//#define step3_en A8
//AccelStepper Step1(MotorInterfaceType, step1_pin, step1_dir);
//AccelStepper Step2(MotorInterfaceType, step2_pin, step2_dir);
//AccelStepper Step3(MotorInterfaceType, step3_pin, step3_dir);
//
//int convertVelocity2StepSpeed(float vel)
//{
//    /*Convert angular velocity (rad/s) of wheel to step/s to control Stepper*/
//    int speed = int(vel/(2*PI)*200) * 16;
//    return speed;
//}
//
//void speed1StateCallback(const std_msgs::Float64& msg)
//{
//    /*Wheel 1 velocity call back (rad/s) and control wheel 1*/
//    int speed = convertVelocity2StepSpeed(msg.data);
//    Step1.setSpeed(speed);
////    Step1.runSpeed();
//}
//
//void speed2StateCallback(const std_msgs::Float64& msg)
//{
//    /*Wheel 2 velocity call back (rad/s) and control wheel 2*/
//    int speed = convertVelocity2StepSpeed(msg.data);
//    Step2.setSpeed(speed);
////    Step2.runSpeed();
//}
//
//void speed3StateCallback(const std_msgs::Float64& msg)
//{
//    /*Wheel 3 velocity call back (rad/s) and control wheel 3*/
//    int speed = convertVelocity2StepSpeed(msg.data);
//    Step3.setSpeed(speed);
////    Step3.runSpeed();
//}

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
                comeBack();
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

//ros::Subscriber<std_msgs::Float64> w1_sub("/librarian/front_wheel_controller/command", &speed1StateCallback);
//ros::Subscriber<std_msgs::Float64> w2_sub("/librarian/left_wheel_controller/command", &speed2StateCallback);
//ros::Subscriber<std_msgs::Float64> w3_sub("/librarian/right_wheel_controller/command", &speed3StateCallback);
ros::Subscriber<std_msgs::Int16> state_sub("/state", &stateCallback);

void setup()
{
//    Serial.begin(57600);
    
    // ros init arduino
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    /************************
    * PLANAR ARM 3DOF SETUP *
    ************************/
//    nh.subscribe(joint_sub);

    // Declare Checked pin
    pinMode(check_pin, INPUT);

    // Declare Plarnar Arm 3DOF params
    nh.advertise(chatter);
    J1Stepper.setMaxSpeed(10000);
    J2Servo.attach(j2_servo_pin);   // Joint 2
    J3Servo.attach(j3_servo_pin);   // Joint 3
    GripServo.attach(grip_servo_pin);   // Grip
    
    // Calib robot arm
//    calibrate();
    J2Servo.write(0);
    J3Servo.write(0);
    GripServo.write(60);

    /********************
    * OMNI WHEELS SETUP *
    ********************/
//    nh.subscribe(w1_sub);   
//    nh.subscribe(w2_sub);
//    nh.subscribe(w3_sub);
    nh.subscribe(state_sub);
//    // Set max Speed for each stepper
//    Step1.setMaxSpeed(MAX_SPEED);
//    Step2.setMaxSpeed(MAX_SPEED);
//    Step3.setMaxSpeed(MAX_SPEED);
//    Step1.setAcceleration(MAX_ACCEL);
//    Step2.setAcceleration(MAX_ACCEL);
//    Step3.setAcceleration(MAX_ACCEL);
}

void loop()
{
    /*Planar Arm*/
//    controlJoint1(theta_1);
//    controlJoint2(theta_2);
//    controlJoint3(theta_3);
    if(Serial.available() > 0)
    {
        String cmd = Serial.readString();
        int a = cmd.toInt();
        switch (a)
        {
          case 1:
          {
            moveToGoal();
            break;
          }
          case 2:
          {
            comeBack();
            break;
          }
          case 3:
          {
            droff();
            break;
          }
        }
    }

    /*Omni wheel*/
//    Step1.runSpeed();
//    Step2.runSpeed();
//    Step3.runSpeed();

    // ros spin
    nh.spinOnce();
    delay(200);
}
