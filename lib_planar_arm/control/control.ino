#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

float theta_1, theta_2, theta_3;
bool is_grip = false;

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    /*
     * Get angle of each joints, get from robot
     * Output: 3 angle of theta 1,2,3 (rad)
     */
    sensor_msgs::JointState joint_state = msg;
    theta_1 = joint_state.position[0];
    theta_2 = joint_state.position[1];
    theta_3 = joint_state.position[2];
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
        }
        else
        {
            // Pick up
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
}

void controlJoint3(float theta)
{
    /*
     * Control joint 3 - Servo motor
     */
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
}

void loop()
{
    // Code here

    // ros spin
    nh.spinOnce();
    delay(100);
}