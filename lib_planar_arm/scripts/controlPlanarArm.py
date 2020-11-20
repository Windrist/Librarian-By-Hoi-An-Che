#!/usr/bin/env python
'''
The node control planar arm 3DOF
Subscribe: /goal_point - the goal position in Oxyz
Publish: /joint_states - joint states of planar arm
'''

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Declare the params of planar arm
global ARM_LENGTH
ARM_LENGTH = np.array([5.2, 6.9, 6.8])  # cm
global GRIP
GRIP = 10                               # cm
global is_new_goal
is_new_goal = False
global is_grip
is_grip = False

# Global variance
global JOINT_NAMES
JOINT_NAMES = ["joint_1", "joint_2", "joint_3"]
global RATE
RATE = 200 # 200Hz
global GOAL
GOAL = Point()
GOAL.x = GOAL.y = GOAL.z = 0

def goal_point_callback(data):
    '''
    Receive the goal position that planar arm have to move to it
    data - geomatry_msgs/Point
    '''
    global GOAL
    if not(data.x - GOAL.x == 0.0 and data.y - GOAL.y == 0.0 and data.z - GOAL.z == 0.0):
        print("change")
        GOAL = data
        is_new_goal = True
        rospy.loginfo(rospy.get_caller_id() + " New goal: %s", data)

def inverse_kinematic(arm_length, goal):
    '''
    Inverse Kinematic of Planar 3DOF arm
    goal - geomatry_msgs/Point
    arm_length - 
    '''
    # Get length of planar arm
    a1 = arm_length[0]
    a2 = arm_length[1]
    a3 = arm_length[2]

    # Get goal position
    px = goal.x
    pz = goal.z

    # The target phi of End-Effector is 0 rad
    phi = 0

    # Equations for Inverse kinematics
    wx = px - a3*np.cos(phi)
    wz = pz - a3*np.sin(phi)

    delta = wx**2 + wz**2
    c2 = (delta -a1**2 -a2**2)/(2*a1*a2)
    s2 = np.sqrt(1-c2**2)
    theta_2 = np.arctan2(s2, c2)

    s1 = ((a1+a2*c2)*wz - a2*s2*wx)/delta
    c1 = ((a1+a2*c2)*wx + a2*s2*wz)/delta
    theta_1 = np.arctan2(s1,c1)
    theta_3 = phi-theta_1-theta_2

    joint_target = np.array([theta_1, theta_2, theta_3])
    print "Target joint: {}".format(joint_target)
    return joint_target

def compute_joint_sets(current, goal):
    '''
    Establish the joint trajactory trapezoidal shape 2-1-2.
    Build parameter set for each joint. [c0 c1 c2 tc tf]
    Output: Joint value set of each joint
    '''
    # Compute the max time tf
    speed_max = 150
    angle_max = np.max(np.abs(goal - current))  # Angle max of moving in all joints
    tf = 1.0*angle_max/speed_max    # Time max of moving
    dt = 1.0/RATE             # The time step
    steps = int(round(tf/dt))    # Number the joint value

    joint_sets = np.zeros((len(goal), steps))

    # Computer the param set
    for i in range(len(goal)):
        c0 = current[i]
        c1 = 0
        w = 2*(goal[i] - current[i])/tf
        tc = (current[i] - goal[i])/w + tf
        c2 = w/tc

        # Compute the joint i set
        joint_set = np.zeros(steps)
        for j in range(steps):
            if dt*j < tc:
                # Speed up
                joint_set[j] = c0 + c2/2 * (dt*j)**2
            elif tc < dt*j and dt*j < (tf-tc):
                # Stability
                joint_set[j] = w*dt*j
            else:
                # Speed down
                joint_set[j] = goal[i] - 0.5*c2*(tf-dt*j)**2
        
        joint_sets[i] = joint_set
    print joint_sets
    return joint_sets

def gripper_pick_up():
    print("Pick up")

def gripper_droff_off():
    print("Droff off")

def state_planar_arm():
    pass

def ros_control():
    rospy.init_node("planar_arm", anonymous=True)
    # Subscribe to goal point that arm have to move to
    rospy.Subscriber("/goal_point", Point, goal_point_callback)
    # Publish joint values
    pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=RATE)

    # Create a robot joint state message
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp = rospy.Time.now()
    joint_states.name = JOINT_NAMES

    joint_states.position = [0, 0, 0]
    joint_states.velocity = []
    joint_states.effort = []

    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        # state_machine()

        joint_states.header.stamp = rospy.Time.now()
        # Publish robot joint state data
        pub_joint.publish(joint_states)
        r.sleep()

if __name__ == '__main__':    
    try:
        ros_control()
        # inverse_kinematic(arm_length, goal)
        # compute_joint_sets(np.array([0,0,0]), np.array([20,50,40]))
    except rospy.ROSInterruptException:
        pass