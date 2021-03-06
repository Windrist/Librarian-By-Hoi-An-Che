#!/usr/bin/env python
'''
The node control planar arm 3DOF
Subscribe: /goal_point - the goal position in Oxyz
Publish: /joint_states - joint states of planar arm
'''

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Declare the params of planar arm
global ARM_LENGTH
ARM_LENGTH = np.array([14, 17, 17])  # cm
global GRIP
GRIP = 10                               # cm
global CURRENT
CURRENT = JointState()
global JOINT_SETS
JOINT_SETS = []

# Global variance
global STATE
STATE = 0
global JOINT_NAMES
JOINT_NAMES = ["joint_front_wheel", "joint_left_wheel", "joint_right_wheel","joint_1", "joint_2", "joint_3"]
global RATE
RATE = 20 # 20Hz
global GOAL
GOAL = Point()
GOAL.x = GOAL.y = GOAL.z = 0
global COUNT
COUNT = 0

# The flag checking state
global is_new_goal, is_grip
is_new_goal = False
is_grip = False

def current_joint_callback(data):
    '''
    Receive the current joint state of planar arm
    data - sensor_msgs/JointState
    '''
    global CURRENT
    CURRENT = data
    # rospy.loginfo(rospy.get_caller_id() + " Current joint state: %s", data)

def goal_point_callback(data):
    '''
    Receive the goal position that planar arm have to move to it
    data - geomatry_msgs/Point
    '''
    global GOAL, is_new_goal
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
    phi = np.pi/2

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
    print("Target joint: {}".format(joint_target))
    return joint_target

def compute_joint_sets(current, goal):
    '''
    Establish the joint trajactory trapezoidal shape 2-1-2.
    Build parameter set for each joint. [c0 c1 c2 tc tf]
    Output: Joint value set of each joint
    '''
    print(current)
    # Compute the max time tf
    speed_max = 150*np.pi/180
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
    print(list(joint_sets))
    return np.array(joint_sets)

def computing():
    '''
    STATE Computing - When Arm are idle and receive new goal to move to
    system will compute joint sets of each joint will be moved
    '''
    # print("computing")
    global ARM_LENGTH, GOAL, JOINT_SETS, STATE, COUNT
    goal = inverse_kinematic(ARM_LENGTH, GOAL)
    JOINT_SETS = compute_joint_sets(np.array(CURRENT.position[3:6]), goal)
    STATE = 2   # STATE: Moving
    COUNT = 0

def moving():
    '''
    STATE Moving - After computing STATE, the arm will move to each joint set
    in joint sets that computed in Computing State
    '''
    # print("Move to goal")
    global CURRENT, COUNT, STATE, JOINT_SETS, RATE
    pre_pos = np.array(CURRENT.position)
    cur_pos = np.append(np.array([0, 0, 0]), np.array(JOINT_SETS[:,COUNT]), 0)

    # Update Joint State
    CURRENT.position = list(cur_pos)
    CURRENT.velocity = list((cur_pos-pre_pos)*RATE)

    COUNT += 1
    if COUNT >= JOINT_SETS.shape[1]:
        STATE = 3   # STATE: Gripping

def gripping():
    '''
    STATE Gripping - After Moving State was done, the gripper will change
    from pick up (True) to droff off (False) or vice versa
    '''
    # print("gripping")
    global STATE
    global is_grip
    is_grip = not is_grip
    STATE = 0   # STATE: Stop
    
def state_planar_arm():
    '''
    STATE:  0 - Stop
            1 - Computing
            2 - Moving
            3 - Gripping
    '''
    global STATE,CURRENT, is_new_goal
    if STATE == 0 and is_new_goal:
        STATE = 1
        is_new_goal = False
    elif STATE == 1:
        computing()
    elif STATE == 2:
        moving()
    elif STATE == 3:
        gripping()
    else:
        STATE = 0
    
    if STATE != 2:
        CURRENT.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def ros_control():
    rospy.init_node("planar_arm", anonymous=True)
    # Subscribe to goal point that arm have to move to
    rospy.Subscriber("/goal_point", Point, goal_point_callback)
    rospy.Subscriber("/joint_states", JointState, current_joint_callback)
    # Publish joint values
    pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=1)

    pub_joint_1 = rospy.Publisher('/joint_1', Float64, queue_size=1)
    pub_joint_2 = rospy.Publisher('/joint_2', Float64, queue_size=1)
    pub_joint_3 = rospy.Publisher('/joint_3', Float64, queue_size=1)

    pub_is_grip = rospy.Publisher('/grip_state', Bool, queue_size=1)

    # Create a robot joint state message
    global CURRENT, is_grip
    CURRENT.header = Header()
    CURRENT.header.stamp = rospy.Time.now()
    CURRENT.name = JOINT_NAMES

    CURRENT.position = [0, 0, 0, 0, 0, 0]
    CURRENT.velocity = []
    CURRENT.effort = []

    # Create a gripper state message
    grip_state = Bool()

    r = rospy.Rate(RATE)

    j1 = Float64()
    j2 = Float64()
    j3 = Float64()

    while not rospy.is_shutdown():
        state_planar_arm()

        CURRENT.header.stamp = rospy.Time.now()
        grip_state.data = is_grip
        # Publish robot joint state data
        
        pub_joint.publish(CURRENT)

        j1.data = float(CURRENT.position[3])
        j2.data = CURRENT.position[4]
        j3.data = CURRENT.position[5]
        pub_joint_1.publish(j1)
        pub_joint_2.publish(j2)
        pub_joint_3.publish(j3)
        pub_is_grip.publish(grip_state)
        r.sleep()

if __name__ == '__main__':    
    try:
        # ros_control()
        # inverse_kinematic(arm_length, goal)
        joint_sets = compute_joint_sets(np.array([0,0,40]), np.array([75,180,10]))
        # for i in range (5):
        #     print(joint_sets[:,i])
        
    except rospy.ROSInterruptException:
        pass