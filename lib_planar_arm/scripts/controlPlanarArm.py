#!/usr/bin/env python
'''
The node control planar arm 3DOF
Subscribe: goal - the goal position that the arm have to move to
Publish: 
'''

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

# Declare the params of planar arm
arm_length = np.array([5.2, 6.9, 6.8])  # cm
grip = 10                               # cm

# Global variance
goal = Point()
rate = 200 # 200Hz

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

    # Convert radian to degree
    theta_1 = np.rad2deg(theta_1)
    theta_2 = np.rad2deg(theta_2)
    theta_3 = np.rad2deg(theta_3)

    joint_target = np.array([theta_1, theta_2, theta_3])
    print "Target joint: {}".format(joint_target)
    return joint_target

def goal_point_callback(data):
    '''
    Receive the goal position that planar arm have to move to it
    data - geomatry_msgs/Point
    '''
    global goal
    goal = data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def compute_joint_sets(current, goal):
    '''
    Establish the joint trajactory trapezoidal shape 2-1-2.
    Build parameter set for each joint. [c0 c1 c2 tc tf]
    Output: Joint value set of each joint
    '''
    global rate
    # Compute the max time tf
    speed_max = 150
    angle_max = np.max(np.abs(goal - current))  # Angle max of moving in all joints
    tf = 1.0*angle_max/speed_max    # Time max of moving
    dt = 1.0/rate             # The time step
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

def state_planar_arm():
    pass

# def publish_joint_state():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # rate.sleep()

def ros_control():
    rospy.init_node("planar_arm", anonymous=True)
    # Subscribe to goal point that arm have to move to
    rospy.Subscriber("/goal_point", Point, goal_point_callback)

    # Publish joint values
    pub_t1 = rospy.Publisher('/planar_arm/theta1', Float32)
    pub_t2 = rospy.Publisher('/planar_arm/theta2', Float32)
    pub_t3 = rospy.Publisher('/planar_arm/theta3', Float32)

    global rate
    r = rospy.Rate(rate)

    while not rospy.is_shutdown():
        # state_machine()
        r.sleep()

if __name__ == '__main__':    
    try:
        # ros_control()
        # inverse_kinematic(arm_length, goal)
        # compute_joint_sets(np.array([0,0,0]), np.array([20,50,40]))
    except rospy.ROSInterruptException:
        pass