#!/usr/bin/env python
'''
The node control three omniwheel mobile robot
Subscribe: /goal_point - the goal position in Oxyz (z = 0)
Publish: base link position
'''

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

def ros_control():
    rospy.init_node("planar_arm", anonymous=True)
    # Subscribe to goal point that arm have to move to
    rospy.Subscriber("/topic", Type, callback)
    # Publish joint values
    pub_joint = rospy.Publisher('/topic', Type, queue_size=RATE)

    r = rospy.Rate(RATE)


    while not rospy.is_shutdown():

        r.sleep()

if __name__ == '__main__':    
    try:
        ros_control()        
    except rospy.ROSInterruptException:
        pass