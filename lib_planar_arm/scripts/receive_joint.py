#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState

if __name__ == '__main__':    
    try:
        rospy.init_node("receive_joint", anonymous=True)
    except rospy.ROSInterruptException:
        pass