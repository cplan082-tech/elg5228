#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
import numpy as np

def callback(received_message):
    print(received_message.theta)
    return rospy.loginfo('Pose = %s', received_message)

def listener():
    rospy.init_node('pose_reporter', anonymous=False)
    
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    
    rospy.spin()
    

if __name__ == '__main__':
    listener()
