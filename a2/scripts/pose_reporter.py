#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

def callback(received_message):
    return rospy.loginfo('Pose = %s', received_message)

def listener():
    rospy.init_node('pose_reporter', anonymous=False)
    
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    
    rospy.spin()
    

if __name__ == '__main__':
    listener()