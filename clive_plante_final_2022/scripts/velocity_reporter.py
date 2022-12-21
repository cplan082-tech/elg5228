#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class velocity_reporter():
    def __init__(self):
        rospy.init_node('velocity_reporter', anonymous=False)
        
        self.sub_cmd_vel = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.callback_cmd_vel)
        
        self.rate = rospy.Rate(1) # publish frequency in Hz
        
        
    def callback_cmd_vel(self, msg):
        self.msg_cmd_vel = msg
        
        
    def loginfo_final(self):
        msg_cmd_vel = self.msg_cmd_vel
        v = msg_cmd_vel.linear.x
        w = msg_cmd_vel.angular.z
        
        rospy.loginfo("\n\n======================================")
        rospy.loginfo("v = %7.3f m/s", v)
        rospy.loginfo("w = %7.3f rad/s", w)
        self.rate.sleep()
        
        
if __name__ == "__main__":
    
    obj = velocity_reporter()
    while not rospy.is_shutdown():
        try:
            obj.loginfo_final()
        except:
            pass