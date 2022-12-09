#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist

class data_reporter():
    def __init__(self):
        rospy.init_node('data_reporter', anonymous=False)
        
        self.sub_sensObj = rospy.Subscriber('/sense_object', Pose, self.callback_sensObj)
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd_vel)
        
        self.msg_sensObj = Pose()
        self.msg_cmd_vel = Twist()
        
        self.rate = rospy.Rate(2) # publish frequency in Hz
        
    
    def callback_sensObj(self, msg):
        self.msg_sensObj = msg
    
    
    def callback_cmd_vel(self, msg):
        self.msg_cmd_vel = msg
        
        
    def loginfo_a4(self):
        msg_sensObj = self.msg_sensObj
        msg_cmd_vel = self.msg_cmd_vel
        
        d = msg_sensObj.position.x
        err = msg_sensObj.position.y
        err_nrm = msg_sensObj.position.z
        alpha = msg_sensObj.orientation.z
        v = msg_cmd_vel.linear.x
        w = msg_cmd_vel.angular.z
        
        rospy.loginfo("\n\n======================================")
        rospy.loginfo("d = %7.3f m", d)
        rospy.loginfo("e = %7.3f m", err)
        rospy.loginfo("e_nrm = %7.3f", err_nrm)
        rospy.loginfo("alpha = %7.3f deg", alpha)
        rospy.loginfo("v = %7.3f m/s", v)
        rospy.loginfo("w = %7.3f deg/s", w)
        self.rate.sleep()
        
        
if __name__ == "__main__":
    
    obj = data_reporter()
    while not rospy.is_shutdown():
        try:
            obj.loginfo_a4()
        except:
            pass
