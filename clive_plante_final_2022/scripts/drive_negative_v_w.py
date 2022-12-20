#!/usr/bin/env python
import rospy
from drive_positive_v_w import drive_positive_v_w as baseClass

class drive_negative_v_w(baseClass):
    def __init__(self):
        super(drive_negative_v_w, self).__init__()
        
        
    def _build_msg(self):
        self.msg.linear.x = -0.5
        self.msg.angular.z = -0.5
        
        

if __name__ == "__main__":
    obj = drive_negative_v_w()
        
    while not rospy.is_shutdown():
        try:
            obj.pub_vel()
        except:
            pass
    