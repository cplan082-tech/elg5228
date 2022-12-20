#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class drive_positive_v_w(object):
    def __init__(self):
        self.msg = Twist()
        self._build_msg()
        
        rospy.init_node('drive_positive_v_w',  anonymous=False)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        self.rate = rospy.Rate(10)
        
    
    def _build_msg(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 0.5
        
        
        
    def pub_vel(self):
        self.pub.publish(self.msg)
        self.rate.sleep()
        
        

if __name__ == "__main__":
    obj = drive_positive_v_w()
        
    while not rospy.is_shutdown():
        try:
            obj.pub_vel()
        except:
            pass
        