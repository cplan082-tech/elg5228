#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist

class drive_robot():
    x_L = -0.032
    y_L = 0
    theta = 0
    
    def __init__(self):
        rospy.init_node('drive_robot', anonymous=False)
        
        self.ratio_xy = drive_robot.y_L/drive_robot.x_L
        self.ratio_1_x = 1/drive_robot.x_L
        
        self.sub_sensObj = rospy.Subscriber('/sense_object', Pose, self.callback_sensObj)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.msg_sensObj = Pose()
        
        self.rate = rospy.Rate(10) # publish frequency in Hz

        
    def callback_sensObj(self, msg):
        self.msg_sensObj = msg
        
        
    def publish_cmd_vel(self, x, z):
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = x
        msg_cmd_vel.angular.z = z
            
        self.pub_cmd_vel.publish(msg_cmd_vel)
        
        
    def calc_vel(self):
        msg = self.msg_sensObj
        
        arr_v_a_L = np.array([[msg.orientation.x],
                              [msg.orientation.y]])
        
        if not (np.isnan(arr_v_a_L)).any():
            alpha = msg.orientation.z
            _arg = np.deg2rad(drive_robot.theta + alpha)
            _cos = np.cos(_arg)
            _sin = np.sin(_arg)
            
            arr_TR = np.array([[_cos + self.ratio_xy*_sin, -_sin + self.ratio_xy*_cos],
                               [self.ratio_1_x*_sin, self.ratio_1_x*_cos]])
            
            vel_lin_ang = np.dot(arr_TR, arr_v_a_L)
            
        else:
            vel_lin_ang = np.array([0, 0]).reshape(-1,1)
            
        self.publish_cmd_vel(vel_lin_ang[0][0], vel_lin_ang[1][0])
        self.rate.sleep()
        
        

if __name__ == "__main__":
    obj = drive_robot()
    while not rospy.is_shutdown():
        try:
            obj.calc_vel()
        except:
            pass