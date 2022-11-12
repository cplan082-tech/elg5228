#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
import numpy as np

class pose_reporter():
    
    def __init__(self):
        
        rospy.init_node('pose_reporter', anonymous=False)
        
        self.rate = rospy.Rate(2)
        
        self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, 
                                         self.callback_pose)
        self.sub_dest = rospy.Subscriber('/turtle1/destination', Pose, 
                                         self.callback_dest)
        
        self.pose = Pose()
        self.dest = Pose()
        
        self.HT_0_d = self.HT_builder(self.z_elemental(0), np.array([5.5, 5.5, 0]).reshape(-1,1))
        

    def callback_pose(self, received_message):
        self.pose = received_message
        
        # return rospy.loginfo('Pose = %s', received_message)
    
    
    def callback_dest(self, received_message):
        self.dest = received_message
        
        
    def loginfo_pub(self):
        # R_0_d stands for destination(d) orientation wrt world frame(0)
        R_0_d = self.z_elemental(np.arctan2(self.dest.y, self.dest.x))
        R_t_0 = self.z_elemental(self.pose.theta).T
        
        # position of dest wrt to world frame
        p_0 = np.array([self.dest.x, self.dest.y, 0]).reshape(-1,1)
        q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
        
        self.HT_0_d = self.HT_builder(R_0_d, p_0)
        HT_0_t = self.HT_builder(R_t_0, -np.dot(R_t_0, q_0))
        
        self.HT_t_d = np.dot(HT_0_t, self.HT_0_d)
        
        # log info
        
        lst_robo_position_curr = [self.pose.x, self.pose.y]
        dist2go = np.sqrt(self.HT_t_d[0,3]**2 + self.HT_t_d[1,3]**2)
        robo_orien_curr = np.rad2deg(self.pose.theta)
        orient_err = np.rad2deg(np.arctan2(self.HT_t_d[1,3], self.HT_t_d[0,3]))
        
        # rospy.loginfo('', )
        rospy.loginfo('\n\n====================Info====================')
        rospy.loginfo('Robot Position:\n\t\t\t\tx= %s m \n\t\t\t\ty= %s m', 
                      lst_robo_position_curr[0],
                      lst_robo_position_curr[1])
        rospy.loginfo('Distance between robot and destination:\n\t\t\t\t %s m', 
                      dist2go)
        rospy.loginfo("Robot's current orientation:\n\t\t\t\t %s Deg", 
                      robo_orien_curr)
        rospy.loginfo('Orientation error:\n\t\t\t\t %s m', orient_err)
        
        # self.rate.sleep()
        
    
    def z_elemental(self, gamma):            
        R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])
        
        return R_z
    
    
    def HT_builder(self, R, p):
        arr_zeros = np.array([[0, 0, 0, 1]])
        return np.vstack((np.hstack((R, p)), arr_zeros))
    
    
    # def listener():
    #     rospy.init_node('pose_reporter', anonymous=False)
        
    #     rospy.Subscriber('/turtle1/pose', Pose, callback_pose)
        
    #     rospy.spin()
    

if __name__ == '__main__':
    obj = pose_reporter()
    
    while not rospy.is_shutdown():
        try:
            obj.loginfo_pub()
            
        except rospy.ROSInterruptException:
            pass
        obj.rate.sleep()
