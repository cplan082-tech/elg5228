#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class robot_driver():
    
    eps = 1e-2
    dist_arrived = 10e-2
    
    def __init__(self, rot_spd, drv_spd):
        
        self.rot_spd = rot_spd
        self.drv_spd = drv_spd
        
        self.dest = Pose()
        self.pose = Pose()
        self.vel = Twist()

        rospy.init_node('robot_driver', anonymous=False)

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub_dest = rospy.Subscriber('/turtle1/destination', Pose, self.dest_coord)
        self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.pose_coord)
        
        # TODO: initialise this better
        self.HT_0_d = self.HT_builder(self.z_elemental(0), np.array([0]*3).reshape(-1,1))
        
    
    def dest_coord(self, received_msg):
        self.dest = received_msg
        
        # R_0_d stands for destination(d) orientation wrt world frame(0)
        R_0_d = self.z_elemental(np.arctan2(self.dest.y, self.dest.x))
        
        # position of dest wrt to world frame
        p_0 = np.array([self.dest.x, self.dest.y, 0]).reshape(-1,1)
        self.HT_0_d = self.HT_builder(R_0_d, p_0)
        
    
    def pose_coord(self, received_msg):
        self.pose = received_msg
        self.publish_coord()
        
        
    def publish_coord(self):
        self.direction_finder()

        if abs(self.angle_t_d) > robot_driver.eps:
            self.vel.linear.x = 0
            self.vel.angular.z = self.direction
            
        else:
            self.vel.angular.z = 0
            dist2go = np.sqrt(self.HT_t_d[0,3]**2 + self.HT_t_d[1,3]**2)
            
            if dist2go > robot_driver.dist_arrived:
                self.vel.linear.x = self.drv_spd
            
            else:
                self.vel.linear.x = 0
            
        self.pub.publish(self.vel)
                
        
    def direction_finder(self):
        # self.HT_t_d_calc()
        R_0_t = self.z_elemental(self.pose.theta)
        q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
        
        self.HT_0_t = self.HT_builder(R_0_t, q_0)
        self.HT_t_d = np.dot(np.linalg.inv(self.HT_0_t), self.HT_0_d)
        
        self.angle_t_d = np.arctan2(self.HT_t_d[1,3], self.HT_t_d[0,3])
        self.direction = self.rot_spd*np.sign(self.angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        

    def z_elemental(self, gamma):            
        R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])
        
        return R_z
    
    
    def HT_builder(self, R, p):
        arr_zeros = np.array([[0, 0, 0, 1]])
        return np.vstack((np.hstack((R, p)), arr_zeros))
    
    
    # def HT_t_d_calc(self):        
    #     # R_0_t stands for turtle (t) orientation wrt world frame(0)
    #     R_0_t = self.z_elemental(self.pose.theta)
    #     q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
    #     self.HT_0_t = self.HT_builder(R_0_t, q_0)
    #     self.HT_t_d = np.dot(np.linalg.inv(self.HT_0_t), self.HT_0_d)
        

        
if __name__ == '__main__':
    rot_spd = 1
    drv_spd = 1
    
    obj = robot_driver(rot_spd, drv_spd)
    rospy.spin()
