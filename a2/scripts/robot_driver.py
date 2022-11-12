#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class robot_driver():
    
    eps = 2
    dist_arrived = 10e-2
    
    def __init__(self, rot_spd, drv_spd):
        rospy.init_node('robot_driver', anonymous=False)
        
        self.rate = rospy.Rate(20)
        
        self.rot_spd = rot_spd
        self.drv_spd = drv_spd
        
        # self.dest = Pose()
        # self.pose = Pose()
        self.drv_cmd = Pose()
        self.vel = Twist()
        
        # self.pub = rospy.Publisher('/turtle1/test', Twist, queue_size=10)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/drv_cmd', Pose, self.callback_drv_cmd)
        # self.sub_dest = rospy.Subscriber('/turtle1/destination', Pose, self.dest_coord)
        # self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.pose_coord)
        
        # self.HT_0_d = self.HT_builder(self.z_elemental(0), np.array([5.5, 5.5, 0]).reshape(-1,1))
        
    
    def callback_drv_cmd(self, rec_msg):
        self.drv_cmd = rec_msg
    
    
    # def dest_coord(self, received_msg):
    #     self.dest = received_msg
        
    #     # # R_0_d stands for destination(d) orientation wrt world frame(0)
    #     # R_0_d = self.z_elemental(np.arctan2(self.dest.y, self.dest.x))
        
    #     # # position of dest wrt to world frame
    #     # p_0 = np.array([self.dest.x, self.dest.y, 0]).reshape(-1,1)
    #     # self.HT_0_d = self.HT_builder(R_0_d, p_0)
        
    
    # def pose_coord(self, received_msg):
    #     self.pose = received_msg
    #     # self.publish_drv_cmd()
    
        
        
    def publish_drv_cmd(self):
        dist2go = self.drv_cmd.linear_velocity
        angle_t_d = self.drv_cmd.angular_velocity
        direction = self.rot_spd*np.sign(angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        # dist2go = np.sqrt(self.HT_t_d[0,3]**2 + self.HT_t_d[1,3]**2)
        
        if dist2go > robot_driver.dist_arrived:
            
            if abs(angle_t_d) > robot_driver.eps:
                self.vel.linear.x = 0
                self.vel.angular.z = direction
                
            else:
                self.vel.angular.z = 0
                self.vel.linear.x = self.drv_spd
                
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            
        self.pub.publish(self.vel)
        # self.rate.sleep()
                
        
    # def direction_finder(self):
    #     # self.HT_t_d_calc()
    #     # R_t_0 = self.z_elemental(self.pose.theta).T
    #     # q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
        
    #     # HT_0_t = self.HT_builder(R_t_0, -np.dot(R_t_0, q_0))
    #     # self.HT_t_d = np.dot(HT_0_t, self.HT_0_d)
        
    #     # self.angle_t_d = np.arctan2(self.HT_t_d[1,3], self.HT_t_d[0,3])
    #     self.direction = self.rot_spd*np.sign(self.angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        

    # def z_elemental(self, gamma):            
    #     R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
    #                     [np.sin(gamma), np.cos(gamma), 0],
    #                     [0, 0, 1]])
        
    #     return R_z
    
    
    # def HT_builder(self, R, p):
    #     arr_zeros = np.array([[0, 0, 0, 1]])
    #     return np.vstack((np.hstack((R, p)), arr_zeros))
        
    
    
    # def HT_t_d_calc(self):        
    #     # R_0_t stands for turtle (t) orientation wrt world frame(0)
    #     R_0_t = self.z_elemental(self.pose.theta)
    #     q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
    #     self.HT_0_t = self.HT_builder(R_0_t, q_0)
    #     self.HT_t_d = np.dot(np.linalg.inv(self.HT_0_t), self.HT_0_d)
        

        
if __name__ == '__main__':
    rot_spd = 0.2
    drv_spd = 1
    
    obj = robot_driver(rot_spd, drv_spd)
    while not rospy.is_shutdown():
        try:
            obj.publish_drv_cmd()
            
        except rospy.ROSInterruptException:
            pass
        obj.rate.sleep()
    # rospy.spin()
