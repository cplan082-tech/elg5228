#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class robot_driver():
    
    eps = 1e-3
    
    def __init__(self, rot_spd, drv_spd):
        
        self.rot_spd = rot_spd
        self.drv_spd = drv_spd
        
        self.dest = Pose()
        self.pose = Pose()
        # self.vel = Twist()
        self.vel = Pose() # TODO: remove when done testing
        self.angle = None # TODO: might not need

        rospy.init_node('robot_driver', anonymous=False)

        self.pub = rospy.Publisher('/turtle1/test', Pose, queue_size=10) # TODO: delete this after test
        # self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # TODO: uncomment this after test
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
        self.publish_test() # TODO: Remove after test
        
        
    def publish_test(self):
        self.direction_finder()
        self.vel.angular_velocity = self.direction
        self.vel.theta = self.angle_t_d
        self.pub.publish(self.vel)

        """
        use homogeneous matrices
        """
        
    def HT_t_d_calc(self):        
        # R_0_t stands for turtle (t) orientation wrt world frame(0)
        R_0_t = self.z_elemental(self.pose.theta)
        q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
        self.HT_0_t = self.HT_builder(R_0_t, q_0)
        self.HT_t_d = np.dot(np.linalg.inv(self.HT_0_t), self.HT_0_d)
        
        # self.R_t_d = np.dot(self.R_0_t.T, self.R_0_d)
        
    def rot_t_d_angle(self):
        self.HT_t_d_calc()
        self.angle_t_d = np.arctan2(self.HT_t_d[1,3], self.HT_t_d[0,3])
        
        
    def direction_finder(self):
        self.rot_t_d_angle()
        self.direction = self.rot_spd*np.sign(self.angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        # self.rot_drvr() # TODO: uncomment
        
    # def rot_drvr(self):
    #     while(abs(self.angle_t_d) > robot_driver.eps):
    #         self.turtle_drvr()
    #         self.rot_t_d_angle()
    
    # def turtle_drvr(self):
    #     pass
        
    def z_elemental(self, gamma):            
        R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])
        
        return R_z
    
    
    def HT_builder(self, R, p):
        arr_zeros = np.array([[0, 0, 0, 1]])
        
        return np.vstack((np.hstack((R, p)), arr_zeros))
    
    # def remaining_dist(self):
    #     self.x_remaining = self.pose.x - self.dest.x
    #     self.y_remaining = self.pose.y - self.dest.y
        
if __name__ == '__main__':
    rot_spd = 0.1
    drv_spd = 0.2
    
    obj = robot_driver(rot_spd, drv_spd)
    rospy.spin()
