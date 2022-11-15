#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
import numpy as np

class pose_reporter():
    
    def __init__(self):
        
        rospy.init_node('pose_reporter', anonymous=False)
        
        self.rate = rospy.Rate(2) # 2Hz
        
        # Instantiate subscriber obj for turtle current pose
        self.sub_pose = rospy.Subscriber('/turtle1/pose', 
                                         Pose, 
                                         self.callback_pose)
        
        # Instantiate subscriber obj for desired destination
        self.sub_dest = rospy.Subscriber('/turtle1/destination', 
                                         Pose, 
                                         self.callback_dest)
        
        # Instantiate publisher obj
        self.pub_drv_cmd = rospy.Publisher("turtle1/drv_cmd", 
                                           Pose,
                                           queue_size=10)
        
        self.pose = Pose() # Store recived turtle pose
        self.dest = Pose() # Store recieved destination coords
        self.pose_pub = Pose() # Store drv cmd's to be published
        
        # Initialise destination coords/orrientation to turtle starting pose
        self.R_0_d = self.z_elemental(0) # R_0_d => rot of dest (d) wrt world (0)
        self.p_0 = np.array([5.5, 5.5, 0]).reshape(-1,1) # vec wrt world frame (0)
        

    def callback_pose(self, received_message):
        self.pose = received_message # stores turtle current pose
    
    
    def callback_dest(self, received_message):
        self.dest = received_message # stores desired destination
        
        self.R_0_d = self.z_elemental(np.arctan2(self.dest.y, # R_0_d => rot of dest (d) wrt world (0)
                                                 self.dest.x))
        self.p_0 = np.array([self.dest.x, 
                             self.dest.y, 0]).reshape(-1,1) # vec of dest wrt world frame (0)
        
        
    def loginfo_pub(self):
        # R_t_0 ==  R_0_t^T => rot of world (0) wrt turtle (t)
        R_t_0 = self.z_elemental(self.pose.theta).T 
        
        # position of dest wrt to world frame
        q_0 = np.array([self.pose.x, self.pose.y, 0]).reshape(-1,1)
        
        HT_0_d = self.HT_builder(self.R_0_d, self.p_0) # Homo. Trans. of dest wrt world
        HT_t_0 = self.HT_builder(R_t_0, -np.dot(R_t_0, q_0)) # Homo. Trans. of world wrt turtle (inverse of HT_0_t)
        
        HT_t_d = np.dot(HT_t_0, HT_0_d) # Homo. Trans. of dest wrt turtle (HT_t_d = HT_t_0@HT_0_d)
        
        # log info
        dist2go = np.sqrt(HT_t_d[0,3]**2 + HT_t_d[1,3]**2) # distance left to go
        orient_err = np.rad2deg(np.arctan2(HT_t_d[1,3], HT_t_d[0,3])) # orrientation error
        
        self.pose_pub.x = self.pose.x # publish current x coord of turtle
        self.pose_pub.y = self.pose.y # publish current y coord of turtle
        self.pose_pub.theta = np.rad2deg(self.pose.theta) # publish current ang of turtle in deg
        
        self.pose_pub.linear_velocity = dist2go # send distance to go
        self.pose_pub.angular_velocity = orient_err # send orr err
        
        rospy.loginfo('\n\n====================Info====================')
        
        rospy.loginfo('Robot Position:\n\t\t\t\tx= %s m \n\t\t\t\ty= %s m', 
                      self.pose_pub.x,
                      self.pose_pub.y)
        rospy.loginfo('Distance between robot and destination:\n\t\t\t\t %s m', 
                      dist2go)
        rospy.loginfo("Robot's current orientation:\n\t\t\t\t %s Deg", 
                      self.pose_pub.theta)
        rospy.loginfo('Orientation error:\n\t\t\t\t %s Deg', orient_err)
        
        self.pub_drv_cmd.publish(self.pose_pub) # publish drv cmd's
        
        self.rate.sleep() # ensure pub @ 2Hz
        
    
    # creates a z rot elemental matrix
    def z_elemental(self, gamma):            
        R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])
        
        return R_z
    
    
    # builds homogeneous transformation matrices w/ rot mat and position vec
    def HT_builder(self, R, p):
        arr_zeros = np.array([[0, 0, 0, 1]])
        return np.vstack((np.hstack((R, p)), arr_zeros))


if __name__ == '__main__':
    obj = pose_reporter()
    
    while not rospy.is_shutdown():
        try:
            obj.loginfo_pub()
            
        except rospy.ROSInterruptException:
            pass
