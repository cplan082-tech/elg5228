#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class robot_driver():
    
    eps = 5e-1 # Smallest acceptable orientation error
    slow_zone_angle = 2 # error angle in deg where angular vel = min_vel
    slow_zone_dist = 1 # error angle in deg where angular vel = min_vel
    dist_arrived = 10e-2 # min distance error in cm where destination is considered reached
    
    rot_spd = 2 # gain of angular vel P controller
    drv_spd = 0.8 # gain of linear vel P controller
    min_vel_ang = 1e-2
    min_vel_lin = 0.1
    
    def __init__(self):
        
        # initialise node that must be unique
        rospy.init_node('robot_driver', anonymous=False) 
        
        self.rate = rospy.Rate(20) # frequency of cmd_vel to turtlesim bot in Hz
        
        self.drv_cmd = Pose()
        self.vel = Twist()
        
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/drv_cmd', Pose, self.callback_drv_cmd)
        
    
    def callback_drv_cmd(self, rec_msg):
        self.drv_cmd = rec_msg    
        
        
    def publish_drv_cmd(self):
        dist2go = self.drv_cmd.linear_velocity
        angle_t_d = self.drv_cmd.angular_velocity # messgae sent in deg, algo uses rad
        direction = robot_driver.rot_spd*np.sign(angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        
        if dist2go > robot_driver.dist_arrived:
            
            if abs(angle_t_d) > robot_driver.eps:
                self.vel.linear.x = 0
                u_ang_vel = angle_t_d*robot_driver.rot_spd/180 # normalised
                
                if (abs(angle_t_d) < robot_driver.slow_zone_angle) or \
                    (abs(u_ang_vel) < robot_driver.min_vel_ang):
                    self.vel.angular.z = robot_driver.min_vel_ang*direction
                else:
                    self.vel.angular.z = u_ang_vel

                
            else:
                self.vel.angular.z = 0
                u_lin_vel = dist2go*robot_driver.drv_spd
                
                if (dist2go > robot_driver.slow_zone_dist) or \
                    (u_lin_vel > robot_driver.min_vel_lin):
                    self.vel.linear.x = u_lin_vel
                
                else:
                    self.vel.linear.x = robot_driver.min_vel_lin
                
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            
        self.pub.publish(self.vel)
       

        
if __name__ == '__main__':
    
    obj = robot_driver()
    while not rospy.is_shutdown():
        try:
            obj.publish_drv_cmd()
            
        except rospy.ROSInterruptException:
            pass
        obj.rate.sleep()
