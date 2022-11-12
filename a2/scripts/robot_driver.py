#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class robot_driver():
    
    eps = 7e-1
    min_vel = 5e-2
    dist_arrived = 10e-2
    
    def __init__(self, rot_spd, drv_spd):
        rospy.init_node('robot_driver', anonymous=False)
        
        self.rate = rospy.Rate(20)
        
        self.rot_spd = rot_spd
        self.drv_spd = drv_spd
        
        self.drv_cmd = Pose()
        self.vel = Twist()
        
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/drv_cmd', Pose, self.callback_drv_cmd)
        
    
    def callback_drv_cmd(self, rec_msg):
        self.drv_cmd = rec_msg    
        
        
    def publish_drv_cmd(self):
        dist2go = self.drv_cmd.linear_velocity
        angle_t_d = self.drv_cmd.angular_velocity
        direction = self.rot_spd*np.sign(angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        
        if dist2go > robot_driver.dist_arrived:
            
            if abs(angle_t_d) > robot_driver.eps:
                self.vel.linear.x = 0
                # self.vel.angular.z = direction
                angular_vel = angle_t_d*self.rot_spd/180
                
                if (abs(angular_vel) < robot_driver.min_vel):
                    self.vel.angular.z = robot_driver.min_vel*direction
                    
                else:
                    self.vel.angular.z = angular_vel
                
            else:
                self.vel.angular.z = 0
                # self.vel.linear.x = self.drv_spd
                self.vel.linear.x = dist2go
                
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            
        self.pub.publish(self.vel)
       

        
if __name__ == '__main__':
    rot_spd = 1
    drv_spd = 1
    
    obj = robot_driver(rot_spd, drv_spd)
    while not rospy.is_shutdown():
        try:
            obj.publish_drv_cmd()
            
        except rospy.ROSInterruptException:
            pass
        obj.rate.sleep()
    # rospy.spin()
