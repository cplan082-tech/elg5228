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
    
    # proportional gain (K_p) for P type controller
    kp_ang = 2
    kp_lin = 0.8
    
    # minimum allowable velocities
    min_vel_ang = 1e-2
    min_vel_lin = 0.2
    
    def __init__(self):
        
        # initialise "robot_driver" node that must be unique
        rospy.init_node('robot_driver', anonymous=False) 
        
        self.rate = rospy.Rate(20) # frequency of cmd_vel to turtlesim bot in Hz
        
        self.drv_cmd = Pose() # Recevied drive commands
        self.vel = Twist() # Published Drive commands
        
        # Publisher/Subscriber object instantiation
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/drv_cmd', Pose, self.callback_drv_cmd)
        
    
    def callback_drv_cmd(self, rec_msg):
        self.drv_cmd = rec_msg # Stores drive commands used in publish_drv_cmd()
        
    
    # Publishes drive commands for turtlesim @ 2Hz
    def publish_drv_cmd(self):
        dist2go = self.drv_cmd.linear_velocity
        angle_t_d = self.drv_cmd.angular_velocity # angle or destination (d) wrt TurtleSim (t)
        direction = robot_driver.kp_ang*np.sign(angle_t_d) # if angle is -tiv, must turn CCW (+tiv rot)
        
        # checks if destination has been reached
        if dist2go > robot_driver.dist_arrived: 
            
            # Checks if orientation error is small enough to start frwd
            if abs(angle_t_d) > robot_driver.eps: # turtle must continue to rotate
                self.vel.linear.x = 0 # prevents frwd motion
                u_ang_vel = angle_t_d*robot_driver.kp_ang/180 # normalised orrientation err
                
                # prevents ang vel from being too slow/fast when orr err is low
                if (abs(angle_t_d) > robot_driver.slow_zone_angle) or \
                    (abs(u_ang_vel) > robot_driver.min_vel_ang):
                    self.vel.angular.z = u_ang_vel
                
                # set ang vel to min_vel_ang if orientation in slow zone OR control sig too small
                else:
                    self.vel.angular.z = robot_driver.min_vel_ang*direction

                
            else: # turtle can start moving frwd
                self.vel.angular.z = 0
                u_lin_vel = dist2go*robot_driver.kp_lin
                
                # Prevents turtle from going too slow
                if (dist2go > robot_driver.slow_zone_dist) or \
                    (u_lin_vel > robot_driver.min_vel_lin):
                    self.vel.linear.x = u_lin_vel
                
                else:
                    self.vel.linear.x = robot_driver.min_vel_lin
                
        else: # destination reached
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            
        self.pub.publish(self.vel)
        self.rate.sleep()
       

        
if __name__ == '__main__':
    
    obj = robot_driver()
    while not rospy.is_shutdown():
        try:
            obj.publish_drv_cmd()
            
        except rospy.ROSInterruptException:
            pass
