#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Pose2D, Twist


class cls_navigate_robot():
    e = 0.7
    
    eps_ang = 5
    eps_dist = 1e-1
    
    zone_frwd_angle = 15 # error angle in deg where angular vel = min_vel
    slow_zone_dist = 1 # error angle in deg where angular vel = min_vel
    
    # proportional gain (K_p) for P type controller
    kp_ang = 1
    kp_lin = 0.5
    
    max_frwd_vel = kp_lin
    min_frwd_vel = 0.05
    
    max_ang_vel = kp_ang
    min_ang_vel = 0.1
    
    def __init__(self):
        rospy.init_node('navigate_robot', anonymous=False)
        
        # Publisher/Subscriber object instantiation
        self.sub_sensObj = rospy.Subscriber('/sense_object', Pose2D, self.callback_sensObj)
        self.sub_husky_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_husky_pose)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.msg_sensObj = Pose2D()
        self.msg_pose_husky = Twist()
        
        
        
    def callback_sensObj(self, msg):
        self.msg_sensObj = msg
        
        
        
    def callback_husky_pose(self, msg):
        self.msg_pose_husky = msg
       
        

    def publish_cmd_vel(self, x, z):
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = x
        msg_cmd_vel.angular.z = z
            
        self.pub_cmd_vel.publish(msg_cmd_vel)
        
        
        
    def find_ang_vel(self, err_angle):       
        direction = np.sign(err_angle)*-1
        ang_vel = self.vel_check(abs(cls_navigate_robot.kp_ang*err_angle/180), 
                                 cls_navigate_robot.min_ang_vel,
                                 cls_navigate_robot.max_ang_vel)
          
        return direction*ang_vel
    
    
    
    def find_lin_vel(self, err_dist):  
        lin_vel = self.vel_check(cls_navigate_robot.kp_lin*err_dist,
                                 cls_navigate_robot.min_frwd_vel,
                                 cls_navigate_robot.max_frwd_vel)
        return lin_vel
        
    
    
    def quat2euler(self, orientation):
        lst_quat = [orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w]
        return euler_from_quaternion(lst_quat)
    
    
    
    def track_obj(self):
        msg_sensObj = self.msg_sensObj
        err_dist = msg_sensObj.x 
        err_angle = msg_sensObj.theta
        
        if err_dist < cls_navigate_robot.e:
            lin_vel_x = 0
            ang_vel_z = 0
            # print('if\n\n\n')
        
        elif abs(err_angle) > cls_navigate_robot.zone_frwd_angle:
            lin_vel_x = 0
            ang_vel_z = self.find_ang_vel(err_angle)
            
        else:
            lin_vel_x = self.find_lin_vel(err_dist)
            
            ang_vel_z = self.find_ang_vel(err_angle)
            
        self.publish_cmd_vel(lin_vel_x, ang_vel_z)
        
    
    def vel_check(self, vel, min_vel, max_vel):
        if vel > max_vel:
            vel = max_vel
            
        elif vel < min_vel:
            vel = min_vel
            
        return vel
        
        
        
if __name__ == "__main__":
    obj = cls_navigate_robot()
    while not rospy.is_shutdown():
        try:
            obj.track_obj()
        except:
            pass
        
        