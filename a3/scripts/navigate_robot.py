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
    kp_ang = 2
    kp_lin = 0.5
    
    max_frwd_vel = kp_ang
    min_frwd_vel = 0.05
    
    max_ang_vel = 3
    min_ang_vel = 0.2
    
    def __init__(self):
        rospy.init_node('navigate_robot', anonymous=False)
        
        # Publisher/Subscriber object instantiation
        self.sub_sensObj = rospy.Subscriber('/sense_object', Pose2D, self.callback_sensObj)
        self.sub_husky_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_husky_pose)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.msg_sensObj = Pose2D()
        self.msg_pose_husky = Twist()
        
        self.track_obj_phase = True
        self.mv2circ_pos_phase = False
        self.cir_orientation_set_phase = False
        
        
        
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
        # msg_sensObj = self.msg_sensObj
        # err_dist = msg_sensObj.x 
        # err_angle = msg_sensObj.theta
        
        err_dist, err_angle = self.get_obj_err()
        
        if err_dist < cls_navigate_robot.e:
            lin_vel_x = 0
            ang_vel_z = 0
            self.track_obj_phase = False
            self.mv2circ_pos_phase = True
            # self.update_target() # TODO: Remove once done testing
            
        elif abs(err_angle) > cls_navigate_robot.zone_frwd_angle:
            lin_vel_x = 0
            ang_vel_z = self.find_ang_vel(err_angle)
        
        elif abs(err_angle) < cls_navigate_robot.eps_ang:
            lin_vel_x = self.find_lin_vel(err_dist)
            ang_vel_z = 0
            
        else:
            lin_vel_x = self.find_lin_vel(err_dist)
            ang_vel_z = self.find_ang_vel(err_angle)
            
        self.publish_cmd_vel(lin_vel_x, ang_vel_z)
        
        
        
    def update_target(self):
        msg_pose_husky = self.msg_pose_husky
        self.husky_trg_position = msg_pose_husky.pose.pose.position
        self.husky_trg_orientation = msg_pose_husky.pose.pose.orientation
        
    
    
    def dist_from_target(self):
        msg_pose_husky = self.msg_pose_husky
        husky_pres_position = msg_pose_husky.pose.pose.position
        
        del_x = husky_pres_position.x - self.husky_trg_position.x
        del_y = husky_pres_position.y - self.husky_trg_position.y
        
        dist = np.sqrt(del_x**2 + del_y**2)
        
        return dist
    
    
    
    def move2circ_position(self):
        err_dist, err_angle = self.get_obj_err()
        
        # 0.143 m is dist between lsr and front bumper
        # 0.337074 m is dist between lsr and outside left wheel
        if err_dist < cls_navigate_robot.eps_dist + 0.337074: 
            lin_vel_x = 0
            self.mv2circ_pos_phase = False
            self.cir_orientation_set_phase = True
            
        else:
            lin_vel_x = cls_navigate_robot.min_frwd_vel
        
        self.publish_cmd_vel(lin_vel_x, 0)
    
    
    def get_obj_err(self):
        msg_sensObj = self.msg_sensObj
        
        return msg_sensObj.x, msg_sensObj.theta


    def vel_check(self, vel, min_vel, max_vel):
        if vel > max_vel:
            vel = max_vel
            
        elif vel < min_vel:
            vel = min_vel
            
        return vel
    
    
    
    def process_sequencer(self):
        if self.track_obj_phase:
            self.track_obj()
            
        elif self.mv2circ_pos_phase:
            self.move2circ_position()
            
        elif self.cir_orientation_set_phase:
            # for testing vvvvvvvvvvvvvvvvvvvvvvvvvvv
            print('here')
            self.cir_orientation_set_phase = False
            self.track_obj_phase = True
            # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        else:
            pass
        
        
        
if __name__ == "__main__":
    obj = cls_navigate_robot()
    while not rospy.is_shutdown():
        try:
            obj.process_sequencer()
        except:
            pass
        
        