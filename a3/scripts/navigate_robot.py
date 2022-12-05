#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Pose2D, Twist


class cls_navigate_robot():
    e = 0.7
    e_buffr = e + 5e-1
    
    eps_ang = 5
    eps_circ_ang = 2
    eps_dist = 1e-1
    
    zone_frwd_angle = 15 # error angle in deg where angular vel = min_vel
    # slow_zone_dist = 1 # error angle in deg where angular vel = min_vel
    
    # proportional gain (K_p) for P type controller
    kp_ang = 2
    kp_lin = 0.5
    
    kp_ang_circ = 3
    ki_ang_circ = 0.05
    
    max_frwd_vel = kp_lin
    min_frwd_vel = 0.05
    
    max_ang_vel = 3
    min_ang_vel = 0.1
    
    circ_lin_vel = 0.2
    circ_ang_vel = 0.5
    
    ang_err_offset = 90
    
    def __init__(self):
        rospy.init_node('navigate_robot', anonymous=False)
        
        # Homogeneous transform for base_laser wrt base_link
        p_base = np.array([[0.337], [0], [0.307]])
        
        R_link_laser = x_elemental(180)
        self.HT_link_laser = HT_builder(R_link_laser, p_base)
        
        # Publisher/Subscriber object instantiation
        self.sub_sensObj = rospy.Subscriber('/sense_object', Pose2D, self.callback_sensObj)
        self.sub_husky_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_husky_pose)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.msg_sensObj = Pose2D()
        self.msg_pose_husky = Twist()
        
        self.sum_err_dist = 0
        
        self.phase = 'track_obj'
        
        self.rate = rospy.Rate(10) # publish frequency in Hz
        
        
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
        
        err_dist, err_angle = self.get_obj_err()
        
        # obstacle reached
        if err_dist < cls_navigate_robot.e:
            lin_vel_x = 0
            ang_vel_z = 0
            self.phase = 'move2circ_position'
            
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
    
    
    def find_q_base(self, err_dist, err_angle):
        x = np.cos(np.deg2rad(err_angle))*err_dist
        y = np.sin(np.deg2rad(err_angle))*err_dist
        q_laser = np.array([[x], [y], [0]])
        R_laser_obj = z_elemental(err_angle)
        HT_laser_obj = HT_builder(R_laser_obj, q_laser)
        
        HT_base_obj = np.dot(self.HT_link_laser, HT_laser_obj)
        R_base_obj, q_base = T_extrct_Rp(HT_base_obj)
        
        q_base_x = q_base[0][0]
        q_base_y = q_base[1][0]
        
        return q_base_x, q_base_y
    
    
    def move2circ_position(self):
        err_dist, err_angle = self.get_obj_err()
        
        # ensures cir_phase was not accidently engaged
        if err_dist > cls_navigate_robot.e_buffr:
            self.phase = 'track_obj'
        
        # 0.143 m is dist between lsr and front bumper
        # 0.337074 m is dist between lsr and outside left wheel
        elif err_dist < cls_navigate_robot.eps_dist + 0.337074: 
            lin_vel_x = 0
            self.phase = 'cir_orientation_set'
            
        else:
            lin_vel_x = cls_navigate_robot.min_frwd_vel
        
        self.publish_cmd_vel(lin_vel_x, 0)
        
        
    def cir_orientation_set(self):
        err_dist, err_angle = self.get_obj_err()
        q_base_x, q_base_y = self.find_q_base(err_dist, err_angle)
        
        err_ang_circ = np.rad2deg(np.arctan2(q_base_y, q_base_x)) + cls_navigate_robot.ang_err_offset
        
        # ensures cir_phase was not accidently engaged
        if err_dist > cls_navigate_robot.e_buffr:
            self.phase = 'track_obj' 
        
        elif abs(err_ang_circ) > cls_navigate_robot.eps_circ_ang:
            ang_vel_z = -1*np.sign(self.find_ang_vel(err_ang_circ))*cls_navigate_robot.circ_ang_vel
            
        else:
            ang_vel_z = 0
            if self.phase == 'cir_orientation_set':
                self.trg_dist = err_dist
                self.update_target()
                self.bigger_than_eps = False
        
        return ang_vel_z
        
    
    def circumvent(self):
        err_dist, err_angle = self.get_obj_err()
        q_base_x, q_base_y = self.find_q_base(err_dist, err_angle)
        dist2obj = np.sqrt(q_base_x**2 + q_base_y**2)
        
        err_dis2obj = self.trg_dist - err_dist
        
        # return to tracking obstacle
        if err_dist > cls_navigate_robot.e_buffr:
            self.phase = 'track_obj'
        
        elif self.dist_from_target() > cls_navigate_robot.eps_dist:
            
            # means we just got far enough from the target to start tracking task
            if not self.bigger_than_eps: 
                self.bigger_than_eps = True
                
            ang_vel_z = err_dis2obj*cls_navigate_robot.kp_ang_circ + \
                self.sum_err_dist*cls_navigate_robot.ki_ang_circ + \
                    self.cir_orientation_set()
                    
            lin_vel_x = cls_navigate_robot.circ_lin_vel
            
        else:
            
            if self.bigger_than_eps: # checks if we are starting circ
                ang_vel_z = 0
                lin_vel_x = 0
                print("arrived/n/n")
                self.phase = 'reset'
                
            else:
                ang_vel_z = err_dis2obj*cls_navigate_robot.kp_ang_circ + \
                    self.sum_err_dist*cls_navigate_robot.ki_ang_circ + \
                        self.cir_orientation_set()
                        
                lin_vel_x = cls_navigate_robot.circ_lin_vel
        
        self.sum_err_dist += err_dis2obj
            
        print(self.dist_from_target())
        self.publish_cmd_vel(lin_vel_x, ang_vel_z)
            
        
        
    
    def process_sequencer(self):
        
        if self.phase == 'track_obj':
            self.track_obj()
            print('track_obj_phase') # TODO: Remove once done testin
            
        elif self.phase == 'move2circ_position':
            self.move2circ_position()
            print('mv2circ_pos_phase') # TODO: Remove once done testin
            
        elif self.phase == 'cir_orientation_set':
            ang_vel_z = self.cir_orientation_set()
            
            # z == 0 if orientated properly
            if ang_vel_z == 0:   
                self.phase = 'circumvent_phase'
                
            self.publish_cmd_vel(0, ang_vel_z)
            print('cir_orientation_set_phase') # TODO: Remove once done testin
        
        elif self.phase == 'circumvent_phase':
            self.circumvent()
            print('circumvent_phase') # TODO: Remove once done testin
            
        elif self.phase == 'reset':
            self.sum_err_dist = 0
            self.phase = 'track_obj'
            
        self.rate.sleep()
        
        
    def get_obj_err(self):
        msg_sensObj = self.msg_sensObj
        
        return msg_sensObj.x, msg_sensObj.theta
        
        
    def vel_check(self, vel, min_vel, max_vel):
        if vel > max_vel:
            vel = max_vel
            
        elif vel < min_vel:
            vel = min_vel
            
        return vel
    

def x_elemental(alpha, deg=True, rnd=3):
    if deg:
        alpha = np.deg2rad(alpha)
        
    R_x = np.array([[1, 0, 0],
                  [0 ,np.cos(alpha), -np.sin(alpha)],
                  [0, np.sin(alpha), np.cos(alpha)]])
    
    R_x = np.around(R_x, decimals=rnd) # round all elements of array R_x
    
    return R_x


def z_elemental(gamma, deg=True, rnd=3):
    if deg:
        gamma = np.deg2rad(gamma)
        
    R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                    [np.sin(gamma), np.cos(gamma), 0],
                    [0, 0, 1]])
    
    R_z = np.around(R_z, decimals=rnd) # round all elements of array R_z
    
    return R_z


def HT_builder(R, p):
    arr_zeros = np.array([[0, 0, 0, 1]])
    
    return np.vstack((np.hstack((R, p)), arr_zeros))


def T_extrct_Rp(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3].reshape(-1,1)
    return R,p
    
    
        
if __name__ == "__main__":
    obj = cls_navigate_robot()
    while not rospy.is_shutdown():
        try:
            obj.process_sequencer()
        except:
            pass
        
        