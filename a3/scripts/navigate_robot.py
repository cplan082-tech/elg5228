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
    
    slow_zone_angle = 2 # error angle in deg where angular vel = min_vel
    slow_zone_dist = 1 # error angle in deg where angular vel = min_vel
    
    # proportional gain (K_p) for P type controller
    kp_ang = 2
    kp_lin = 0.8
    
    max_frwd_vel = 0.5
    max_ang_vel = 0.5
    
    def __init__(self):
        rospy.init_node('navigate_robot', anonymous=False)
        
        # Publisher/Subscriber object instantiation
        self.sub_sensObj = rospy.Subscriber('/sense_object', Pose2D, self.callback_sensObj)
        self.sub_husky_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_husky_pose)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.msg_sensObj = Pose2D()
        self.msg_pose_husky = Twist()
        
        # For testing vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        # self.pub_test_sensObj = rospy.Publisher('/test_sensObj', Pose2D, queue_size=10)
        # self.pub_test_husky_pose = rospy.Publisher('/test_husky_pose', Odometry, queue_size=10)
        # rospy.spin()
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        
    def callback_sensObj(self, msg):
        self.msg_sensObj = msg
        
        
    def callback_husky_pose(self, msg):
        self.msg_pose_husky = msg
        # msg_sensObj = self.msg_sensObj
        
        # orientation_husky = self.msg_pose_husky.pose.pose.orientation
        # arr_orientation_husky = np.array([orientation_husky.x,
        #                                   orientation_husky.y,
        #                                   orientation_husky.z,
        #                                   orientation_husky.w])
        
        # # numpy arr
        # arr_orientation_obj = quaternion_from_euler(0, 
        #                                         0, 
        #                                         np.deg2rad(self.msg_sensObj.theta))
        
        
        # # arr_quat_obj_husky = quaternion_multiply(arr_orientation_obj, arr_orientation_husky)
        # arr_quat_obj_husky = quaternion_multiply(arr_orientation_husky, arr_orientation_obj)
        
        # print("Robot wrt world: {}".format(np.rad2deg(euler_from_quaternion(arr_orientation_husky)[2])))
        # print("Obj wrt LiDAR: {}".format(np.rad2deg(euler_from_quaternion(arr_orientation_obj)[2])))
        # print("Robot wrt LiDAR: {}".format(np.rad2deg(euler_from_quaternion(arr_quat_obj_husky)[2])))
        # print('\n\n\n\n')
        
        # orientation_euler_husky = self.quat2euler(orientation_husky)
        

    def publish_cmd_vel(self, x, z):
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = x
        msg_cmd_vel.angular.z = z
        # msg_sensObj = self.msg_sensObj
        
        # err_angle = msg_sensObj.theta
        # direction = np.sign(err_angle)*-1
        
        # if abs(err_angle) > cls_navigate_robot.eps_ang:
        #     msg_cmd_vel.linear.x = 0
        #     msg_cmd_vel.angular.z = -1*cls_navigate_robot.kp_ang*err_angle/180
            
        # else:
        #     msg_cmd_vel.linear.x = 0
        #     msg_cmd_vel.angular.z = 0
            
        self.pub_cmd_vel.publish(msg_cmd_vel)
        
        
    def find_ang_vel(self, msg_sensObj):
        # msg_sensObj = self.msg_sensObj
        
        err_angle = msg_sensObj.theta
        direction = np.sign(err_angle)*-1
        
        if abs(err_angle) > cls_navigate_robot.eps_ang:
            angular_z = -1*cls_navigate_robot.kp_ang*err_angle/180
            
        else:
            angular_z = 0
            
        return angular_z
    
    
    def find_lin_vel(self, msg_sensObj):
        err_dist = msg_sensObj.x
        
        if err_dist > cls_navigate_robot.e:
            lin_x = cls_navigate_robot.kp_lin*err_dist
            
        else:
            lin_x = 0
            
        return lin_x
        
    
    def quat2euler(self, orientation):
        lst_quat = [orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w]
        return euler_from_quaternion(lst_quat)
    
    
    def track_obj(self):
        msg_sensObj = self.msg_sensObj        
        self.publish_cmd_vel(self.find_lin_vel(msg_sensObj), 
                             self.find_ang_vel(msg_sensObj))
        
        
if __name__ == "__main__":
    obj = cls_navigate_robot()
    while not rospy.is_shutdown():
        try:
            obj.track_obj()
        except:
            pass
        
        