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
        msg_sensObj = self.msg_sensObj
        
        orientation_husky = self.msg_pose_husky.pose.pose.orientation
        arr_orientation_husky = np.array([orientation_husky.x,
                                          orientation_husky.y,
                                          orientation_husky.z,
                                          orientation_husky.w])
        
        # numpy arr
        arr_orientation_obj = quaternion_from_euler(0, 
                                                0, 
                                                np.deg2rad(self.msg_sensObj.theta))
        
        # arr_orientation_test = quaternion_from_euler(0, 
        #                                         0, 
        #                                         np.pi-2)
        
        # arr_quat_obj_husky = quaternion_multiply(arr_orientation_obj, arr_orientation_husky)
        arr_quat_obj_husky = quaternion_multiply(arr_orientation_husky, arr_orientation_obj)
        
        print("Robot wrt world: {}".format(np.rad2deg(euler_from_quaternion(arr_orientation_husky)[2])))
        print("Obj wrt LiDAR: {}".format(np.rad2deg(euler_from_quaternion(arr_orientation_obj)[2])))
        print("Robot wrt LiDAR: {}".format(np.rad2deg(euler_from_quaternion(arr_quat_obj_husky)[2])))
        print('\n\n\n\n')
        
        # print(euler_from_quaternion(arr_quat_obj_husky))
        # print(type(arr_orientation_obj))
        # print(arr_orientation_obj)
        
        orientation_euler_husky = self.quat2euler(orientation_husky)
        # print(type(np.array(orientation_euler_husky)))
        # print(orientation_euler_husky[2])
        

    def publish_cmd_vel(self):
        msg_cmd_vel = Twist()
        msg_sensObj = self.msg_sensObj
        
        angle_err = msg_sensObj.theta
        direction = np.sign(angle_err)*-1
        
        if abs(angle_err) > cls_navigate_robot.eps_ang:
            msg_cmd_vel.linear.x = 0
            msg_cmd_vel.angular.z = -1*cls_navigate_robot.kp_ang*angle_err/180
            
        else:
            msg_cmd_vel.linear.x = 0
            msg_cmd_vel.angular.z = 0
            
        self.pub_cmd_vel.publish(msg_cmd_vel)
        
    
    def quat2euler(self, orientation):
        lst_quat = [orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w]
        return euler_from_quaternion(lst_quat)
        
        
if __name__ == "__main__":
    obj = cls_navigate_robot()
    while not rospy.is_shutdown():
        try:
            obj.publish_cmd_vel()
        except:
            pass
        
        