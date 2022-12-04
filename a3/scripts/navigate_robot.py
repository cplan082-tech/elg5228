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
        self.msg_husky_pose = Twist()
        
        # For testing vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        # self.pub_test_sensObj = rospy.Publisher('/test_sensObj', Pose2D, queue_size=10)
        # self.pub_test_husky_pose = rospy.Publisher('/test_husky_pose', Odometry, queue_size=10)
        # rospy.spin()
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        
    def callback_sensObj(self, msg):
        self.msg_sensObj = msg
        
        
    def callback_husky_pose(self, msg):
        self.msg_husky_pose = msg
        
        
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
        
        
if __name__ == "__main__":
    obj = cls_navigate_robot()
    while not rospy.is_shutdown():
        try:
            obj.publish_cmd_vel()
        except:
            pass
        
        