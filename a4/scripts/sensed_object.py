# -*- coding: utf-8 -*-
"""
Created on Fri Dec  9 11:08:15 2022

@author: clive
"""

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

class cls_sense_object():
    
    el = 1 # saftey dist. between LiDAR and boundary (in m)
    e_max = 0.3 # in m
    k_p = 1
    
    
    def __init__(self):
        # initialise "robot_driver" node that must be unique
        rospy.init_node('sense_object', anonymous=False) 
        
        # Publisher/Subscriber object instantiation
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/sense_object', Pose, queue_size=10)
        
        self.msg_lidar = LaserScan() # subed msg
        
        self.rate = rospy.Rate(10) # publish frequency in Hz
        
        
        def callback(self, msg):
            self.msg_lidar = msg
            
            
        def get_rng_arr(self):
            msg_lidar = self.msg_lidar
            msg_rng = msg_lidar.ranges
            
            arr_rng = np.array(msg_rng)
            arr_rng[arr_rng > msg_lidar.range_max] = float('NaN')
            arr_rng[arr_rng < msg_lidar.range_min] = float('NaN')
            
            return arr_rng, msg_lidar
        
        
        # gets "d" and "alpha"
        def get_direction_heading(self, arr_rng, msg_lidar):            
            # find heading of object
            self.msg_direction_heading.position.y = float('NaN')            
            self.msg_direction_heading.position.x = np.nanmin(arr_rng)
            
            # Find direction of obj
            idx = np.reshape(np.argwhere(arr_rng == self.msg_direction_heading.position.x), -1) # convert 2dim arr to 1 dim arr
            self.msg_direction_heading.orientation.z = np.rad2deg(idx*msg_lidar.angle_increment + msg_lidar.angle_min) 
            
            
        def calc_err(self):
            d = self.msg_direction_heading.position.x
            
            err = d - cls_sense_object.el
            err_nrm = err/cls_sense_object.e_max
            
            # True if max err WAS exceeded
            if abs(err) > cls_sense_object.e_max:
                err_nrm = np.sing(err_nrm) # return -1 or 1
                
            self.msg_direction_heading.position.y = err
            self.msg_direction_heading.position.z = err_nrm
            
            self.pub.publish(self.msg_direction_heading)
            self.rate.sleep()
            
        
        def pub_sensed_obj(self):
            # TODO: change from type Pose2D() to type Pose()
            self.msg_direction_heading = Pose() # resets msg 2 be published every iter.
            
            arr_rng, msg_lidar = self.get_rng_arr()
            self.get_direction_heading(arr_rng, msg_lidar)
            self.calc_err()
            
            err_nrm = self.msg_direction_heading.position.z
            
            self.msg_direction_heading.orientation.x = cls_sense_object.k_p*err_nrm
            self.msg_direction_heading.orientation.y = cls_sense_object.k_p*(1 - abs(err_nrm))
            
            
if __name__ == "__main__":
    obj = cls_sense_object()
    while not rospy.is_shutdown():
        try:
            obj.pub_sensed_obj()
        except:
            pass