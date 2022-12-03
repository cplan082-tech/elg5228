#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan


def callback(msg):
    msg_rng = msg.ranges
    
    arr_rng = np.array(msg_rng)
    
    arr_rng[arr_rng > msg.range_max] = float('NaN')
    arr_rng[arr_rng < msg.range_min] = float('NaN')
    
    min_dist = np.nanmin(arr_rng)
    idx = np.argwhere(arr_rng == min_dist).reshape(-1) # convert 2dim arr to 1 dim arr
    rospy.loginfo("Min dist. = %7.3f, Angles = %7.3f", 
                  min_dist, 
                  np.rad2deg(idx*msg.angle_increment) + msg.angle_min)
    
    
def lsr_scn_rd():
    rospy.init_node('sensed_object', anonymous=False)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
    
    
    

if __name__ == "__main__":
    lsr_scn_rd()