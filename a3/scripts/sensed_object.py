#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan


class sense_object():
    
    def __init__(self):
        # initialise "robot_driver" node that must be unique
        rospy.init_node('sense_object', anonymous=False) 
        
        self.rate = rospy.Rate(2) # publish frequency in Hz
        
        self.msg_lidar = LaserScan() # subed msg
        
        # Publisher/Subscriber object instantiation
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/sense_object', Pose2D, queue_size=10)
        
        
    def callback(self, msg):
        self.msg_lidar = msg
        
    def pub_direction_heading(self):
        msg_direction_heading = Pose2D() # published msg
        msg = self.msg_lidar
        msg_rng = msg.ranges
        
        arr_rng = np.array(msg_rng)
        
        arr_rng[arr_rng > msg.range_max] = float('NaN')
        arr_rng[arr_rng < msg.range_min] = float('NaN')
        
        msg_direction_heading.x = np.nanmin(arr_rng)
        idx = np.reshape(np.argwhere(arr_rng == msg_direction_heading.x), -1) # convert 2dim arr to 1 dim arr
        msg_direction_heading.theta = np.rad2deg(idx*msg.angle_increment + msg.angle_min)       
        

        rospy.loginfo("Min dist. = %7.3f, Angles = %7.3f", 
                      msg_direction_heading.x, 
                      msg_direction_heading.theta) 
        
        self.pub.publish(msg_direction_heading)
        self.rate.sleep()



if __name__ == "__main__":
    
    obj = sense_object()
    while not rospy.is_shutdown():
        try:
            obj.pub_direction_heading()
        except:
            pass
    
# Functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
# def callback(msg):
#     msg_rng = msg.ranges
    
#     arr_rng = np.array(msg_rng)
    
#     arr_rng[arr_rng > msg.range_max] = float('NaN')
#     arr_rng[arr_rng < msg.range_min] = float('NaN')
    
#     min_dist = np.nanmin(arr_rng)
#     idx = np.reshape(np.argwhere(arr_rng == min_dist), -1) # convert 2dim arr to 1 dim arr

#     rospy.loginfo("Min dist. = %7.3f, Angles = %7.3f", 
#                   min_dist, 
#                   np.rad2deg(idx*msg.angle_increment + msg.angle_min))
    
    
# def lsr_scn_rd():
#     rospy.init_node('sensed_object', anonymous=False)
#     rospy.Subscriber('/scan', LaserScan, callback)
#     rospy.spin()
    
    
    

# if __name__ == "__main__":
#     lsr_scn_rd()