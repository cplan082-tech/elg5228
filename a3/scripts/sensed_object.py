#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan

class cls_sense_object():
    
    def __init__(self):
        # initialise "robot_driver" node that must be unique
        rospy.init_node('sense_object', anonymous=False) 
        
        # Publisher/Subscriber object instantiation
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/sense_object', Pose2D, queue_size=10)
        
        self.msg_lidar = LaserScan() # subed msg
        
        self.rate = rospy.Rate(2) # publish frequency in Hz
        
        
    def callback(self, msg):
        self.msg_lidar = msg
        
    def pub_direction_heading(self):
        msg_direction_heading = Pose2D() # published msg
        msg_direction_heading.y = float('NaN')
        msg = self.msg_lidar
        msg_rng = msg.ranges
        
        arr_rng = np.array(msg_rng)
        arr_rng[arr_rng > msg.range_max] = float('NaN')
        arr_rng[arr_rng < msg.range_min] = float('NaN')
        
        # Test if within range. If not, send 'nan'
        if not (np.isnan(arr_rng)).all():
            msg_direction_heading.x = np.nanmin(arr_rng)
            idx = np.reshape(np.argwhere(arr_rng == msg_direction_heading.x), -1) # convert 2dim arr to 1 dim arr
            msg_direction_heading.theta = np.rad2deg(idx*msg.angle_increment + msg.angle_min)       
            
        else:
            msg_direction_heading.x = float('NaN')
            msg_direction_heading.theta = float('NaN')
            
        rospy.loginfo("Min dist. = %7.3f m, Angles = %7.3f deg", 
                      msg_direction_heading.x, 
                      msg_direction_heading.theta) 
        self.pub.publish(msg_direction_heading)
        self.rate.sleep()



if __name__ == "__main__":
    
    obj = cls_sense_object()
    while not rospy.is_shutdown():
        try:
            obj.pub_direction_heading()
        except:
            pass
    