#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose


# ensures input coordinates are within aceptable range
def get_coords(axis):
    while(True):
        input_val = float(input("\n{}: ".format(axis))) # converts input val to float
        
        # True if input val out of range
        if(input_val < 0) or (input_val > 11):
            print("\n{} must be between 0 and 11 inclusivley".format(axis))
            
        else: # Input val within range
            break
    return input_val


# control function
def trg_pub_run():
    
    # get desired destination coords
    print("Enter a value between 0 and 11 for both x and y")
    
    x = get_coords('x')
    y = get_coords('y')
    
    print("\nx = {} and y = {}".format(x, y)) # prints dest coords to term
    
    pose = Pose() # var used to store dest coords to be published
    pose.x = x
    pose.y = y
    
    # instantiates publisher obj
    pub = rospy.Publisher('/turtle1/destination', Pose, queue_size=10)
    rospy.init_node('target_publisher', anonymous=False)
    rate = rospy.Rate(10) # publish at 10 Hz
    
    while not rospy.is_shutdown():
        pub.publish(pose) # publish dest coords
        rate.sleep() # ensure publishing @ 2Hz
        
        
if __name__ == '__main__':
    try:
        trg_pub_run()
    except rospy.ROSInterruptException:
        pass

