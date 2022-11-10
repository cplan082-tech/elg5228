#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose



def get_coords(axis):
    while(True):
        input_val = int(input("\n{}: ".format(axis)))
        
        if(input_val < 0) or (input_val > 11):
            print("\n{} must be between 0 and 11 inclusivley".format(axis))
            
        else:
            break
    return input_val



def trg_pub_run():
    
    print("Enter a value between 0 and 11 for both x and y")
    
    x = get_coords('x')
    y = get_coords('y')
    
    print("\nx = {} and y = {}".format(x, y))
    
    pose = Pose()
    pose.x = x
    pose.y = y
    
    pub = rospy.Publisher('/turtle1/destination', Pose, queue_size=10)
    rospy.init_node('target_publisher', anonymous=False)
    rate = rospy.Rate(10) # publish at 10 Hz
    
    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()
        
        
if __name__ == '__main__':
    try:
        trg_pub_run()
    except rospy.ROSInterruptException:
        pass

