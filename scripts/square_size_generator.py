#!/usr/bin/env python3

import rospy
import random
from ros_cartesian_space_movements.msg import square_size

def square_size_generator():
    #init new topic
    pub = rospy.Publisher('size', square_size, queue_size=0)
    #init new node
    rospy.init_node('square_size_generator', anonymous=True)
    #run once every 20 seconds
    rate = rospy.Rate(1/20)
    #init new message
    msg = square_size()
    #loop until the node is shutdown
    while not rospy.is_shutdown():
        #make the message a random value between 0.05 and 0.20
        msg.size = random.uniform(0.05, 0.20)
        #log and publish
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException:
        pass
