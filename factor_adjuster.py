#!/usr/bin/env python
import sys
import os
import rospy
import signal
from std_msgs.msg import Empty, String

from datetime import datetime



def main():
    """Start the ROS node, create the publishers, and continuosly update and
    publish the IR sensor data"""
    
    # ROS Setup
    ###########
    rospy.init_node("factadjuster")
    
    
    # Publishers
    ############
    puber = rospy.Publisher("/simulation/simfact", String, queue_size=1, tcp_nodelay=False)
    r = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        x = ""
        try:
            x = raw_input()
        except NameError:
            x = input()
            
        msg = String()
        msg.data = x
        puber.publish(msg)
        r.sleep()


if __name__ == "__main__":
    main()

