#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  9 15:20:22 2016

@author: hoarau
"""

import rospy
from std_msgs.msg import Bool,Float32
from time import time

rospy.init_node("safety_stop")

stop_zone = 1.4 # 1.4m
hysteresis = 0.2
hysteresis_time = 1.0 # 1s
oldest_ok = -1 

def distanceCB(msg):
    print msg
    if msg.data < stop_zone :
        print "STOP"
        pub.publish(True)
    elif msg.data < stop_zone + hysteresis :
        print "Neutral zone"
    else :
        print 'OK'
        pub.publish(False)         
        

sub = rospy.Subscriber("/sick_proc/min_dist_to_laser",Float32,callback=distanceCB,tcp_nodelay=True,queue_size=10)
pub = rospy.Publisher("/robot_emergency_stopped",Bool,tcp_nodelay=True,queue_size=1)
rospy.spin()
print("End")
exit()