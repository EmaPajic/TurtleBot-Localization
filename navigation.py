#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 22 16:50:27 2018

@author: user
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Navigator:
    def __init__(self):
        self.scan_msg = None
    def callback(self, msg):
        self.scan_msg = msg
    def navigation(self):
        cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
        safety_vel = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size = 1)
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber('scan', LaserScan, self.callback)
        rate = rospy.Rate(25) # 25hz
        while not rospy.is_shutdown():
            if self.scan_msg is not None:
                for r in self.scan_msg.ranges:
                    if r > self.scan_msg.range_min and r < self.scan_msg.range_max:
                        if r < max(self.scan_msg.range_min + 0.3, 0.3):
                            turn_cmd = Twist()
                            turn_cmd.angular.z = np.radians(180)
                            safety_vel.publish(turn_cmd)
                            rate.sleep()
                            break
            move_cmd = Twist()
            move_cmd.linear.x = np.random.uniform(0,1)
            cmd_vel.publish(move_cmd)
            rate.sleep()
            if np.random.uniform(0,1) < 0.4: 
                turn_cmd = Twist()
                turn_cmd.angular.z = np.radians(np.random.uniform(-360,360))
                cmd_vel.publish(turn_cmd)
                rate.sleep()
        
if __name__ == '__main__':
    try:
        nav = Navigator()
        nav.navigation()
    except rospy.ROSInterruptException:
        pass