#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 12:29:14 2018
@author: EmaPajic
"""

import rospy
import numpy as np
from scan.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
from particle import Particle

class Localizator:
    def __init__(self):
        self.map_width = 298
        self.map_height = 803
        self.map = None
        self.particles = []
        self.sum_of_weights = 0.0
        self.particle_weights = []
        self.num_of_particles = 1000000
        self.scan_msg = None
        self.odom_msg = None
        self.last_odom = None
        self.meter_to_pixel = 34.035087719
        rospy.init_node('particle_filter', anonymous=True)
        self.rate = rospy.Rate(25) # 25hz
        self.odom_sub = rospy.Subscriber("odom", Odometry)
        self.laser_sub = rospy.Subscriber("scan", LaserScan)
        self.ts = message_filters.TimeSynchronizer([self.odom_sub, self.laser_sub], 10)
        self.ts.registerCallback(self.callback)
    
    def move_particles(self):
        if self.odom_msg is not None:
            if self.last_odom is not None:
                for i in range(self.num_of_particles):
                    new_width = self.particles[i].get_width() + \
                    self.odom_msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
                    self.particles[i].set_width(new_width)
                    new_height = self.particles[i].get_height() + \
                    self.odom_msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
                    self.particles[i].set_width(new_height)
                    new_angle = self.particles[i].get_angle() + \
                    self.odom_msg.pose.pose.orientation.z - self.last_odom.pose.pose.orientation.z
                    self.particles[i].set_width(new_angle)
            self.last_odom = self.odom_msg  
               
    def particle_likelihood(self, particle):
        similarity = 0
        cnt = 0
        if self.scan_msg is not None:
                for i in range(self.scan_msg.ranges.length):
                    r = self.scan_msg.ranges[i]
                    if r > self.scan_msg.range_min and r < self.scan_msg.range_max:
                        cnt += 1
                        pixel_length = int(r * self.meter_to_pixel)
                        pixel_angle = particle.get_angle() + self.scan_msg.angle_min + self.scan_msg.angle_increment * i
                        wall_x = particle.get_width() + int(pixel_length * np.cos(pixel_angle))
                        wall_y = particle.get_height() + int(pixel_length * np.sin(pixel_angle))
                        if self.map[wall_x][wall_y] == [255][255][255]:
                            similarity += 1
        return similarity/cnt
        
    def sample_particles(self):
        for i in range(self.num_of_particles):
            while True:
                particle_width = np.random.uniform(0, self.map_width)
                particle_height = np.random.uniform(0, self.map_height)
                if self.map[particle_width][particle_height] != [255,255,255]:
                    break
            particle_angle = np.random.uniform(0,360)
            particle = Particle(particle_width, particle_height, particle_angle)
            self.particles.append(particle)     
            
    def resample_particles(self):
        new_particles = []
        for i in range(self.num_of_particles):
            new_particle = Particle()
            temp = np.random.uniform(0, self.sum_of_weights)
            sum_until_temp = 0.0
            for j in range(self.num_of_particles):
                sum_until_temp += self.weights[j]
                if temp < sum_until_temp:
                    new_particle = self.particles[j]
                    break
            new_particles.append(new_particle)
        self.particles = new_particles
   
    def compute_weights(self):
        self.sum_of_weights = 0.0
        for particle in self.particles:
            particle_weight = self.particle_likelihood(particle)
            self.weights.append(particle_weight)
            self.sum_of_weights += particle_weight
        
    def callback(self, odom, scan):
        self.scan_msg = scan
        self.odom_msg = odom

    def particle_filter(self):
        self.sample_particles()
        while not rospy.is_shutdown():
            self.compute_weights()
            self.resample_particles()
            self.move_particles()
            self.rate.sleep()
    