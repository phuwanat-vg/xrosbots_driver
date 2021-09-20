#!/usr/bin/env python

#
# This file is part of ROSbots ROS Drivers.
#
# Copyright
#
#     Copyright (C) 2017 Jack Pien <jack@rosbots.com>
#
# License
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU Lesser General Public License as published
#     by the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU Lesser General Public License for more details at
#     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
#
# Documentation
#
#     http://www.rosbots.com
#

import rospy

class DifferentialDrive:
    def __init__(self, wheelbase, wheel_radius):
      
        self.wheelbase = wheelbase 
        self.wheel_radius = wheel_radius

    def uni_to_diff(self, v, w):
        '''
        Return mm per sec wheel velocities
        '''
        w_lower_limit = 4.2
        w_upper_limit = 4.6
        scaling_factor = 5.0
        if v == 0.0:
           w = scaling_factor*w
           if w<0:
              w = min(max(w,-w_upper_limit),-w_lower_limit)
           elif w>0:
              w = max(min(w,w_upper_limit),w_lower_limit)

        # In meters per radian
        L = self.wheelbase
        R = self.wheel_radius
        #w = min(4.3,w)
        # w is angular velocity counter clockwise - radians/sec
        # v - m/s
        # L - wheelbase - meter/radian
        # R - wheel radius - meter/radian
        vr = ((2.0 * v) + (w * L)) / (2.0 * R)
        vl = ((2.0 * v) + (-1.0 * w * L)) / (2.0 * R)

        # In m per sec
        return {"vl": vl, "vr": vr}
