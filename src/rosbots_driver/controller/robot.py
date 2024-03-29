#!/usr/bin/env python3

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
import math
from _thread import allocate_lock
import rospy
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu

class Robot:
    def __init__(self):
      
        self.wheelbase = rospy.get_param("wheelbase", default=0.177)
        self.wheel_radius = rospy.get_param("wheel_radius", default=0.04255)  #Motor with encoder 0.04255
                                                                              #TT DC gear motor white wheel 0.02
        
        #Encoder : define ticks per revolution
        self.encoder_ticks_per_rev = \
            rospy.get_param("encoder_ticks_per_rev", default = 210)  #140 for 225 RPM motor
                                                                     #Motor with encoder : 210 counts/rev
                                                                     #Motor with encoder1:120 : 840 counts/rev
        self.meters_per_tick = \
            ((math.pi * 2.0 * self.wheel_radius) / (float)(self.encoder_ticks_per_rev))   # 2piR/number of ticks

        # Wheel min and max no-load velocities in radians per sec

        self.wheel_speed_min = rospy.get_param("wheel_speed/min", default=3.5)   #8.5 rad/s for 225RPM
        self.wheel_speed_mid = rospy.get_param("wheel_speed/mid", default=8.5) #12.7 rad/s for 225 RPM
        self.wheel_speed_max = rospy.get_param("wheel_speed/max", default=12)  #17 rad/s for 225 RPM

        self.wheel_speed_min_power = rospy.get_param("wheel_speed/min_power", default=0.1)
        self.wheel_speed_mid_power = rospy.get_param("wheel_speed/mid_power", default=0.55)
        self.wheel_speed_max_power = rospy.get_param("wheel_speed/max_power", default=1.0)

        # Publish out wheel power
        self.cur_wheel_power_right = Float32()
        self.cur_wheel_power_left = Float32()
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        
        self._cur_imu_angular_z = Imu()
        self._cur_imu_angular_ts = 0.0      
        
        self.pub_power_right = rospy.Publisher('/wheel_power_right', Float32, queue_size=10)
        self.pub_power_left = rospy.Publisher('/wheel_power_left', Float32, queue_size=10)
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        
  
    
        # Subscribe to wheel encoder ticks
        self.wheel_ticks_right_lock = allocate_lock()    #Create lock object
        self.wheel_ticks_left_lock = allocate_lock()
        self.sub_wheel_ticks_right = rospy.Subscriber("/tick_wheel_right", Int32, self.wheel_ticks_cb,(True))
        self.sub_wheel_ticks_left = rospy.Subscriber("/tick_wheel_left", Int32, self.wheel_ticks_cb, (False))
        
        # Subscribe to imu data
        self.imu_angular_z = rospy.Subscriber("/imu", Imu, self.imu_cb)
        
        #defne new variable and set initial condition
        self._cur_wheel_ticks_right = None 
        self._cur_wheel_ticks_left = None
        self._cur_wheel_ticks_ts = None
        self._prev_wheel_ticks = {"r" : None, "l" : None, "ts": None}
        self._wheel_velocity = {"r": 0.0, "l": 0.0}

        #Initial Robot pose
        self.pose2D = Pose2D(0.0,0.0,0.0)
        
      
        
    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Robot shutdown")
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        
    def get_tick_timestamp(self):
        timestamp = rospy.Time.now()
        return timestamp
        
    def get_wheel_ticks(self):
        ticks = {}
        self.wheel_ticks_right_lock.acquire()     #.acquire use to chabge lock object state from unlocked to locked
        self.wheel_ticks_left_lock.acquire()
        ticks["r"] = self._cur_wheel_ticks_right  #keep ticks value and time stanp to tick{} with dictionary
        ticks["l"] = self._cur_wheel_ticks_left
        ticks["ts"] = self._cur_wheel_ticks_ts
        self.wheel_ticks_right_lock.release()     #unlock locked object
        self.wheel_ticks_left_lock.release()
        self.get_tick_timestamp()
        return ticks
    
    def get_pose2D(self):  #create method for return of pose2D
        return self.pose2D

    def set_pose2D(self, pose2D):
        self.pose2D.x = pose2D.x
        self.pose2D.y = pose2D.y
        self.pose2D.theta = pose2D.theta
    
    #Check wheel direction 
    def get_wheel_dir(self):
        wheel_dir = {"r": 1.0, "l": 1.0}
        if self.cur_wheel_power_left.data < 0.0:
            wheel_dir["l"] = -1.0
        elif self.cur_wheel_power_left.data > 0.0:
            wheel_dir["l"] = 1.0
        else:
            wheel_dir["l"] = 0.0

        if self.cur_wheel_power_right.data < 0.0:
            wheel_dir["r"] = -1.0
        elif self.cur_wheel_power_right.data > 0.0:
            wheel_dir["l"] = 1.0
        else:
            wheel_dir["r"] = 0.0

        return wheel_dir
    
    def wheel_ticks_cb(self, ticks, is_right_wheel):
        if is_right_wheel:                                #right wheel
            self.wheel_ticks_right_lock.acquire()         #lock
            self._cur_wheel_ticks_ts = rospy.Time.now()   #keep time stamp
            self._cur_wheel_ticks_right = ticks.data      #keep number pf ticks
            self.wheel_ticks_right_lock.release()         #unlock
        else:                                             #Left wheel
            self.wheel_ticks_left_lock.acquire()
            self._cur_wheel_ticks_ts = rospy.Time.now()
            self._cur_wheel_ticks_left = ticks.data
            self.wheel_ticks_left_lock.release()
      
    def imu_cb(self, imu_w):
        
        self._cur_imu_angular_z = imu_w.angular_velocity.z
        self._cur_imu_angular_ts = rospy.Time.now()
    
    
    def get_imu_angular(self):
        imu_angular_vel = {}
        imu_angular_vel["w_z"] = self._cur_imu_angular_z
        imu_angular_vel["ts"] = self._cur_imu_angular_ts

        return imu_angular_vel
    
    
    def velocity_to_power(self, v):
        av = abs(v)   #absolute of velocity

        # If velocity is below minimum velocity turnable by PWM, then
        # just set to zero since the wheels won't spin anyway
        if av < self.wheel_speed_min:   #if velocity less than min speed of motor, set the speed to zero
            return 0.0

        a = b = a_pow = b_pow = None   #define variable
        nnn = None
        if av >= self.wheel_speed_min and av < self.wheel_speed_mid:    #Case1 : Desired velocity is between minimum and middle velocity
            a = self.wheel_speed_min                                    #wheel speed
            a_pow = self.wheel_speed_min_power                          #power of speed
            b = self.wheel_speed_mid                                    #wheel speed
            b_pow = self.wheel_speed_mid_power                          #power of speed
        elif av >= self.wheel_speed_mid and av <= self.wheel_speed_max: #Case2 : Desired velocity is between middle and maximum velocity

            a = self.wheel_speed_mid                      #wheel speed
            a_pow = self.wheel_speed_mid_power            #power of speed
            b = self.wheel_speed_max                      #wheel speed
            b_pow = self.wheel_speed_max_power            #power of speed
        
        # Linearly interpolate a and b
        nnn = ((av - a)/(b - a))                          # velocity fraction = (desired vel - lower vel)/(upper vel - lower_vel)
        wheel_power = ((nnn * (b_pow - a_pow)) + a_pow)   # wheel_power = [velocity_fraction*(upper power - lower power)]+lower power

     
        assert(wheel_power <= 1.0)
        assert(wheel_power >= 0.0)

        # Negate if necessary
        if v < 0:
            wheel_power *= -1.0

        return wheel_power

    def set_wheel_speed(self, vr, vl):
        # Clamp the wheel speeds to actuator limits
        factor_vr = 1.0                                    #Tune it when the wheel speed not equal
        factor_vl = 1.0
        vr = max(min(vr*factor_vr, self.wheel_speed_max), self.wheel_speed_max * -1.0)
        vl = max(min(vl*factor_vl, self.wheel_speed_max), self.wheel_speed_max * -1.0)
        
        cur_ticks = self.get_wheel_ticks()
        
        # Special stop case
        if vr == 0.0 and vl == 0.0:                        #when the robot stop
            for ddd in ["l", "r"]:                         #create ddd for loop   
                self._wheel_velocity[ddd] = 0.0            #set wheel velocity left and right to zero
                self.cur_wheel_power_right.data = 0.0      #set wheel power right to zero
            self.cur_wheel_power_left.data = 0.0           #set wheel power left to zero
            self._prev_wheel_ticks["r"] = cur_ticks["r"]   #suvstitute previous wheel ticks with current ticks
            self._prev_wheel_ticks["l"] = cur_ticks["l"]
            self._prev_wheel_ticks["ts"] = cur_ticks["ts"]
        else:
            # What direction do we want to go in?
            v_dir = {"l": 1.0, "r": 1.0} 
            for ddd in ["l", "r"]:
                if (vl < 0.0 and ddd == "l") or (vr < 0.0 and ddd == "r"):
                    v_dir[ddd] = -1.0

            # Get actual direction motors are turning in
            motor_dir = self.get_wheel_dir()
            
            # Compute velocity of wheels
            if self._prev_wheel_ticks["r"] != None and self._prev_wheel_ticks["l"] != None and self._prev_wheel_ticks["ts"] != None:
                tick_dur = cur_ticks["ts"] - self._prev_wheel_ticks["ts"] #time duration between ticks
                inv_sec = 0.0
                if tick_dur.nsecs != 0:
                    inv_sec = 1000000000.0 / (float)(tick_dur.nsecs)
                for ddd in ["l", "r"]:
                    self._wheel_velocity[ddd] = (float)(cur_ticks[ddd] - self._prev_wheel_ticks[ddd]) * inv_sec * self.meters_per_tick * motor_dir[ddd]
            
            self._prev_wheel_ticks["r"] = cur_ticks["r"]
            self._prev_wheel_ticks["l"] = cur_ticks["l"]
            self._prev_wheel_ticks["ts"] = cur_ticks["ts"]
            
        # Convert to power norms
        self.cur_wheel_power_right.data = vr*9.5493
        self.cur_wheel_power_left.data = vl*9.5493

        # Publish out
        
        self.cur_wheel_power_left = self.cur_wheel_power_left
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        
        

    
        

        
