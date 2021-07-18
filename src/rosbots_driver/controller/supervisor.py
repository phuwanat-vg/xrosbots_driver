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
import rospy
import tf
import tf2_ros
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Point,Quaternion,Twist,Vector3
from sensor_msgs.msg import Imu

from .robot import Robot
from .rc_teleop import RCTeleop
from .dynamics.differential_drive import DifferentialDrive

class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)
        
        #robto name for TF 
        
        self.robot_name = rospy.get_name()

        self.controllers = {"rc": RCTeleop()}
        self.current_state = "rc"
        self.current_controller = self.controllers[self.current_state]

        self.robot = Robot()
  
        
        self.dd = DifferentialDrive(self.robot.wheelbase,
                                    self.robot.wheel_radius)
        
        #time data
        self.last_time=rospy.Time.now()
        
        #Twist
        self.theta_dt = Float32()
        self.x_dt = Float32()
        self.y_dt = Float32()
        
        # Initialize TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Initialize previous wheel encoder ticks
        self.prev_wheel_ticks = None
        
        #publish odometry message
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=0)
            
        self.int_thres = 10000

    def execute(self):
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()

        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])
        
        self.update_odometry()
        
    
        
    def shutdown_cb(self):
        for ctrl in self.controllers.values():
            ctrl.shutdown()

        self.robot.shutdown()

    def update_odometry(self):  #this method for update odometry
        # Get wheel encoder ticks
        
        v_xy = 0
        v_th = 0
        
        ticks = self.robot.get_wheel_ticks()
        timestamp = self.robot.get_tick_timestamp()
        imu_angular_vel = self.robot.get_imu_angular()
        
        #Change of TIme
        dt = timestamp-self.last_time
        dt = dt.to_sec()*2

        # Have not seen a wheel encoder message yet so no need to do anything
        if ticks["r"] == None or ticks["l"] == None:
            return

        # Robot may not start with encoder count at zero
        if self.prev_wheel_ticks == None:
            self.prev_wheel_ticks = {"r": ticks["r"], "l": ticks["l"]}

        # Get current pose from robot
        prev_pose = self.robot.get_pose2D()

        # Compute odometry - kinematics in meters
        R = self.robot.wheel_radius;                           #wheel radius
        L = self.robot.wheelbase;                              #wheel base
        ticks_per_rev = self.robot.encoder_ticks_per_rev
        meters_per_tick = (2.0 * math.pi * R) / ticks_per_rev  #calculate how many distance per tick
        
        # How far did each wheel move
        wheel_dir = self.robot.get_wheel_dir()
        
        #----------------Uncomment for DC Gear Motor with ENcoder------------
      
        meters_right = meters_per_tick * (ticks["r"] - self.prev_wheel_ticks["r"]) #*wheel_dir["r"]
        meters_left =  meters_per_tick * (ticks["l"] - self.prev_wheel_ticks["l"]) #*wheel_dir["l"]
        meters_center = (meters_right + meters_left) * 0.5  #Average of distance
          
        #RPM calculation
        rpm_r = (((ticks["r"]-self.prev_wheel_ticks["r"])/ticks_per_rev)/dt)*60
        rpm_l = (((ticks["l"]-self.prev_wheel_ticks["l"])/ticks_per_rev)/dt)*60
        rps_r = (rpm_r*2*math.pi)/60
        rps_l = (rpm_l*2*math.pi)/60
        
        #uncomment to show RPM
        #rospy.loginfo("Right wheel speed : "+str(round(rpm_r,3))+" RPM "+ str(round(rps_r,3))+ " rad/s")
        #rospy.loginfo("Left wheel speed : "+str(round(rpm_l,3))+" RPM "+ str(round(rps_l,3))+ " rad/s")
        
        # Compute new pose
        self.x_dt = meters_center * math.cos(prev_pose.theta);  #projection of meter to x and y axis
        self.y_dt = meters_center * math.sin(prev_pose.theta);
        
        self.theta_dt = (meters_left-meters_right) / L;
        
        v_xy = meters_center/dt
        v_th = self.theta_dt/dt

        
        new_pose = Pose2D(0.0, 0.0, 0.0)
        new_pose.x = prev_pose.x + self.x_dt
        new_pose.y = prev_pose.y + self.y_dt
        theta_tmp = prev_pose.theta + self.theta_dt
        new_pose.theta = math.atan2( math.sin(theta_tmp), math.cos(theta_tmp) )
        #new_pose.theta = theta_tmp

        # Update robot with new pose
        self.robot.set_pose2D(new_pose)

        # Update the tick count
        self.prev_wheel_ticks["r"] = ticks["r"]
        self.prev_wheel_ticks["l"] = ticks["l"]

        # Broadcast pose as ROS tf
        ppp = self.robot.get_pose2D()  #receive value from get_Pose2D method in Robot class
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = ppp.x
        t.transform.translation.y = ppp.y
        t.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, ppp.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        #position odometry

        odom.pose.pose = Pose(Point(ppp.x,ppp.y,0),Quaternion(*q))
        #velocity odometry
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(v_xy,0.0,0.0),Vector3(0,0,v_th))       
        
        self.pub_odom.publish(odom)
        
        #Update time
        self.last_time = timestamp
