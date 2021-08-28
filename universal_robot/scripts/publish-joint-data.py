#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 24 21:10:58 2021

@author: abc
"""

import rospy
from std_msgs.msg import Float64
from math import pi as pi
import numpy as np

#inverse Kinematics
from UR5_inverse_kinematics import invKin
desired_pos = np.matrix([[1,0,0,3]
                         ,[0,1,0,2]
                         ,[0,0,1,8]
                         ,[0,0,0,1]])
theta =invKin(desired_pos)
theta_list=theta[:,0]
#joint angle
joint1_position=theta_list[0]*(180/pi)
joint2_position=theta_list[1]*(180/pi)
joint3_position=theta_list[2]*(180/pi)
joint4_position=theta_list[3]*(180/pi)
joint5_position=theta_list[4]*(180/pi)
joint6_position=theta_list[5]*(180/pi)

def Joint_state_publisher():
    pub1 = rospy.Publisher('/simple_robot/joint1_position_controller/command',Float64,queue_size=10)
    pub2 = rospy.Publisher('/simple_robot/joint2_position_controller/command',Float64,queue_size=10)
    pub3 = rospy.Publisher('/simple_robot/joint3_position_controller/command',Float64,queue_size=10)
    pub4 = rospy.Publisher('/simple_robot/joint4_position_controller/command',Float64,queue_size=10)
    pub5 = rospy.Publisher('/simple_robot/joint5_position_controller/command',Float64,queue_size=10)
    pub6 = rospy.Publisher('/simple_robot/joint6_position_controller/command',Float64,queue_size=10)
    rospy.init_node('Joint_state_publisher',anonymous=True)
    
    while not rospy.is_shutdown():
        pub1.publish(joint1_position*(pi/180))
        pub2.publish(joint2_position*(pi/180))
        pub3.publish(joint3_position*(pi/180))
        pub4.publish(joint4_position*(pi/180))
        pub5.publish(joint5_position*(pi/180))
        pub6.publish(joint6_position*(pi/180))
        rospy.loginfo(joint1_position)
        rospy.loginfo(joint2_position)
        rospy.loginfo(joint3_position)
        rospy.loginfo(joint4_position)
        rospy.loginfo(joint5_position)
        rospy.loginfo(joint6_position)

if __name__=='__main__':
    try:
        Joint_state_publisher()
    except rospy.ROSInterruptException:
        pass