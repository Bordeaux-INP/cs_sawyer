#!/usr/bin/env python
# -*- coding: utf-8 -*-


import intera_interface
from intera_interface import CHECK_VERSION
from intera_interface import Lights
from cs_sawyer.msg import ButtonPressed, LightStatus
from std_msgs.msg import UInt8
import rospy, rospkg
from os.path import join, isdir, isfile
from os import listdir
from tf import LookupException
import json, cv2, cv_bridge
from numpy import zeros, uint8
from sensor_msgs.msg import Image
from collections import deque
from cv_bridge import CvBridge
import argparse	
import time
import random



class Head_Move(object):


    def __init__(self):

        self._done = False
        self._head = intera_interface.Head()
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self.robot_state = 0 # normal
        rospy.Subscriber("cs_sawyer/head_light", UInt8, self.callback_update_move)
 
    def wobble(self):
        """
        Performs the wobbling
        """
        command_rate = rospy.Rate(1)
        control_rate = rospy.Rate(100)
        while self.robot_state== 0:
            angle = random.uniform(-2.8, 0.3)
            while (not rospy.is_shutdown() and
                   not (abs(self._head.pan() - angle) <=
                       intera_interface.HEAD_PAN_ANGLE_TOLERANCE)):
                self._head.set_pan(angle, speed=0.3, timeout=0)
                control_rate.sleep()
            command_rate.sleep()
        
    def set_neutral(self):
        self._head.set_pan(-1.57)
    
    def set_look_board(self):
        self._head.set_pan(-3)

    def callback_update_move(self,msg):
        self.robot_state = msg.data

    def update_move(self):
        if self.robot_state == 0:   # normal
            self.wobble()
        elif self.robot_state == 1 :   # error
            self.set_neutral()
        elif self.robot_state == 2 or self.robot_state == 3:  # writing
            self.set_look_board()
       

            
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_move()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("cs_sawyer_head_move")
    head_move = Head_Move()
    head_move.run()