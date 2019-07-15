#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import UInt8
import time
import random
import rospy, rospkg
from os.path import join
from os.path import isdir, isfile
from os import listdir
import intera_interface
import argparse
from playsound import playsound
from tf import LookupException
import json, cv2, cv_bridge
from numpy import zeros, uint8
from collections import deque
from cv_bridge import CvBridge


class Sound_Expressions(object):

    RATE_SEC = 40

    def __init__(self):
        rospy.on_shutdown(self.screen_off)
        nbhappy = 4
        nbsad = 3
        self.cpt_fear=0
        self.cpt_hope=0
        self.sound_hope = {}
        self.sound_fear = {}
        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.list_sound_sad = []
        self.list_sound_happy = []
    


############ LOAD SAD SOUND
        for i in range (0,nbsad):
           num = i+1
           self.list_sound_sad.append(str(num))
        
        for sound in self.list_sound_sad:
            filename = sound + ".mp3"
            path = join(self.rospack.get_path("cs_sawyer"), "sound/sad", filename)
            self.sound_fear[sound] = path


############ LOAD HOPE SOUND
        for i in range (0,nbhappy):
           num = i+1
           self.list_sound_happy.append(str(num))
        
        for sound in self.list_sound_happy:
            filename = sound + ".mp3"
            path = join(self.rospack.get_path("cs_sawyer"), "sound/happy", filename)
            self.sound_hope[sound] = path

        self.robot_state = 0 # normal
        rospy.Subscriber("cs_sawyer/head_light", UInt8, self.callback_update_face)


    def callback_update_face(self,msg):
        self.robot_state = msg.data


    def update_sound(self):

        if self.robot_state == 0 or self.robot_state == 1:
            self.cpt_hope = 0
            self.cpt_fear = 0
       
        elif self.robot_state == 2 and self.cpt_hope == 0:  # hope
            self.cpt_fear = 0
            self.cpt_hope = self.cpt_hope + 1
            playsound(self.sound_hope[str(random.randint(1, 4))])
            

        elif self.robot_state == 3 and self.cpt_fear == 0:  # fear
            self.cpt_hope = 0
            self.cpt_fear = self.cpt_fear + 1
            playsound(self.sound_fear[str(random.randint(1, 3))])
            
            
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update_sound()                
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("cs_sawyer_sound_expressions")
    sound_expressions = Sound_Expressions()
    sound_expressions.run()