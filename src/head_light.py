#!/usr/bin/env python
# -*- coding: utf-8 -*-

from intera_interface import Lights
from std_msgs.msg import UInt8
import rospy, rospkg
from numpy import uint8
import argparse
import time


class Head_Light(object):

    BLINK_DURATION = 0.7     # Second of blinking on and off

    def __init__(self):
        self.l = Lights()
        #self.robot_light_fear = ['head_green_light', 'head_red_light']
        self.robot_light_fear = ['head_blue_light']
        self.robot_light_hope = ['head_green_light']
        self.robot_light_error = ['head_red_light']
        self.robot_all_leds = ['head_red_light', 'head_blue_light','head_green_light']
        self.state_error_light = False
        self.robot_state = 0 # normal
        rospy.Subscriber("cs_sawyer/head_light", UInt8, self.callback_update_light)
 
    def on_off(self,leds,on=True):
        for light in self.robot_all_leds:
            if light not in leds:
                self.l.set_light_state(light, False)
            else : 
                self.l.set_light_state(light, on)
                

    def blink(self, leds):
        self.state_error_light = not self.state_error_light
        self.on_off(leds,self.state_error_light)   
        time.sleep(self.BLINK_DURATION)

    def callback_update_light(self,msg):
        self.robot_state = msg.data

    def update_leds(self):
        if self.robot_state == 0:   # normal
            self.on_off(self.robot_all_leds)
        elif self.robot_state == 1:   # error
            self.blink(self.robot_light_error)
        elif self.robot_state == 2 :  # hope
            self.on_off(self.robot_light_hope)
        elif self.robot_state == 3 : #fear
            self.on_off(self.robot_light_fear)

            
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_leds()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("cs_sawyer_head_light")
    head_light = Head_Light()
    head_light.run()