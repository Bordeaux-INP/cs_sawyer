#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import intera_interface
from os.path import join
from cs_sawyer.msg import ButtonPressed, LightStatus

class InteractionController(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.limb = None
        self.rs = None
        self.waiting = {"fear": 0, "hope": 0}  # Votes waiting for execution
       
        with open(join(self.rospack.get_path("cs_sawyer"), "config/poses.json")) as f:
            self.poses = json.load(f)

        rospy.Subscriber("cs_sawyer/button", ButtonPressed, self._cb_button_pressed)
        self._light_pub_fear = rospy.Publisher("cs_sawyer/light/fear", LightStatus)
        self._light_pub_hope = rospy.Publisher("cs_sawyer/light/hope", LightStatus)
        rospy.loginfo("Sawyer Interaction Controller is ready!")

    def _cb_button_pressed(self, msg):
        if msg.type == ButtonPressed.FEAR:
            self.waiting["fear"] += 1
        elif msg.type == ButtonPressed.HOPE:
            self.waiting["hope"] += 1
        elif msg.type == ButtonPressed.RESET:
            raise NotImplementedError()

    def start_robot(self):
        self.rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self.rs.enable()
        self.limb = intera_interface.Limb('right')
        self.limb.move_to_neutral()

    def tuck_robot(self):
        self.limb.move_to_neutral()
        self.rs.disable()

    def run(self):
        self.start_robot()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.waiting["hope"] > 0:
                self.waiting["hope"] -= 1
                self.move("hope")
            elif self.waiting["fear"] > 0:
                self.waiting["fear"] -= 1
                self.move("fear")
            rate.sleep()

    def move(self, type):
        if type == "hope":
            self.limb.move_to_neutral()

if __name__=='__main__':
    rospy.init_node('cs_sawyer_interaction_controller')
    ic = InteractionController()
    ic.run()