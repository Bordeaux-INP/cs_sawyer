#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import intera_interface
from os.path import join
from std_msgs.msg import Int32
from cs_sawyer.msg import ButtonPressed, LightStatus

class InteractionController(object):
    # Couples of [hope light status, fear light status]
    ANIMATION_MOTION_RUNNING_HOPE = [LightStatus.SLOW_BLINK, LightStatus.OFF]
    ANIMATION_MOTION_RUNNING_FEAR = [LightStatus.OFF, LightStatus.SLOW_BLINK]
    ANIMATION_IDLE = [LightStatus.ON, LightStatus.ON]
    ANIMATION_ERROR = [LightStatus.FAST_BLINK, LightStatus.FAST_BLINK]
    ANIMATION_RESETTING = [LightStatus.SLOW_BLINK, LightStatus.SLOW_BLINK]
    ANIMATION_OFF = [LightStatus.OFF, LightStatus.OFF]

    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.limb = None
        self.rs = None
        self.waiting = {"fear": 0, "hope": 0}  # Votes waiting for execution
       
        with open(join(self.rospack.get_path("cs_sawyer"), "config/poses.json")) as f:
            self.poses = json.load(f)

        rospy.Subscriber("cs_sawyer/button", ButtonPressed, self._cb_button_pressed)
        self.light_pub_fear = rospy.Publisher("cs_sawyer/light/fear", LightStatus, queue_size=1)
        self.light_pub_hope = rospy.Publisher("cs_sawyer/light/hope", LightStatus, queue_size=1)
        rospy.loginfo("Sawyer Interaction Controller is ready!")

    def _cb_button_pressed(self, msg):
        if msg.type.data == ButtonPressed.FEAR:
            self.waiting["fear"] += 1
        elif msg.type.data == ButtonPressed.HOPE:
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
        rospy.sleep(0.5)
        self.update_lights(self.ANIMATION_OFF)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.waiting["hope"] > 0:
                self.waiting["hope"] -= 1
                self.move("hope")
            elif self.waiting["fear"] > 0:
                self.waiting["fear"] -= 1
                self.move("fear")
            rate.sleep()

    def update_lights(self, animations):
        hope = animations[0]
        fear = animations[1]
        self.light_pub_hope.publish(LightStatus(type=Int32(hope)))
        self.light_pub_fear.publish(LightStatus(type=Int32(fear)))

    def move(self, type):
        if type == "hope":
            self.update_lights(self.ANIMATION_MOTION_RUNNING_HOPE)
            self.limb.move_to_neutral()
        elif type == "fear":
            self.update_lights(self.ANIMATION_MOTION_RUNNING_FEAR)
            self.limb.move_to_neutral()




if __name__=='__main__':
    rospy.init_node('cs_sawyer_interaction_controller')
    ic = InteractionController()
    ic.run()