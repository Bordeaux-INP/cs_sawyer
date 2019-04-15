#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import intera_interface
from os.path import join
from std_msgs.msg import Int32
from cs_sawyer.msg import ButtonPressed, LightStatus
from copy import deepcopy

class InteractionController(object):
    # Couples of [hope light status, fear light status]
    ANIMATION_MOTION_RUNNING_HOPE = [LightStatus.FAST_BLINK, LightStatus.OFF]
    ANIMATION_MOTION_RUNNING_FEAR = [LightStatus.OFF, LightStatus.FAST_BLINK]
    ANIMATION_IDLE = [LightStatus.ON, LightStatus.ON]
    ANIMATION_ERROR = [LightStatus.SLOW_BLINK, LightStatus.SLOW_BLINK]
    ANIMATION_OFF = [LightStatus.OFF, LightStatus.OFF]

    def __init__(self, speed=0.15):
        self.rospack = rospkg.RosPack()
        self.speed = speed
        self.limb = None
        self.rs = None
        self.error = False
        self.waiting = {"fear": 0, "hope": 0}  # Votes waiting for execution
       
        with open(join(self.rospack.get_path("cs_sawyer"), "config/motions.json")) as f:
            self.motions = json.load(f)
        with open(join(self.rospack.get_path("cs_sawyer"), "config/poses.json")) as f:
            self.poses = json.load(f)

        rospy.Subscriber("cs_sawyer/button", ButtonPressed, self._cb_button_pressed)
        self.light_pub_fear = rospy.Publisher("cs_sawyer/light/fear", LightStatus, queue_size=1)
        self.light_pub_hope = rospy.Publisher("cs_sawyer/light/hope", LightStatus, queue_size=1)
        rospy.loginfo("Sawyer Interaction Controller is ready!")

    def _cb_button_pressed(self, msg):
        if msg.type.data == ButtonPressed.FEAR:
            rospy.loginfo("Queuing new vote: fear")
            self.waiting["fear"] += 1
        elif msg.type.data == ButtonPressed.HOPE:
            rospy.loginfo("Queuing new vote: hope")
            self.waiting["hope"] += 1
        elif msg.type.data == ButtonPressed.RESET:
            self.reset_errors()

    def start_robot(self):
        self.rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self.rs.enable()
        self.limb = intera_interface.Limb('right')
        #self.limb.move_to_neutral(speed=self.speed)

    def tuck_robot(self):
        self.limb.move_to_neutral(speed=self.speed)
        self.rs.disable()

    def move_to_joint_positions(self, positions, speed=None, threshold=0.008726646):
        speed = speed if speed is not None else self.speed
        self.limb.set_joint_position_speed(speed)
        self.limb.move_to_joint_positions(positions, threshold=threshold)

    def move_to_pause_position(self):
        self.limb.set_joint_position_speed(self.speed)
        self.move_to_joint_positions(self.poses["pause"])

    def check_for_errors(self):
        if not self.error and (rospy.get_param("cs_sawyer/votes/hope/executed", 0) == len(self.motions["hope"]) or \
           rospy.get_param("cs_sawyer/votes/fear/executed", 0) == len(self.motions["fear"])):
           rospy.logerr("Board full, please reset...")
           self.error = True
           self.update_lights(self.ANIMATION_ERROR)

    def reset_errors(self):
        rospy.logerr("Resetting errors")
        rospy.set_param("cs_sawyer/votes/hope/executed", 0)
        rospy.set_param("cs_sawyer/votes/fear/executed", 0)
        self.waiting['fear'] = 0
        self.waiting['hope'] = 0
        self.error = False

    def run(self):
        self.start_robot()
        rospy.sleep(0.5)
        self.move_to_pause_position()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_for_errors()
            if not self.error:
                self.update_lights(self.ANIMATION_IDLE)
                if self.waiting["hope"] > 0:
                    self.waiting["hope"] -= 1
                    self.move("hope")
                elif self.waiting["fear"] > 0:
                    self.waiting["fear"] -= 1
                    self.move("fear")
            rate.sleep()
        self.update_lights(self.ANIMATION_OFF)

    def update_lights(self, animations):
        hope = animations[0]
        fear = animations[1]
        self.light_pub_hope.publish(LightStatus(type=Int32(hope)))
        self.light_pub_fear.publish(LightStatus(type=Int32(fear)))

    def tuck_finger(self, positions):
        tucked_positions = deepcopy(positions)
        tucked_positions["right_j5"] += -0.5
        #tucked_positions.update({"right_j5": -2.65})
        return tucked_positions

    def move(self, type):
        vote_id = rospy.get_param("cs_sawyer/votes/{}/executed".format(type), 0)
        rospy.logwarn("Executing {} vote num {}".format(type, vote_id))
        self.update_lights(self.ANIMATION_MOTION_RUNNING_HOPE if type == "hope" else self.ANIMATION_MOTION_RUNNING_FEAR)
        # TODO: There are better ways to execute cartesian motions

        pose_init = dict(zip(self.motions["joints"], self.motions[type][vote_id][0]))
        tucked_pose_init = self.tuck_finger(pose_init)
        self.move_to_joint_positions(tucked_pose_init)
        self.move_to_joint_positions(pose_init, speed=0.15)

        for point in self.motions[type][vote_id][1:]:
            self.move_to_joint_positions(dict(zip(self.motions["joints"], point)), threshold=0.01)    
            
        pose_end = dict(zip(self.motions["joints"], self.motions[type][vote_id][-1]))
        tucked_pose_end = self.tuck_finger(pose_end)
        self.move_to_joint_positions(tucked_pose_end, speed=0.4)
        self.move_to_pause_position()
        rospy.set_param("cs_sawyer/votes/{}/executed".format(type), vote_id + 1)
        self.update_lights(self.ANIMATION_IDLE)





if __name__=='__main__':
    rospy.init_node('cs_sawyer_interaction_controller')
    ic = InteractionController()
    ic.run()