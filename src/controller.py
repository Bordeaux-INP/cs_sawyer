#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import intera_interface
from os.path import join
from std_msgs.msg import Int32
from sawyer.sticks import Sticks
from cs_sawyer.msg import ButtonPressed, LightStatus
from copy import deepcopy

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_msgs.msg import (
    Trajectory,
    TrajectoryOptions,
)

class InteractionController(object):
    # Couples of [hope light status, fear light status]
    ANIMATION_MOTION_RUNNING_HOPE = [LightStatus.FAST_BLINK, LightStatus.OFF]
    ANIMATION_MOTION_RUNNING_FEAR = [LightStatus.OFF, LightStatus.FAST_BLINK]
    ANIMATION_IDLE = [LightStatus.ON, LightStatus.ON]
    ANIMATION_CALIBRATING = [LightStatus.FAST_BLINK, LightStatus.FAST_BLINK]
    ANIMATION_ERROR = [LightStatus.SLOW_BLINK, LightStatus.SLOW_BLINK]
    ANIMATION_OFF = [LightStatus.OFF, LightStatus.OFF]
    Z_PEN_OFFSET = 0.19

    def __init__(self, speed=0.15, acceleration=0.001):
        self.rospack = rospkg.RosPack()
        self.speed = speed
        self.acceleration = acceleration
        self.limb = None
        self.rs = None
        self.calibrate = False
        self.calibrate_requested = False
        self.calibrate_state_machine_step = 0
        self.error = False
        self.reset_error = False
        self.waiting = {"fear": 0, "hope": 0}  # Votes waiting for execution
        self._sticks = Sticks()
        self.motions = self._sticks.get_cartesian_motions()
        self.head = intera_interface.head.Head()
        self.head.set_pan(-1.57)

        with open(join(self.rospack.get_path("cs_sawyer"), "config/poses.json")) as f:
            self.poses = json.load(f)

        rospy.Subscriber("cs_sawyer/button", ButtonPressed, self._cb_button_pressed)
        self.light_pub_fear = rospy.Publisher("cs_sawyer/light/fear", LightStatus, queue_size=1)
        self.light_pub_hope = rospy.Publisher("cs_sawyer/light/hope", LightStatus, queue_size=1)
        rospy.loginfo("Sawyer Interaction Controller is ready!")

    def _cb_button_pressed(self, msg):
        if msg.type.data == ButtonPressed.FEAR and not self.error:
            rospy.loginfo("Queuing new vote: fear")
            self.waiting["fear"] += 1
        elif msg.type.data == ButtonPressed.HOPE and not self.error:
            rospy.loginfo("Queuing new vote: hope")
            self.waiting["hope"] += 1
        elif msg.type.data == ButtonPressed.RESET:
            self.reset_error = True
        elif msg.type.data == ButtonPressed.CALIBRATE:
            self.calibrate = True
            self.calibrate_requested = True

    def start_robot(self):
        self.rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self.rs.enable()
        self.limb = intera_interface.Limb('right')
        self._sticks.set_limb(self.limb)
        #self.limb.move_to_neutral(speed=self.speed)

    def tuck_robot(self):
        self.limb.move_to_neutral(speed=self.speed)
        self.rs.disable()

    def move_to_joint_positions(self, positions, speed=None, acceleration=None):
        speed = speed if speed is not None else self.speed
        acceleration = acceleration if acceleration is not None else self.acceleration
        traj = MotionTrajectory(limb = self.limb)
        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed,
                                         max_joint_accel=acceleration)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self.limb)
        if isinstance(positions, dict):
            joint_angles = positions.values()
            waypoint.set_joint_angles(joint_angles = joint_angles)
            traj.set_joint_names(positions.keys())
            traj.append_waypoint(waypoint.to_msg())
        elif isinstance(positions, (list, tuple)):
            if len(positions) > 0:
                joint_angles = positions[0].values()
                joint_names = positions[0].keys()
                for point in positions:
                    waypoint.set_joint_angles(joint_angles = point.values())
                    traj.append_waypoint(waypoint.to_msg())
                traj.set_joint_names(joint_names)
                #t_opt = TrajectoryOptions(end_time=rospy.Time(1))
                #traj.set_trajectory_options(t_opt)
        else:
            rospy.logerr("Incorrect inputs to move_to_joint_positions")
            return

        result = traj.send_trajectory(timeout=30)
        if result is None:
            rospy.logerr("Trajectory failed to send")
        elif not result.result:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

    def move_to_pause_position(self):
        self.limb.set_joint_position_speed(self.speed)
        self.move_to_joint_positions(self.poses["pause"])

    def check_for_errors(self):
        if (rospy.get_param("cs_sawyer/votes/hope/executed", 0) >= len(self.motions["hope"]) or \
            rospy.get_param("cs_sawyer/votes/fear/executed", 0) >= len(self.motions["fear"])):
            if not self.error:
                rospy.logerr("Board full, please reset...")
            self.error = True
            self.update_lights(self.ANIMATION_ERROR)
        else:
            self.error = False

        if self.reset_error:
            rospy.logerr("Resetting errors")
            self.update_lights(self.ANIMATION_OFF)
            rospy.set_param("cs_sawyer/votes/hope/executed", 0)
            rospy.set_param("cs_sawyer/votes/fear/executed", 0)
            rospy.sleep(3)
            # We have just recovered from error, wait a bit for the user...
            self.waiting['fear'] = 0
            self.waiting['hope'] = 0
            self.reset_error = False       
            rospy.logwarn("Reset is over, resuming operation...")
    
    def check_calibration(self):
        if self.calibrate and self.calibrate_requested:
            if self.calibrate_state_machine_step == 0:
                # Waiting for a second press
                self.update_lights(self.ANIMATION_CALIBRATING)
                self.calibrate_state_machine_step = 1
                rospy.logwarn("Prepare init pose and press calibrate again")
                self.calibrate_requested = False
            elif self.calibrate_state_machine_step == 1:
                self.update_lights(self.ANIMATION_OFF)
                success = self._sticks.calibrate(self.Z_PEN_OFFSET)
                if success:
                    self.calibrate_state_machine_step = 2
                else:
                    self.calibrate_state_machine_step = 0
            elif self.calibrate_state_machine_step == 2:
                self.update_lights(self.ANIMATION_IDLE)
                self._sticks.overwrite_cartesian_motions()
                self.motions = self._sticks.get_cartesian_motions()
                rospy.logwarn("Calibration is over, resuming operation...")
                self.move_to_pause_position()
                self.waiting['fear'] = 0
                self.waiting['hope'] = 0
                self.calibrate = False
                self.calibrate_requested = False
                self.calibrate_state_machine_step = 0


    def run(self):
        self.start_robot()
        rospy.sleep(0.5)
        self.move_to_pause_position()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_calibration()
            if not self.calibrate:
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

    def move(self, type):
        vote_id = rospy.get_param("cs_sawyer/votes/{}/executed".format(type), 0)
        rospy.logwarn("Executing {} vote num {}".format(type, vote_id))
        self.update_lights(self.ANIMATION_MOTION_RUNNING_HOPE if type == "hope" else self.ANIMATION_MOTION_RUNNING_FEAR)
        # TODO: There are better ways to execute cartesian motions

        self.move_to_joint_positions([dict(zip(self.motions["joints"], point)) for point in self.motions[type][vote_id]])    
            
        self.move_to_pause_position()
        rospy.set_param("cs_sawyer/votes/{}/executed".format(type), vote_id + 1)
        self.update_lights(self.ANIMATION_IDLE)


if __name__=='__main__':
    rospy.init_node('cs_sawyer_interaction_controller')
    ic = InteractionController()
    ic.run()