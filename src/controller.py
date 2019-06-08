#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tf
import rospy
import rospkg
import json
import intera_interface
from os.path import join, isdir
from os import makedirs
from std_msgs.msg import Int32
from sawyer.sticks import Sticks
from cs_sawyer.msg import ButtonPressed, LightStatus
from copy import deepcopy
from time import strftime, time
from sawyer.transformations import list_to_pose_stamped

from intera_core_msgs.msg import EndpointState

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

    # Collision detection and safety
    NUM_COLLISION_SAMPLES = 50
    WRENCH_LIMIT = 700.0

    def __init__(self, speed=0.2, acceleration=0.1):
        rospy.set_param("cs_sawyer/error", "")
        self.rospack = rospkg.RosPack()
        self.endpoint = []
        self.last_activity = rospy.Time(0)
        self.speed = speed
        self.acceleration = acceleration
        self.limb = None
        self.rs = None
        self.calibrate = False
        self.calibrate_requested = False
        self.calibrate_state_machine_step = 0
        self.error = False
        self.external_error = False
        self.waiting = {"fear": 0, "hope": 0}  # Votes waiting for execution
        self._sticks = Sticks()
        self._seed = None
        self.motions = self._sticks.get_cartesian_motions()
        self.head = intera_interface.head.Head()
        self.head.set_pan(-1.57)
        self.tfb = tf.TransformBroadcaster()

        with open(join(self.rospack.get_path("cs_sawyer"), "config/poses.json")) as f:
            self.poses = json.load(f)

        rospy.Subscriber("cs_sawyer/button", ButtonPressed, self._cb_button_pressed)
        self.light_pub_fear = rospy.Publisher("cs_sawyer/light/fear", LightStatus, queue_size=1)
        self.light_pub_hope = rospy.Publisher("cs_sawyer/light/hope", LightStatus, queue_size=1)
        rospy.loginfo("Sawyer Interaction Controller is ready!")

    def _cb_endpoint_received(self, msg):
        if self.rs and self.rs.state().enabled and self.rs.state().ready:
            self.endpoint.append(msg)
            if len(self.endpoint) > self.NUM_COLLISION_SAMPLES:
                del self.endpoint[0]
                average_wrench = sum([p.wrench.force.x**2 + p.wrench.force.y**2 + p.wrench.force.z**2 for p in self.endpoint])/len(self.endpoint)
                if not self.calibrate and not self.error and average_wrench > self.WRENCH_LIMIT:
                    self.rs.disable()
                    self.external_error = True
                    self.update_lights(self.ANIMATION_ERROR)
                    rospy.set_param("cs_sawyer/error", "collision")
                    rospy.logerr("COLLISION DETECTED! Wrench limit of {} above {} authorized during {} sec. Move robot and reset.".format(
                        average_wrench, self.WRENCH_LIMIT, len(self.endpoint)/100.))

    def _cb_button_pressed(self, msg):
        if msg.type.data == ButtonPressed.FEAR and not self.error:
            if not self.calibrate:
                rospy.loginfo("Queuing new vote: fear")
                self.waiting["fear"] += 1
                self.save_vote("fear")
            else:
                self.calibrate_requested = True
        elif msg.type.data == ButtonPressed.HOPE and not self.error:
            if not self.calibrate:
                rospy.loginfo("Queuing new vote: hope")
                self.waiting["hope"] += 1
                self.save_vote("hope")
            else:
                self.calibrate_requested = True
        elif msg.type.data == ButtonPressed.RESET:
            self.reset_errors()
            self.save_vote("reset")
        elif msg.type.data == ButtonPressed.CALIBRATE:
            self.calibrate = True
            self.calibrate_requested = True
            self.save_vote("calibration")

    def reset_errors(self):
        rospy.logwarn("Resetting robot power")
        self.rs.enable()
        self.endpoint = []
        rospy.logwarn("Resetting board")
        self.update_lights(self.ANIMATION_OFF)
        if rospy.get_param("cs_sawyer/error", "") == "full":
            rospy.set_param("cs_sawyer/votes/hope/executed", 0)
            rospy.set_param("cs_sawyer/votes/fear/executed", 0)
            self.waiting['fear'] = 0
            self.waiting['hope'] = 0
        rospy.sleep(3)
        # We have just recovered from error, wait a bit for the user...
        self.external_error = False
        rospy.set_param("cs_sawyer/error", "")
        rospy.logwarn("Reset is over, resuming operation...")

    def save_vote(self, type):
        date = strftime("%Y/%B/%d")
        day = strftime("%H_%M_%S")
        vote_dir = join(self.rospack.get_path("cs_sawyer"), "data", date)
        vote_file = join(vote_dir, day + "_{}.json".format(type))
        vote_data = {"type": type, "stamp": time()}
        try:
            if not isdir(vote_dir):
                makedirs(vote_dir)
            with open(vote_file, "w") as f:
                json.dump(vote_data, f)
        except IOError as e:
            rospy.logerr("Can't save vote: " + repr(e))
            self.external_error = True
            rospy.set_param("cs_sawyer/error", "cantsave")
            self.update_lights(self.ANIMATION_ERROR)

    def start_robot(self):
        self.rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self.rs.enable()
        self.limb = intera_interface.Limb('right')
        self._sticks.set_limb(self.limb)
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self._cb_endpoint_received)
        #self.limb.move_to_neutral(speed=self.speed)

    def tuck_robot(self):
        self.limb.move_to_neutral(speed=self.speed)
        self.rs.disable()

    def execute_trajectory(self, trajectory, joint_names, speed=None, acceleration=None):
        """
        trajectory is a list of points: approach (joint), init (cart), drawing (cart), retreat (cart)
        """
        speed = speed if speed is not None else self.speed
        acceleration = acceleration if acceleration is not None else self.acceleration
        if isinstance(trajectory, dict) and "approach" in trajectory:
            for mtype in ["approach", "init", "drawing", "retreat"]:
                points = trajectory[mtype]
                if points["type"] == "joint":
                    self._seed = points["joints"]
                    self.move_to_joint_positions(dict(zip(joint_names, points["joints"])))
                elif points["type"] == "cart":

                    wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed,
                                                     max_joint_accel=acceleration)

                    traj  = MotionTrajectory(limb = self.limb)
                    waypoint = MotionWaypoint(options=wpt_opts, limb=self.limb)
                    t_opt = TrajectoryOptions(interpolation_type=TrajectoryOptions.CARTESIAN)
                    jv = deepcopy(points["joints"])
                    jv.reverse()
                    waypoint.set_joint_angles(jv) 
                    waypoint.set_cartesian_pose(list_to_pose_stamped(points["pose"], frame_id="base"))
                    traj.append_waypoint(waypoint)
                    jn = [str(j) for j in joint_names]
                    jn.reverse()
                    traj.set_joint_names(jn)
                    traj.set_trajectory_options(t_opt)
                    result = traj.send_trajectory(timeout=10)
                    self.last_activity = rospy.Time.now()
                    if result is None:
                        rospy.logerr("Trajectory failed to send")
                    elif not result.result:
                        rospy.logerr('Motion controller failed to complete the trajectory with error %s', result.errorId)
                else:
                    rospy.logwarn("Unknown type %", mtype)
        else:
            rospy.logerr("Incorrect inputs to execute_trajectory")
            return

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
        else:
            rospy.logerr("Incorrect inputs to move_to_joint_positions")
            return

        result = traj.send_trajectory(timeout=30)
        self.last_activity = rospy.Time.now()
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
                rospy.set_param("cs_sawyer/error", "full")
                rospy.logwarn("Board full, please reset...")
            self.error = True
            self.update_lights(self.ANIMATION_ERROR)
        elif self.external_error:
            self.error = True
        else:
            self.error = False
   
    def check_calibration(self):
        if self.calibrate and self.calibrate_requested:
            if self.calibrate_state_machine_step == 0:
                # Waiting for a second press
                self.update_lights(self.ANIMATION_CALIBRATING)
                self.calibrate_state_machine_step = 1
                rospy.logwarn("Prepare init pose and press any button")
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
                rospy.set_param("cs_sawyer/votes/hope/executed", 0)
                rospy.set_param("cs_sawyer/votes/fear/executed", 0)
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
                    elif rospy.Time.now() > self.last_activity + rospy.Duration(5):
                        self.move_to_pause_position()

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

        if vote_id % (self.motions["dimensions"][2] * self.motions["dimensions"][1]) == 0:
            rospy.loginfo("Line full, going through pause position to avoid collisions")
            self.move_to_pause_position()
        self.execute_trajectory(self.motions[type][vote_id], self.motions["joints"], speed=0.6)
        rospy.set_param("cs_sawyer/votes/{}/executed".format(type), vote_id + 1)
        self.update_lights(self.ANIMATION_IDLE)


if __name__=='__main__':
    rospy.init_node('cs_sawyer_interaction_controller')
    ic = InteractionController()
    ic.run()