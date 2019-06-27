#!/usr/bin/env python
# -*- coding: utf-8 -*-


import intera_interface
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
import rospy
from os.path import join, isdir
import json
import argparse	
import time
import random
from math import sin,pi
import rospkg

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    RandomWalk
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_core_msgs.msg import JointLimits
from intera_interface import (
    Limb,
    JointLimits
)

class Breath(object):


    def __init__(self):
        

        self.robot_state = 0 # normal
        self.breath_state =0 # false
        rospy.Subscriber("cs_sawyer/head_light", UInt8, self.callback_update_breath1)
        rospy.Subscriber("cs_sawyer/breath", Bool, self.callback_update_breath2)
        self.rospack = rospkg.RosPack()

        # Set the trajectory options
        self.limb = Limb()
        traj_opts = TrajectoryOptions()
        traj_opts.interpolation_type = 'JOINT'
        self.traj = MotionTrajectory(trajectory_options = traj_opts, limb = self.limb)

         # Set the waypoint options
        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.05, joint_tolerances=0.7)
                                        #max_joint_accel=0.1)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self.limb)
        # Append a waypoint at the current pose
        waypoint.set_joint_angles(self.limb.joint_ordered_angles())
        #self.traj.append_waypoint(waypoint.to_msg())
        #self.limb.set_joint_position_speed(0.3)

        with open(join(self.rospack.get_path("cs_sawyer"), "config/poses.json")) as f:
            self.poses = json.load(f)

        joint_angles= [self.poses["pause"][j] for j in  [ 'right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']]
        
        j1= joint_angles[1]
        j2= joint_angles[2]
        
        x=0
        while x < 16*pi:
           
            new_j1 = 0.07*sin(x)+j1
            new_j2=0.09*sin(0.5*x)+j2
            
          
            joint_angles[1]=new_j1
            joint_angles[2]=new_j2
     
            x=x+pi/40
            waypoint.set_joint_angles(joint_angles = joint_angles)
            self.traj.append_waypoint(waypoint.to_msg())


    def callback_update_breath1(self,msg):
        self.robot_state = msg.data
        if self.robot_state != 0:
            self.traj.stop_trajectory()

    def callback_update_breath2(self,msg):
        self.breath_state = msg.data
        if not self.breath_state:
            self.traj.stop_trajectory()

            
    def run(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.breath_state and self.robot_state == 0:
                self.traj.send_trajectory(timeout=None)
           
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node("cs_sawyer_breath")
    breath = Breath()
    breath.run()