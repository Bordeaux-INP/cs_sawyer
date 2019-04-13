#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Display static poses of robot
# Publish desired poses on /display/joint_states

import rospy
from sensor_msgs.msg import JointState
from rospy import ROSException

ordered_joint_names = [u'right_j0', u'right_j1', u'right_j2', u'right_j3', u'right_j4', u'right_j5', u'right_j6',
                       u'head_pan', u'right_gripper_l_finger_joint', u'right_gripper_r_finger_joint']

js = JointState(name=ordered_joint_names)
js.position = [0 for j in ordered_joint_names]
js_publisher = rospy.Publisher('joint_states', JointState, queue_size=45)

def js_callback(msg):
    if len(msg.name) == len(msg.position):
        for j in range(len(msg.position)):
            if len(js.position) > j:
                try:
                    i = js.name.index(msg.name[j])
                except IndexError:
                    rospy.logerr("Can't deal with joint {}".format(msg.name[j]))
                else:
                    js.position[i] = msg.position[j]
    else:
        rospy.logerr("Published joint states have {} joint names but {} joint positions".format(len(msg.name), len(msg.position)))

rospy.init_node('joint_state_publisher')

rospy.Subscriber("display/joint_states", JointState, js_callback)
rospy.loginfo("Starting Sawyer joint state displayer... You can publish states to display/joint_states")

if __name__ == '__main__':
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        js.header.stamp = rospy.Time.now()
        try:
            js_publisher.publish(js)
        except ROSException:
            pass
        else:
            r.sleep()
