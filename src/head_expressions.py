#!/usr/bin/env python
# -*- coding: utf-8 -*-



from std_msgs.msg import UInt8
import time
import rospy, rospkg
from os.path import join
from os.path import isdir, isfile
from os import listdir
import intera_interface
import argparse
import rospy
from tf import LookupException
import json, cv2, cv_bridge
from numpy import zeros, uint8
from sensor_msgs.msg import Image
from collections import deque
from cv_bridge import CvBridge


class Head_Expressions(object):


    def __init__(self):
        nbhappy = 2
        nbsad = 2
        nbneutral = 2

        self.img_neutral={}
        self.img_happy={}
        self.img_sad={}
        self.img_error={}

        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher("/robot/head_display", Image, queue_size=1)
       
        self.mylist_happy=[]
        self.mylist_neutral=[]
        self.mylist_sad=[]


########## HAPPY LOAD IMG
        for i in range (0,2):
           num = i+1
           self.mylist_happy.append('c'+str(num))
        
        for image in self.mylist_happy:
            filename = image + ".jpg"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/happy", filename))
            self.img_happy[image] = cv_image


########## ERROR LOAD IMG
        for image in ["error-any", "error-collision",  "error-full"]:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/error", filename))
            self.img_error[image] = cv_image

########## SAD LOAD IMG
        for i in range (0,2):
           num=i+1
           self.mylist_sad.append('s'+str(num))
        
        for image in self.mylist_sad:
            filename = image + ".jpg"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/sad", filename))
            self.img_sad[image] = cv_image


########## NEUTRAL LOAD IMG
        for i in range (0,2):
           num=i+1
           self.mylist_neutral.append('n'+str(num))
        
        for image in self.mylist_neutral:
            filename = image + ".jpg"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/neutral", filename))
            self.img_neutral[image] = cv_image

        self.robot_state = 0 # normal
        rospy.Subscriber("cs_sawyer/head_light", UInt8, self.callback_update_face)
 

    def publish(self, image):
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.publisher.publish(ros_image)

    def callback_update_face(self,msg):
        self.robot_state = msg.data


    def update_face(self):
        if self.robot_state == 0:   # normal
            for img in self.mylist_neutral:
                self.publish(self.img_neutral[img])
                if self.robot_state != 0:
                    break

        elif self.robot_state == 1 :   # error
            current_error = rospy.get_param("cs_sawyer/error", "")
            error_image = "error-" + current_error
            if current_error != "":
                if error_image in self.img_error:
                    self.publish(self.img_error[error_image])
                else:
                    self.publish(self.img_error["error-any"])
            
        elif self.robot_state == 2:  # hope
            for img in self.mylist_happy:
                self.publish(self.img_happy[img])
                if self.robot_state != 2:
                    break

        elif self.robot_state == 3:  # fear
            for img in self.mylist_sad:
                self.publish(self.img_sad[img])
                if self.robot_state != 3:
                    break
            
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_face()                
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("cs_sawyer_head_expressions")
    head_expressions = Head_Expressions()
    head_expressions.run()