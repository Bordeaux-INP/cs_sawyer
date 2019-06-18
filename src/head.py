#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Robot head manager

import rospy, rospkg
from os.path import join
from os.path import isdir, isfile
from os import listdir
from tf import LookupException
import json, cv2, cv_bridge
from numpy import zeros, uint8
from sensor_msgs.msg import Image
from collections import deque
from cv_bridge import CvBridge
import argparse
from intera_interface import Lights


class HeadDisplay(object):
    RATE_SEC = 6
    def __init__(self, width, height, font=cv2.FONT_HERSHEY_SIMPLEX, scale=1, thickness=1, color=[255]*3, interline=1.1):
        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.font = font
        self.scale = scale
        self.thickness = thickness
        self.color = color
        self.interline = interline
        self.publisher = rospy.Publisher("/robot/head_display", Image, queue_size=1)
        self.width, self.height = width, height
        self.images = {}
        self.votes = {"fear": 0, "hope": 0}
        self.last_screen_update_time = rospy.Time(0)
        self.standby_screens = ["fear", "hope", "intro"]
        self.current_standby_screen = 0
        self.last_fear_num, self.last_hope_num = 0, 0
       
        

        for image in ["error-any", "error-collision",  "error-full",  "fear",  "hope",  "intro"]:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "assets", filename))
            self.images[image] = cv_image

    def load_votes(self):
        self.votes = {"fear": 0, "hope": 0}
        rospy.loginfo("Loading previous votes...")
        vote_dir = join(self.rospack.get_path("cs_sawyer"), "data")
        for dir in listdir(vote_dir):
            year_dir = join(vote_dir, dir)
            if isdir(year_dir):
                for dir in listdir(year_dir):
                    month_dir = join(year_dir, dir)
                    if isdir(month_dir):
                        for dir in listdir(month_dir):
                            day_dir = join(month_dir, dir)
                            if isdir(day_dir):
                                for file in listdir(day_dir):
                                    filename = join(day_dir, file)
                                    if isfile(filename) and file.split('.')[-1].lower() == "json":
                                        #rospy.loginfo("Loading {}".format(filename))
                                        try:
                                            with open(filename) as f:
                                                vote = json.load(f)
                                        except:
                                            rospy.logerr("Can't load {}".format(filename))
                                        else:
                                            if "type" in vote and vote["type"] in ["fear", "hope"]:
                                                self.votes[vote["type"]] += 1                                      
        rospy.loginfo("All votes loaded: fear: {}, hope: {}".format(self.votes["fear"], self.votes["hope"]))

    def blit_text(self, image, text, font=cv2.FONT_HERSHEY_SIMPLEX, scale=2.3, thickness=5, color=[255]*3):
        blit_image = image.copy()
        def horizontal_align(sentence):
            (width, height), _ = cv2.getTextSize(sentence, font, scale, thickness)
            return width
        x, y = self.width/2 - horizontal_align(text), self.height/2 + 95
        cv2.putText(blit_image, text, (x, y), font, scale, color, thickness=thickness)
        return blit_image

    def show_votes(self, type):
        votes_fear_current = rospy.get_param("/cs_sawyer/votes/fear/executed", 0)
        votes_hope_current = rospy.get_param("/cs_sawyer/votes/hope/executed", 0)
        if votes_fear_current < self.last_fear_num or votes_hope_current < self.last_hope_num:
            rospy.logwarn("Detected a jump back in vote number, reloading stats...")
            self.load_votes()
        votes_fear = self.votes["fear"] + votes_fear_current
        votes_hope = self.votes["hope"] + votes_hope_current
        percentage_fear = int(votes_fear*100. / (votes_fear + votes_hope))
        percentage = percentage_fear if type == "fear" else 100 - percentage_fear
        image = self.images[type]
        blit_image = self.blit_text(image, str(percentage))
        self.publish(blit_image)

    def publish(self, image):
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.publisher.publish(ros_image)

    def run(self):
        self.load_votes()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current_error = rospy.get_param("cs_sawyer/error", "")
            error_image = "error-" + current_error
            if current_error != "":
                if error_image in self.images:
                    self.publish(self.images[error_image])
                else:
                    self.publish(self.images["error-any"])
            else:
                if rospy.Time.now() > self.last_screen_update_time + rospy.Duration(self.RATE_SEC):
                    self.last_screen_update_time = rospy.Time.now()
                    self.current_standby_screen = (self.current_standby_screen + 1) % len(self.standby_screens)
                    current_screen = self.standby_screens[self.current_standby_screen]
                    if current_screen in ["hope", "fear"]:
                        self.show_votes(current_screen)
                    elif current_screen in self.images:
                        self.publish(self.images[current_screen])
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('head')
    head = HeadDisplay(1024, 600, font=cv2.FONT_HERSHEY_SIMPLEX, scale=3, thickness=2, interline=2)
    head.run()