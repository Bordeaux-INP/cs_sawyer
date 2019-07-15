#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import UInt8
import time
import random
import rospy, rospkg
from os.path import join
from os.path import isdir, isfile
from os import listdir
import intera_interface
import argparse
from playsound import playsound
from tf import LookupException
import json, cv2, cv_bridge
from numpy import zeros, uint8
#from pygame import mixer

from sensor_msgs.msg import Image
from collections import deque
from cv_bridge import CvBridge


class Head_Expressions(object):

    RATE_SEC = 40

    def __init__(self):
        rospy.on_shutdown(self.screen_off)
        nbhappy = 150
        nbsad = 150
        nbneutral = 150
        nbattention = 70
        nbsound = 11
        self.img_neutral = {}
        self.img_happy = {}
        self.img_sad = {}
        self.img_error = {}
        self.img_attention = {}
        self.sound_attention = {}

        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher("/robot/head_display", Image, queue_size=1)
        self.last_catch_attention_time = rospy.Time(0)
        self.list_happy = []
        self.list_neutral = []
        self.list_sad = []
        self.list_attention = []
        self.list_sound = []
        #mixer.init()



########## HAPPY LOAD IMG

        for i in range (0,nbhappy):
            img= 'c_0000'
            if i <10:
               img=img+str(i)
            elif i<100:
                img=img[:-1]
                img=img+str(i)
            else:
                img=img[:-2]
                img=img+str(i)

            self.list_happy.append(img)

        for image in self.list_happy:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/happy", filename))
            self.img_happy[image] = cv_image

       


########## ERROR LOAD IMG
        for image in ["error-any", "error-collision",  "error-full"]:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/error", filename))
            self.img_error[image] = cv_image

########## SAD LOAD IMG
        for i in range (0,nbsad):
            img= 't_0000'
            if i <10:
               img=img+str(i)
            elif i<100:
                img=img[:-1]
                img=img+str(i)
            else:
                img=img[:-2]
                img=img+str(i)

            self.list_sad.append(img)
        for image in self.list_sad:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/sad", filename))
            self.img_sad[image] = cv_image


########## NEUTRAL LOAD IMG
        for i in range (0,nbneutral):
            img= 'n_0000'
            if i <10:
               img=img+str(i)
            elif i<100:
                img=img[:-1]
                img=img+str(i)
            else:
                img=img[:-2]
                img=img+str(i)

            self.list_neutral.append(img)
        
        for image in self.list_neutral:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/neutral", filename))
            self.img_neutral[image] = cv_image

        
 
########### ATTENTION LOAD IMG

        for i in range (0,nbattention):
            img= 'a_0000'
            if i <10:
               img=img+str(i)
            elif i<100:
                img=img[:-1]
                img=img+str(i)
            else:
                img=img[:-2]
                img=img+str(i)

            self.list_attention.append(img)
        for image in self.list_attention:
            filename = image + ".png"
            cv_image = cv2.imread(join(self.rospack.get_path("cs_sawyer"), "images/attention", filename))
            self.img_attention[image] = cv_image



############ LOAD SOUND
        for i in range (0,nbsound):
           num = i+1
           self.list_sound.append(str(num))
        
        for sound in self.list_sound:
            filename = sound + ".mp3"
            path = join(self.rospack.get_path("cs_sawyer"), "sound/attention", filename)
            self.sound_attention[sound] = path

    

        self.robot_state = 0 # normal
        rospy.Subscriber("cs_sawyer/head_light", UInt8, self.callback_update_face)

    def publish(self, image):
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.publisher.publish(ros_image)

    def callback_update_face(self,msg):
        self.robot_state = msg.data


    def update_face(self):
        if self.robot_state == 0:   # normal
            
            if self.last_catch_attention_time == rospy.Time(0):
                self.last_catch_attention_time = rospy.Time.now()
            
            elif rospy.Time.now() > self.last_catch_attention_time + rospy.Duration(self.RATE_SEC):  # to catch the attention of visitors if nobody have push the button since 40 sec
                    self.last_catch_attention_time = rospy.Time.now()
                  
                    for img in self.list_attention:
                        
                        if img=="a_00041":
                            playsound(self.sound_attention[str(random.randint(1, 11))])
                        self.publish(self.img_attention[img])
                        if self.robot_state != 0:
                            break
                        else:
                            time.sleep(0.06)
                    
            
            for img in self.list_neutral:
                self.publish(self.img_neutral[img])
                time.sleep(0.03)
                if self.robot_state != 0:
                    break

        elif self.robot_state == 1 :   # error
            self.last_catch_attention_time = rospy.Time(0)
            current_error = rospy.get_param("cs_sawyer/error", "")
            error_image = "error-" + current_error
            if current_error != "":
                if error_image in self.img_error:
                    self.publish(self.img_error[error_image])
                else:
                    self.publish(self.img_error["error-any"])
            
        elif self.robot_state == 2:  # hope
            self.last_catch_attention_time = rospy.Time(0)
            #tab_sound=[join(self.rospack.get_path("cs_sawyer"), "sound/happy", "2.mp3"),join(self.rospack.get_path("cs_sawyer"), "sound/happy", "4.mp3")]
            #mixer.music.load(tab_sound[random.randint(0, 1)]) # Paste The audio file location 
            #mixer.music.play()
            for img in self.list_happy:
                self.publish(self.img_happy[img])
                time.sleep(0.03)
                if self.robot_state != 2:
                    break

        elif self.robot_state == 3:  # fear
            self.last_catch_attention_time = rospy.Time(0)
            
            #mixer.music.load(join(self.rospack.get_path("cs_sawyer"), "sound/sad", "1.mp3")) # Paste The audio file location 
            #mixer.music.play()
            for img in self.list_sad:
                self.publish(self.img_sad[img])
                time.sleep(0.03)
                if self.robot_state != 3:
                    break
            #mixer.music.stop()
      
    def screen_off(self):
        self.publish(self.img_neutral[self.list_neutral[15]])
     

            
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update_face()                
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("cs_sawyer_head_expressions")
    head_expressions = Head_Expressions()
    head_expressions.run()