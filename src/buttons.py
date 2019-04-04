#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cs_sawyer.msg import ButtonPressed, LightStatus
from std_msgs.msg import Int32
try:
    import RPi.GPIO as GPIO
except ImportError:
    gpio_available = False
else:
    gpio_available = True


class Buttons(object):
    # Raspi 3 GPIO Pinout
    PIN_LED_HOPE = 23
    PIN_BUTTON_HOPE = 24
    PIN_LED_FEAR = 17
    PIN_BUTTON_FEAR = 27

    MIN_PRESS_INTERVAL = 7      # Interval in seconds in which a second press will be ignored

    FAST_BLINK_DURATION = 0.25  # Second of blinking on and off
    SLOW_BLINK_DURATION = 1     # Second of blinking on and off

    def __init__(self):
        self.last_pressed_time = rospy.Time(0)
        self.publisher = rospy.Publisher("cs_sawyer/button", ButtonPressed, queue_size=1)
        self.fear_led_status = LightStatus.OFF
        self.hope_led_status = LightStatus.OFF
        self._fear_led_on = False
        self._hope_led_on = False

        rospy.Subscriber("cs_sawyer/light/fear", LightStatus, self._fear_light_update)
        rospy.Subscriber("cs_sawyer/light/hope", LightStatus, self._hope_light_update)

        if gpio_available:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.PIN_BUTTON_HOPE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.PIN_BUTTON_FEAR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.PIN_LED_HOPE, GPIO.OUT)
            GPIO.setup(self.PIN_LED_FEAR, GPIO.OUT)
        else:
            rospy.logwarn("Node hasn't found the GPIO, buttons will not work")
        rospy.loginfo("Buttons node is started!")

    def _fear_light_update(self, msg):
        self.fear_led_status = msg.type.data

    def _hope_light_update(self, msg):
        self.hope_led_status = msg.type.data

    def pressed(self, button_id):
        if gpio_available:
            return GPIO.input(button_id)
        else:
            return False

    def update_leds(self):
        if gpio_available:
            now = rospy.Time.now()
            # Hope LED update
            if self.hope_led_status == LightStatus.ON:
                GPIO.output(self.PIN_LED_HOPE, True)
                self.last_hope_led_update_time = rospy.Time.now()
            elif self.hope_led_status == LightStatus.OFF:
                GPIO.output(self.PIN_LED_HOPE, False)
                self.last_hope_led_update_time = rospy.Time.now()
            elif self.hope_led_status == LightStatus.FAST_BLINK:
                if now > self.last_hope_led_update_time + rospy.Duration(self.FAST_BLINK_DURATION):
                    self._hope_led_on = not self._hope_led_on
                    GPIO.output(self.PIN_LED_HOPE, self._hope_led_on)
                    self.last_hope_led_update_time = now
            elif self.hope_led_status == LightStatus.SLOW_BLINK:
                if now > self.last_hope_led_update_time + rospy.Duration(self.SLOW_BLINK_DURATION):
                    self._hope_led_on = not self._hope_led_on
                    GPIO.output(self.PIN_LED_HOPE, self._hope_led_on)
                    self.last_hope_led_update_time = now

            # Fear LED update
            if self.fear_led_status == LightStatus.ON:
                GPIO.output(self.PIN_LED_FEAR, True)
                self.last_fear_led_update_time = rospy.Time.now()
            elif self.fear_led_status == LightStatus.OFF:
                GPIO.output(self.PIN_LED_FEAR, False)
                self.last_fear_led_update_time = rospy.Time.now()
            elif self.fear_led_status == LightStatus.FAST_BLINK:
                if now > self.last_fear_led_update_time + rospy.Duration(self.FAST_BLINK_DURATION):
                    self._fear_led_on = not self._fear_led_on
                    GPIO.output(self.PIN_LED_FEAR, self._fear_led_on)
                    self.last_fear_led_update_time = now
            elif self.fear_led_status == LightStatus.SLOW_BLINK:
                if now > self.last_fear_led_update_time + rospy.Duration(self.SLOW_BLINK_DURATION):
                    self._fear_led_on = not self._fear_led_on
                    GPIO.output(self.PIN_LED_FEAR, self._fear_led_on)
                    self.last_fear_led_update_time = now

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_leds()
            # TODO: very long press should not increment the counter
            now = rospy.Time.now()
            if self.pressed(self.PIN_BUTTON_HOPE) and now > self.last_pressed_time + rospy.Duration(self.MIN_PRESS_INTERVAL):
                self.last_pressed_time = now
                self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.HOPE)))
            elif self.pressed(self.PIN_BUTTON_FEAR) and now > self.last_pressed_time + rospy.Duration(self.MIN_PRESS_INTERVAL):
                self.last_pressed_time = now
                self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.FEAR)))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("cs_sawyer_buttons")
    buttons = Buttons()
    buttons.run()