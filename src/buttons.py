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

    RESET_PRESS_DURATION = 4    # Press at least this time to send reset signal
    MIN_PRESS_INTERVAL = 5      # Interval in seconds in which a second press will be ignored
    TIMEOUT_DURATION = 60       # Timeout duration if no LED update message is received from the controller

    FAST_BLINK_DURATION = 0.25  # Second of blinking on and off
    SLOW_BLINK_DURATION = 1     # Second of blinking on and off

    def __init__(self):
        self.last_pressed_time = rospy.Time(0)
        self.last_received_led_time = rospy.Time(0)
        self.last_hope_led_update_time = rospy.Time(0)
        self.last_fear_led_update_time = rospy.Time(0)
        self.first_pressed_time_hope = None
        self.first_pressed_time_fear = None
        self.publisher = rospy.Publisher("cs_sawyer/button", ButtonPressed, queue_size=1)
        self.fear_led_status = LightStatus.OFF
        self.hope_led_status = LightStatus.OFF
        self.fear_led_status_previous = LightStatus.OFF
        self.hope_led_status_previous = LightStatus.OFF
        self.led_synced = False
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
        self.fear_led_status_previous = self.fear_led_status
        self.fear_led_status = msg.type.data
        self.last_received_led_time = rospy.Time.now()

    def _hope_light_update(self, msg):
        self.hope_led_status_previous = self.hope_led_status
        self.hope_led_status = msg.type.data
        self.last_received_led_time = rospy.Time.now()

    def pressed(self, button_id):
        if gpio_available:
            return GPIO.input(button_id)
        else:
            return False

    def update_leds(self):
        if gpio_available:
            now = rospy.Time.now()

            # Measure timeout
            if now > self.last_received_led_time + rospy.Duration(self.TIMEOUT_DURATION):
                self.hope_led_status_previous = self.hope_led_status
                self.fear_led_status_previous = self.fear_led_status
                self.hope_led_status = LightStatus.FAST_BLINK
                self.fear_led_status = LightStatus.FAST_BLINK

            # Hack to sync LEDs when they're blinking the same way
            if self.hope_led_status_previous != self.hope_led_status or self.fear_led_status_previous != self.fear_led_status:
                if self.hope_led_status == self.fear_led_status and self.hope_led_status in [LightStatus.SLOW_BLINK, LightStatus.FAST_BLINK]:
                    if not self.led_synced:
                        self.last_fear_led_update_time = now
                        self.last_hope_led_update_time = now
                        self._hope_led_on = True
                        self._fear_led_on = True
                        self.led_synced = True
                else:
                    self.led_synced = False

            # Hope LED update
            if self.hope_led_status == LightStatus.ON:
                GPIO.output(self.PIN_LED_HOPE, True)
                self.last_hope_led_update_time = now
            elif self.hope_led_status == LightStatus.OFF:
                GPIO.output(self.PIN_LED_HOPE, False)
                self.last_hope_led_update_time = now
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
                self.last_fear_led_update_time = now
            elif self.fear_led_status == LightStatus.OFF:
                GPIO.output(self.PIN_LED_FEAR, False)
                self.last_fear_led_update_time = now
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
            if self.pressed(self.PIN_BUTTON_HOPE):
                if self.first_pressed_time_hope is None:
                    self.first_pressed_time_hope = now
                if now > self.last_pressed_time + rospy.Duration(self.MIN_PRESS_INTERVAL):
                    self.last_pressed_time = now
                    self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.HOPE)))
            else:
                self.first_pressed_time_hope = None
            
            if self.pressed(self.PIN_BUTTON_FEAR):
                if self.first_pressed_time_fear is None:
                    self.first_pressed_time_fear = now
                if now > self.last_pressed_time + rospy.Duration(self.MIN_PRESS_INTERVAL):
                    self.last_pressed_time = now
                    self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.FEAR)))
            else:
                self.first_pressed_time_fear = None

            # Reset signal
            if self.first_pressed_time_hope is not None and self.first_pressed_time_fear is not None:
                if now > self.first_pressed_time_hope + rospy.Duration(self.RESET_PRESS_DURATION) and \
                    now > self.first_pressed_time_fear + rospy.Duration(self.RESET_PRESS_DURATION):
                    self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.RESET)))
                    self.first_pressed_time_hope = None
                    self.first_pressed_time_fear = None

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("cs_sawyer_buttons")
    buttons = Buttons()
    buttons.run()