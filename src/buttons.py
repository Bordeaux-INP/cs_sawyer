#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cs_sawyer.msg import ButtonPressed, LightStatus
from std_msgs.msg import Int32
from std_msgs.msg import UInt8
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

    RESET_PRESS_DURATION = 9    # Press at least this time to send reset signal
    CALIB_PRESS_DURATION = 19   # Press at least this time to send calibrate signal
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
        self.last_button_states = {'hope': False, 'fear': False}
        self.publisher = rospy.Publisher("cs_sawyer/button", ButtonPressed, queue_size=1)
        self.state_publisher = rospy.Publisher("/cs_sawyer/head_light",UInt8,queue_size=1)
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

    def current_press_state(self, emotion):
        return GPIO.input(self.PIN_BUTTON_FEAR if emotion == 'fear' else self.PIN_BUTTON_HOPE)

    def pressed(self, emotion, current_state):
        if gpio_available:
            # Considered pressed only on falling edge
            pressed = self.last_button_states[emotion] and not current_state
            self.last_button_states[emotion] = current_state
            return pressed
        else:
            return False

    def update_leds(self, now):
        if gpio_available:
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

    def update_buttons(self, now):
        hope_current_press = self.current_press_state('hope')   # instant
        fear_current_press = self.current_press_state('fear')   # instant
        hope_pressed = self.pressed('hope', hope_current_press) # falling edge
        fear_pressed = self.pressed('fear', fear_current_press) # falling edge

        # UPDATE LONG PRESS SIGNALS IF ANY
        if self.first_pressed_time_hope is not None and self.first_pressed_time_fear is not None:
            if now > self.first_pressed_time_hope + rospy.Duration(self.RESET_PRESS_DURATION) and \
                now > self.first_pressed_time_fear + rospy.Duration(self.RESET_PRESS_DURATION) and \
                    not (fear_current_press and hope_current_press):
                    self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.RESET)))
                    self.first_pressed_time_hope = None
                    self.first_pressed_time_fear = None
                    self.last_pressed_time = now
            elif now > self.first_pressed_time_hope + rospy.Duration(self.CALIB_PRESS_DURATION) and \
                now > self.first_pressed_time_fear + rospy.Duration(self.CALIB_PRESS_DURATION):
                self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.CALIBRATE)))
                self.first_pressed_time_hope = None
                self.first_pressed_time_fear = None
                self.last_pressed_time = now

        # INSTANT PRESSES BEHAVIOURS
        if hope_current_press:
            if self.first_pressed_time_hope is None:
                self.first_pressed_time_hope = now
        elif not fear_current_press:
            self.first_pressed_time_hope = None

        if fear_current_press:
            if self.first_pressed_time_fear is None:
                self.first_pressed_time_fear = now
        elif not hope_current_press:
            self.first_pressed_time_fear = None

        # FALLING EDGES BEHAVIOURS
        if hope_pressed:
            if now > self.last_pressed_time + rospy.Duration(self.MIN_PRESS_INTERVAL):
                if not fear_current_press:  # This excludes mutual presses (reset and calibrate)
                    self.last_pressed_time = now
                    self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.HOPE)))           
        if fear_pressed:
            if now > self.last_pressed_time + rospy.Duration(self.MIN_PRESS_INTERVAL):
                if not hope_current_press:  # This excludes mutual presses (reset and calibrate)
                    self.last_pressed_time = now
                    self.publisher.publish(ButtonPressed(type=Int32(ButtonPressed.FEAR)))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.update_leds(now)
            self.update_buttons(now)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("cs_sawyer_buttons")
    buttons = Buttons()
    buttons.run()