#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import librosa
import json
import numpy as np
from time import time
from scipy.interpolate import interp1d
from os.path import join, isfile
from subprocess import Popen
from signal import SIGSTOP, SIGCONT


class RoboticBeats(object):
    RATE_HZ = 20
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.path_audio = join(self.rospack.get_path("cs_sawyer"), "beats/eiffel2.ogg")
        self.path_beats = join(self.rospack.get_path("cs_sawyer"), "beats/eiffel2.json")
        # Use a default hop size of 512 samples @ 22KHz ~= 23ms
        self.hop_length = 512
        self.beats = {"tempo": 0, "times": [], "duration": 0}
        self.vlc = None

    def load(self):
        # Load beats peaks
        if isfile(self.path_beats):
            with open(self.path_beats) as f:
                self.beats = json.load(f)
        else:
            self._precompute()
            with open(self.path_beats, "w") as f:
                json.dump(self.beats, f)

        print('Song duration: {:0.2f} seconds, estimated tempo: {:0.2f} beats per minute'.format(self.beats["duration"], self.beats["tempo"]))

        # Load VLC audio player
        self.vlc = Popen(['vlc', '--intf', 'dummy', '--play-and-exit', self.path_audio])
        self.vlc.send_signal(SIGSTOP)

    def _precompute(self):
        print("Precomputing beats...")
        y, sr = librosa.load(self.path_audio, sr=22050)
        tempo, beats = librosa.beat.beat_track(y=y, sr=sr, hop_length=self.hop_length)
        times = librosa.frames_to_time(beats, sr=sr, hop_length=self.hop_length)
        self.beats = {"tempo": tempo, "times": times.tolist(), "duration": librosa.core.get_duration(y, sr)}

        self.x = np.linspace(0, self.beats["duration"], int(self.RATE_HZ*self.beats["duration"]))
    

    @staticmethod
    def get_peak_function(tempo, falling=2.0):
        start = -np.pi
        end = np.pi/falling
        x1 = np.linspace(start, 0, 50)
        x2 = np.linspace(0, end, 40)
        x2_edge = np.linspace(end, np.pi, 10)
        s1 = np.cos(x1)
        s2 = np.cos(falling*x2)
        s2_edge = [-1]*10
        x = np.hstack([x1, x2, x2_edge]) * 60 / (np.pi * tempo)
        s = np.hstack([s1, s2, s2_edge])
        f_peak = interp1d(x, s)
        return f_peak

    def run(self):
        self.vlc.send_signal(SIGCONT)
        next_beat = 0
        t0 = time()
        while not rospy.is_shutdown() and next_beat < len(self.beats["times"]):
            now = time() - t0
            next_beat_time = self.beats["times"][next_beat]
            next_beat += 1
            print("Next"), next_beat_time
            # Wait for the next beat
            while not rospy.is_shutdown() and now < next_beat_time:
                now = time() - t0
                rospy.sleep(0.05)
            print("X")
        self.vlc.terminate()
        self.vlc.wait()

    
        
if __name__ == '__main__':
    rospy.init_node("robotics_beats")
    beats = RoboticBeats()
    beats.load()
    beats.run()