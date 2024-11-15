#!/usr/bin/env python3

import pydub.playback
import rclpy
from autonav_msgs.msg import AudibleFeedback

import pydub

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import time
import threading
import os

class AudibleFeedbackConfig:
    def __init__(self):
        self.volume = 100
        

class AudibleFeedbackNode(Node):
    def __init__(self):
        super().__init__("audible_feedback_node")
        self.write_config(AudibleFeedbackConfig())
        self.main_sound_head_ms = 0 # milliseconds
        self.current_playing_thread = None

        # 
        self.audible_feedback_subscriber = self.create_subscription(
            AudibleFeedback, 
            "autonav/audible_feedback",
            self.on_audible_feedback_received,
            20
        )


    def on_audible_feedback_received(self, msg:AudibleFeedback):
        self.log(f"playing {msg.filename}")
        filename = str(msg.filename)
        if self.current_playing_thread is not None:
            self.current_playing_thread.kill()
            self.log("setting do run to false")
            self.current_playing_thread.do_run = False
        
        if msg.main_song == True:
            self.log(f"playing main song")
            self.current_playing_thread = threading.Thread(target=self.play_main_sound, args=[filename])
            self.current_playing_thread.start()
            self.log("other thread continuing")
        else:
            self.current_playing_thread = threading.Thread(target=self.play_secondary_sound, args=[filename])
            self.current_playing_thread.start()
            self.log("other thread continuing")


    def play_main_sound(self, filename):
        sound = pydub.AudioSegment.from_wav(filename)
        self.log(f"created sound {sound}")
        sound_at_start = sound[self.main_sound_head_ms:]
        t = threading.current_thread()

        while getattr(t, "do_run", True):
            start_time = time.time()
            playback = pydub.playback.play(sound_at_start)
            print("playback thread continuing")

        self.log("INTERRUPT")
        playback.stop()
        end_time = time.time()
        time_elapsed_ms = (end_time - start_time) * 1000
        self.main_sound_head_ms += time_elapsed_ms
        self.log("interrupt")


    def play_secondary_sound(self, filename):
        pass

def main():
    rclpy.init()
    audible_feedback_node = AudibleFeedbackNode()
    rclpy.spin(audible_feedback_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()