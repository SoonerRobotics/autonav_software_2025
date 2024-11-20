#!/usr/bin/env python3

import pydub.playback
import rclpy
from autonav_msgs.msg import AudibleFeedback

import pydub
import simpleaudio

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import time
import threading
import multiprocessing
import kthread
import os

class AudibleFeedbackConfig:
    def __init__(self):
        self.volume = 100
        

class AudibleFeedbackNode(Node):
    def __init__(self):
        super().__init__("audible_feedback_node")
        self.write_config(AudibleFeedbackConfig())
        self.current_playing_thread = None
        self.tracks = []

        # 
        self.audible_feedback_subscriber = self.create_subscription(
            AudibleFeedback, 
            "autonav/audible_feedback",
            self.on_audible_feedback_received,
            20
        )


    def on_audible_feedback_received(self, msg:AudibleFeedback):
        if msg.stop_all:
            self.stop_all()

        else:
            self.log(f"playing {msg.filename}")
            filename = str(msg.filename)
            filetype = str(msg.filetype)

            self.log(f"heard request to play {msg.filename}")
            current_playing_process = threading.Thread(target=self.play_sound, args=[filename, filetype])
            self.tracks.append(current_playing_process)
            current_playing_process.start()


    def play_sound(self, filename, filetype):
        if filetype == "wav":
            sound = pydub.AudioSegment.from_wav(filename)
        elif filetype == "mp3":
            sound = pydub.AudioSegment.from_mp3(filename)
        else:
            sound = pydub.AudioSegment.from_file(filename)

        playback = pydub.playback._play_with_ffplay(sound)


    def stop_all(self):
        for track in self.tracks:
            self.log(f"{track}")


def main():
    rclpy.init()
    audible_feedback_node = AudibleFeedbackNode()
    rclpy.spin(audible_feedback_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()