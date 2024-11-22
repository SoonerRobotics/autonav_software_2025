#!/usr/bin/env python3


import rclpy
from autonav_msgs.msg import AudibleFeedback

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import time
import threading
import just_playback
import os
from just_playback import Playback

# TODO: rewrite with just_playback

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
            # self.log(f"playing {msg.filename}")
            filename = str(msg.filename)

            # self.log(f"heard request to play {msg.filename}")
            self.play_sound(filename)


    def play_sound(self, filename):
        playback = Playback()
        playback.load_file(filename)
        playback.play()

        self.tracks.append(playback)


    def stop_all(self):
        for track in self.tracks:
            self.log(f"{track}")

        for track in self.tracks:
            track.stop()


def main():
    rclpy.init()
    audible_feedback_node = AudibleFeedbackNode()
    rclpy.spin(audible_feedback_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()