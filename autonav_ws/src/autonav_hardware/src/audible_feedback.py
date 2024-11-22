#!/usr/bin/env python3


import rclpy
from autonav_msgs.msg import AudibleFeedback

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import time
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
        self.old_system_state = self.system_state

        # 
        self.audible_feedback_subscriber = self.create_subscription(
            AudibleFeedback, 
            "/autonav/audible_feedback",
            self.on_audible_feedback_received,
            20
        )

        self.system_state_monitor = self.create_timer(0.5, self.monitor_system_state)
        self.track_monitor = self.create_timer(0.5, self.monitor_tracks)

    def on_audible_feedback_received(self, msg:AudibleFeedback):
        self.monitor_tracks()

        self.log(f"{len(self.tracks)}", LogLevel.ERROR)
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


    def monitor_tracks(self):
        for track in self.tracks:
            self.log(f"{track.playing}")
            if track.playing == False:
                track.stop()
                self.tracks.remove(track)


    def monitor_system_state(self):
        self.monitor_tracks()
        if self.system_state == SystemState.AUTONOMOUS and self.old_system_state != SystemState.AUTONOMOUS:
            playback = Playback()
            filename = os.path.expanduser('~/Documents/imposter.mp3')
            playback.load_file(filename)
            playback.play()
            self.tracks.append(playback)
            self.old_system_state = SystemState.AUTONOMOUS

        else:
            self.old_system_state = self.system_state

def main():
    rclpy.init()
    audible_feedback_node = AudibleFeedbackNode()
    rclpy.spin(audible_feedback_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()