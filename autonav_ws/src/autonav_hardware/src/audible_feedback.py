#!/usr/bin/env python3


import rclpy
from autonav_msgs.msg import AudibleFeedback

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import time
import os
from just_playback import Playback
import PySoundSphere


class AudibleFeedbackConfig:
    def __init__(self):
        self.volume = 1.0
        

class AudibleFeedbackNode(Node):
    def __init__(self):
        super().__init__("audible_feedback_node")
        self.write_config(AudibleFeedbackConfig())
        self.current_playing_thread = None
        self.secondary_tracks = []
        self.main_track = None
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

        self.log(f"{len(self.secondary_tracks)}", LogLevel.ERROR)
        if msg.stop_all:
            self.stop_all()
            return

        if msg.pause_all:
            self.pause_main_track()
            return

        if msg.unpause_all:
            self.unpause_main_track()
            return

        else:
            # self.log(f"playing {msg.filename}")
            filename = str(msg.filename)
            main_track = msg.main_track

            # self.log(f"heard request to play {msg.filename}")
            self.play_sound(filename, main_track)


    def play_sound(self, filename, main_track: bool):
        playback = PySoundSphere.AudioPlayer("ffplay", debug_allow_multiple_playbacks=False)
        playback.load(filename)
        playback.volume = self.config.get('volume')
        playback.play()

        if main_track:
            self.main_track = playback
        else:
            self.secondary_tracks.append(playback)


    def stop_all(self):
        for track in self.secondary_tracks:
            self.log(f"{track}")

        for track in self.secondary_tracks:
            track.stop()

        self.secondary_tracks = []
        self.main_track.stop()
        self.main_track = None


    def pause_main_track(self):
        try:
            self.main_track.pause()
        except: # main track is paused or doesn't exist
            pass


    def unpause_main_track(self):
            try:
                self.main_track.play()

            except: # main track is playing or doesn't exist
                pass


    def monitor_tracks(self):
        if len(self.secondary_tracks) > 16:
            self.secondary_tracks.pop()


    def monitor_system_state(self):
        self.monitor_tracks()
        if self.system_state == SystemState.AUTONOMOUS and self.old_system_state != SystemState.AUTONOMOUS:
            playback = PySoundSphere.AudioPlayer("ffplay", debug_allow_multiple_playbacks = False)
            filename = os.path.expanduser('~/Documents/imposter.mp3')
            playback.load(filename)
            playback.volume = self.config.get('volume')
            playback.play()
            
            self.secondary_tracks.append(playback)
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