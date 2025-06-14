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
        self.volume = 20.0
        self.on_start_sound = os.path.expanduser('~/autonav_software_2025/music/mine_xp.mp3')
        self.autonomous_transition_filepath = os.path.expanduser('~/autonav_software_2025/music/imposter2.mp3')
        self.manual_transition_filepath = os.path.expanduser('~/autonav_software_2025/music/manual_mode.mp3')
        self.disabled_transition_filepath = os.path.expanduser('~/autonav_software_2025/music/robot_disabled.mp3')


class AudibleFeedbackNode(Node):
    def __init__(self):
        super().__init__("audible_feedback_node")
        self.config = AudibleFeedbackConfig()
        self.current_playing_thread = None
        self.secondary_tracks = []
        self.main_track = None
        self.old_system_state = self.system_state

    def init(self):
        self.set_device_state(DeviceState.OPERATING)
        self.log("Audible feedback node started", LogLevel.INFO)

        # 
        self.audible_feedback_subscriber = self.create_subscription(
            AudibleFeedback, 
            "/autonav/audible_feedback",
            self.on_audible_feedback_received,
            20
        )

        self.system_state_monitor = self.create_timer(0.5, self.monitor_system_state)
        self.track_monitor = self.create_timer(0.5, self.monitor_tracks)

        self.play_sound(self.config.on_start_sound, False)
    
    def apply_config(self, config):
        self.config.volume = config["volume"]
        self.config.autonomous_transition_filepath = config["autonomous_transition_filepath"]
        self.config.manual_transition_filepath = config["manual_transition_filepath"]
        self.config.disabled_transition_filepath = config["disabled_transition_filepath"]

    def on_audible_feedback_received(self, msg:AudibleFeedback):
        self.log(f"Received audible feedback message: {msg}", LogLevel.INFO)
        self.monitor_tracks()

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
            filename = str(msg.filename)
            main_track = msg.main_track
            try:
                self.play_sound(filename, main_track)
            except:
                return

    def play_sound(self, filename, main_track: bool):
        if main_track and self.main_track is not None:
            return
        
        playback = PySoundSphere.AudioPlayer("ffplay", debug_allow_multiple_playbacks=False)
        try:
            playback.load(filename)
        except:
            self.log("invalid filename", LogLevel.ERROR)
            return
        
        playback.volume = self.config.volume
        
        try:
            playback.play()
            self.log("music")
        except:
            return

        if main_track:
            self.main_track = playback

        else:
            self.secondary_tracks.append(playback)


    def stop_all(self):
        if self.main_track is not None:
            self.log(f"{self.main_track}")

        for track in self.secondary_tracks:
            self.log(f"{track}")

        for track in self.secondary_tracks:
            track.stop()

        self.secondary_tracks = []
        try:
            self.main_track.stop()
        except:
            self.log("No main track", LogLevel.INFO)

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
            self.get_logger().info("Autonomous mode sound")
            playback = PySoundSphere.AudioPlayer("ffplay", debug_allow_multiple_playbacks = False)

            try:
                filename = self.config.autonomous_transition_filepath
            except:
                self.log("invalid autonomous transition filepath", LogLevel.ERROR)
                return
            
            playback.load(filename)
            playback.volume = self.config.volume
            playback.play()
            
            self.secondary_tracks.append(playback)
            self.old_system_state = SystemState.AUTONOMOUS

        elif self.system_state == SystemState.MANUAL and self.old_system_state != SystemState.MANUAL:
            self.get_logger().info("Manual mode sound")
            playback = PySoundSphere.AudioPlayer("ffplay", debug_allow_multiple_playbacks = False)

            try:
                filename = self.config.manual_transition_filepath
            except:
                self.log("invalid manual transition filepath", LogLevel.ERROR)
                return
            
            playback.load(filename)
            playback.volume = self.config.volume
            playback.play()
            
            self.secondary_tracks.append(playback)
            self.old_system_state = SystemState.MANUAL

        elif self.system_state == SystemState.DISABLED and self.old_system_state != SystemState.DISABLED:
            playback = PySoundSphere.AudioPlayer("ffplay", debug_allow_multiple_playbacks = False)

            try:
                filename = self.config.disabled_transition_filepath
            except:
                self.log("invalid disabled transition filepath", LogLevel.ERROR)
                return
            
            playback.load(filename)
            playback.volume = self.config.volume
            playback.play()
            
            self.secondary_tracks.append(playback)
            self.old_system_state = SystemState.DISABLED

        else:
            self.old_system_state = self.system_state


def main():
    rclpy.init()
    audible_feedback_node = AudibleFeedbackNode()
    rclpy.spin(audible_feedback_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
