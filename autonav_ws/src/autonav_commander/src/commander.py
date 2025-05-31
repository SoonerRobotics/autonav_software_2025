#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from std_msgs.msg import String

import rclpy
import os
import json
import time


IGNORE_NODES = [
    "autonav_commander", # Ourselves
    "UnityEndpoint", # Unity TCP Connection
]
REQUIRED_NODES = [
    "autonav_sd_display"
]
USER_HOME = os.path.expanduser("~")
PRESETS_DIR = os.path.join(USER_HOME, ".autonav", "presets")


class Commander(Node):
    def __init__(self):
        super().__init__("autonav_commander")
        self.alive_nodes = []
        self.preset = {}
        self.preset_id = "default"

        # Setup listener for preset load/save
        self.preset_load_sub = self.create_subscription(
            String, "/autonav/presets/load", self.preset_load_callback, 10
        )
        self.preset_save_sub = self.create_subscription(
            String, "/autonav/presets/save", self.preset_save_callback, 10
        )

        # Initialize the presets directory and load the active preset
        self.initialize_presets()

        # A timer that runs every 250ms
        self.tick_timer = self.create_timer(0.25, self.on_tick, autostart=True)

    def initialize_presets(self):
        # Ensure the presets directory exists
        if not os.path.exists(PRESETS_DIR):
            os.makedirs(PRESETS_DIR)
            
        # Check if the active_preset file exists
        active_preset_path = os.path.join(PRESETS_DIR, "active_preset")
        if not os.path.exists(active_preset_path):
            with open(active_preset_path, "w") as f:
                f.write("default")
        else:
            with open(active_preset_path, "r") as f:
                self.preset_id = f.read().strip()

        # Load the preset configuration
        preset_path = os.path.join(PRESETS_DIR, f"{self.preset_id}.json")
        if os.path.exists(preset_path) and self.preset_id != "default":
            with open(preset_path, "r") as f:
                self.preset = json.load(f)

            self.log(f"Loaded preset {self.preset_id} from {preset_path}", LogLevel.INFO)
        else:
            self.log(f"Preset {self.preset_id} not found. Using default configuration.", LogLevel.WARN)

    def preset_load_callback(self, msg: String):
        # Read the json file given by msg
        preset_path = os.path.join(PRESETS_DIR, f"{msg.data}.json")
        if os.path.exists(preset_path):
            with open(preset_path, "r") as f:
                self.preset = json.load(f)
            self.log(f"Loaded preset {msg.data} from {preset_path}", LogLevel.INFO)
            self.preset_id = msg.data

            # Send the new preset to all nodes
            for node in self.alive_nodes:
                node_cfg = self.preset.get(node)
                if node_cfg is not None:
                    # self._broadcast_config(node, node_cfg)
                    pass

            # Write the active_preset file
            active_preset_path = os.path.join(PRESETS_DIR, "active_preset")
            with open(active_preset_path, "w") as f:
                f.write(msg.data)
        else:
            self.log(f"Preset {msg.data} not found", LogLevel.ERROR)

    def preset_save_callback(self, msg: String):
        configs = self.other_cfgs

        # Write the conigs to the json file given by msg
        preset_path = os.path.join(PRESETS_DIR, f"{msg.data}.json")
        with open(preset_path, "w") as f:
            json.dump(configs, f, indent=4)
        
        self.log(f"Saved preset {msg.data} to {preset_path}", LogLevel.INFO)
        self.preset_id = msg.data
        self.preset = configs

        # Write the active_preset file
        active_preset_path = os.path.join(PRESETS_DIR, "active_preset")
        with open(active_preset_path, "w") as f:
            f.write(msg.data)
        self.log(f"Set active preset to {msg.data}", LogLevel.INFO)

    def can_initialize_node(self, node_name: str) -> bool:
        # Check if all required nodes are present
        missing_node = None
        for required_node in REQUIRED_NODES:
            if required_node not in self.alive_nodes:
                missing_node = required_node
                break

        # If missing_node is not None and node_name is not the missing node, return False
        if missing_node is not None and node_name != missing_node:
            return False
        
        # If the node is in the ignore list, return False
        if node_name in IGNORE_NODES:
            return False
        
        # If the node is already alive, return False
        if node_name in self.alive_nodes:
            return False
        
        # If all checks passed, return True
        return True

    def on_tick(self):
        network = self.get_node_names()

        # Get all "dead" nodes
        dead_nodes = []
        for node in self.alive_nodes:
            if node not in network:
                dead_nodes.append(node)

        # Add all nodes that have not been seen
        new_nodes = []
        for node in network:
            if self.can_initialize_node(node):
                new_nodes.append(node)
                self.alive_nodes.append(node)

        # For all nodes that have been seen, do the following steps
        for node in new_nodes:
            self.log(f"Discovered {node}", LogLevel.DEBUG)
            node_cfg = self.preset.get(node)
            if node_cfg is not None:
                self.log(f"node cfg: {node_cfg}", LogLevel.DEBUG)
                self._broadcast_config(node, node_cfg)

            time.sleep(0.3)

            # Send their updated device state to warming
            self._set_device_state(node, DeviceState.WARMING)

        # For all nodes that have not been seen, do the following steps
        for node in dead_nodes:
            self.log(f"Lost {node}", LogLevel.DEBUG)
            self.alive_nodes.remove(node)
        
def main():
    rclpy.init()
    Node.run_node(Commander())
    rclpy.shutdown()

if __name__ == "__main__":
    main()