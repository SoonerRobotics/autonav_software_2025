import rclpy
from autonav_msgs.msg import AudibleFeedback

from autonav_shared.node import Node
from autonav_shared.types import LogLvel, DeviceState, SystemState
import time


class AudibleFeedbackConfig:
    def init(self):
        self.volume = 100


class AudibleFeedbackNode(Node):
    def __init__(self):
        super().__init__("audible_feedback_node")
        self.write_config()