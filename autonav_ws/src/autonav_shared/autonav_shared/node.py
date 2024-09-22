from rclpy.node import Node as RclpyNode
from autonav_shared.types import DeviceState, LogLevel, SystemState
from autonav_msgs.msg import SystemState as SystemStateMsg, DeviceState as DeviceStateMsg
import sty
import time
import inspect


class Node(RclpyNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        # Setup our device state
        self.system_state = SystemState.DISABLED
        self.device_states = {}
        self.device_states[name] = DeviceState.OFF
        
        # TODO: Setup all relevant publishers, subscribers, services, clients, etc
        self.system_state_sub = self.create_subscription(SystemStateMsg, "/autonav/shared/system", self.system_state_callback, 10)
        self.device_state_sub = self.create_subscription(DeviceStateMsg, "/autonav/shared/device", self.device_state_callback, 10)
        
    def system_state_callback(self, msg: SystemStateMsg) -> None:
        """
        Callback for the system state topic.
        """
        self.system_state = SystemState(msg.state)
        
    def device_state_callback(self, msg: DeviceStateMsg) -> None:
        """
        Callback for the device state topic.
        """
        self.device_states[msg.device] = DeviceState(msg.state)
        
    def log(self, message: str, level: LogLevel = LogLevel.INFO) -> None:
        """
        Log a message with a given log level.
        """
        # Get the current time as yyyy-mm-dd hh:mm:ss:ms
        current_time = time.strftime("%Y-%m-%d %H:%M:%S:", time.localtime()) + str(time.time() % 1)[2:5]
        
        # Get the calling function name and line number
        frame = inspect.currentframe().f_back
        calling_function = frame.f_code.co_name
        line_number = frame.f_lineno
        
        # Get the log level as a string
        level_str = LogLevel(level).name
        level_str = level_str + " " * (5 - len(level_str))
        
        match level:
            case LogLevel.DEBUG: # 61, 117, 157
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(61, 117,157)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(61, 117, 157)}{message}{sty.rs.all}")
                
            case LogLevel.INFO: # 255, 255, 255
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(255, 255, 255)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(255, 255, 255)}{message}{sty.rs.all}")
                
            case LogLevel.WARN: # 226, 174, 47
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(226, 174, 47)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(226, 174, 47)}{message}{sty.rs.all}")
                
            case LogLevel.ERROR: # 195, 59, 91
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(195, 59, 91)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(195, 59, 91)}{message}{sty.rs.all}")
                
            case LogLevel.FATAL: # 207, 62, 97
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.bg(207, 62, 97)}{level_str}{sty.bg.rs} {sty.fg.white}| {sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.bg(207, 62, 97)}{message}{sty.rs.all}")