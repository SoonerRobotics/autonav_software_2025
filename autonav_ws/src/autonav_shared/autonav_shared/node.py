import rclpy
from rclpy.node import Node as RclpyNode
from autonav_shared.types import DeviceState, LogLevel, SystemState
from autonav_msgs.msg import SystemState as SystemStateMsg, DeviceState as DeviceStateMsg
from autonav_msgs.srv import SetDeviceState, SetSystemState
import sty
import time
import inspect


class Node(RclpyNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        # Setup our device state
        self.system_state = SystemState.DISABLED
        self.mobility = False
        self.device_states = {}
        self.device_states[name] = DeviceState.OFF
        
        # TODO: Setup all relevant publishers, subscribers, services, clients, etc
        self.system_state_sub = self.create_subscription(SystemStateMsg, "/autonav/shared/system", self.system_state_callback, 10)
        self.device_state_sub = self.create_subscription(DeviceStateMsg, "/autonav/shared/device", self.device_state_callback, 10)
        
        self.set_device_state_client = self.create_client(SetDeviceState, "/autonav/shared/set_device_state")
        self.set_system_state_client = self.create_client(SetSystemState, "/autonav/shared/set_system_state")

        self.set_device_state(DeviceState.WARMING)

    def init(self) -> None:
        """
        Called when the node synchronizes with the system.
        """
        pass

    def system_state_callback(self, msg: SystemStateMsg) -> None:
        """
        Callback for the system state topic.
        """
        self.system_state = SystemState(msg.state)
        self.mobility = msg.mobility
        
    def device_state_callback(self, msg: DeviceStateMsg) -> None:
        """
        Callback for the device state topic.
        """
        if msg.device == self.get_name():
            self.log(f"Received update on our own device state from {DeviceState(self.device_states[msg.device])} to {DeviceState(msg.state)}", LogLevel.DEBUG)

        old_state = self.device_states[msg.device]
        self.device_states[msg.device] = DeviceState(msg.state)

        if old_state == None or old_state == DeviceState.OFF and DeviceState(msg.state) == DeviceState.WARMING:
            self.init()

    def set_device_state(self, state: DeviceState) -> None:
        """
        Set the state of our device.
        """
        msg = SetDeviceState.Request()
        msg.device = self.get_name()
        msg.state = state.value

        while not self.set_device_state_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.log("Interrupted while waiting for the set_device_state service", LogLevel.FATAL)
                return
            
            self.log("Service set_device_state not available, waiting again...", LogLevel.WARN)

        # Send the request, use the callback to handle the response
        future = self.set_device_state_client.call_async(msg)
        future.add_done_callback(self.set_device_state_callback)

    def set_device_state_callback(self, future) -> None:
        """
        Callback for the set_device_state service.
        """
        try:
            result = future.result()
            if result is None or not result.ok:
                self.log("Failed to set device state", LogLevel.ERROR)
            else:
                self.log("Successfully set device state", LogLevel.DEBUG)
        except Exception as e:
            self.log(f"Failed to set device state: {e}", LogLevel.ERROR)
        
    def set_system_state(self, state: SystemState) -> None:
        """
        Set the state of the system.
        """
        msg = SetSystemState.Request()
        msg.state = state.value
        msg.mobility = self.mobility

        while not self.set_system_state_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.log("Interrupted while waiting for the set_system_state service", LogLevel.FATAL)
                return
            
            self.log("Service set_system_state not available, waiting again...", LogLevel.WARN)

        # Send the request, use the callback to handle the response
        future = self.set_system_state_client.call_async(msg)
        future.add_done_callback(self.set_system_state_callback)

    def set_mobility(self, mobility: bool) -> None:
        """
        Set the mobility of the system.
        """
        msg = SetSystemState.Request()
        msg.state = self.system_state.value
        msg.mobility = mobility

        while not self.set_system_state_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.log("Interrupted while waiting for the set_system_state service", LogLevel.FATAL)
                return
            
            self.log("Service set_system_state not available, waiting again...", LogLevel.WARN)

        # Send the request, use the callback to handle the response
        future = self.set_system_state_client.call_async(msg)
        future.add_done_callback(self.set_system_state_callback)
        
    def set_system_state_callback(self, future) -> None:
        """
        Callback for the set_system_state service.
        """
        try:
            result = future.result()
            if result is None or not result.ok:
                self.log("Failed to set system state", LogLevel.ERROR)
            else:
                self.log("Successfully set system state", LogLevel.DEBUG)
        except Exception as e:
            self.log(f"Failed to set system state: {e}", LogLevel.ERROR)

    def get_device_state(self, device: str = None) -> DeviceState:
        """
        Get the state of a specific device.

        If no device is provided, the state of the current node is returned.
        """
        if device is None:
            device = self.get_name()

        if device not in self.device_states:
            self.log(f"Device {device} not found in device states", LogLevel.ERROR)
            return DeviceState.UNKNOWN
        
        return self.device_states[device]
    
    def get_system_state(self) -> SystemState:
        """
        Get the state of the system.
        """
        return self.system_state
    
    def is_mobility(self) -> bool:
        """
        Check if the system is in mobility mode.
        """
        return self.mobility

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
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(61, 117,157)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(61, 117, 157)}{message}{sty.rs.all}")
                
            case LogLevel.INFO: # 255, 255, 255
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(255, 255, 255)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(255, 255, 255)}{message}{sty.rs.all}")
                
            case LogLevel.WARN: # 226, 174, 47
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(226, 174, 47)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(226, 174, 47)}{message}{sty.rs.all}")
                
            case LogLevel.ERROR: # 195, 59, 91
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(195, 59, 91)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(195, 59, 91)}{message}{sty.rs.all}")
                
            case LogLevel.FATAL: # 207, 62, 97
                print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.bg(207, 62, 97)}{level_str}{sty.bg.rs} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.bg(207, 62, 97)}{message}{sty.rs.all}")