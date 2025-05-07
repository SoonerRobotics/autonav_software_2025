import rclpy
from rclpy.node import Node as RclpyNode
from autonav_shared.types import DeviceState, LogLevel, SystemState
from autonav_msgs.msg import SystemState as SystemStateMsg, DeviceState as DeviceStateMsg, Performance, Log, ConfigurationBroadcast, ConfigurationUpdate
from rclpy.executors import MultiThreadedExecutor
import sty
import time
import inspect
import json


class Node(RclpyNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        # Setup our device state
        self.system_state = SystemState.DISABLED
        self.mobility = False
        self.device_states = {}
        self.start_times = {}
        self.config = {}
        self.other_cfgs = {}
        self.device_states[name] = DeviceState.OFF
        
        # TODO: Setup all relevant publishers, subscribers, services, clients, etc
        self.system_state_sub = self.create_subscription(SystemStateMsg, "/autonav/shared/system", self.system_state_callback, 10)
        self.device_state_sub = self.create_subscription(DeviceStateMsg, "/autonav/shared/device", self.device_state_callback, 10)
        self.system_state_pub = self.create_publisher(SystemStateMsg, "/autonav/shared/system", 10)
        self.device_state_pub = self.create_publisher(DeviceStateMsg, "/autonav/shared/device", 10)
        self.config_sub = self.create_subscription(ConfigurationUpdate, "/autonav/shared/config/updates", self.config_callback, 10)
        self.config_pub = self.create_publisher(ConfigurationUpdate, "/autonav/shared/config/updates", 10)
        self.config_broadcast_sub = self.create_subscription(ConfigurationBroadcast, "/autonav/shared/config/requests", self.config_broadcast_callback, 10)
        self.config_broadcast_pub = self.create_publisher(ConfigurationBroadcast, "/autonav/shared/config/requests", 10)

        self.performance_pub = self.create_publisher(Performance, "/autonav/shared/performance", 10)
        self.log_pub = self.create_publisher(Log, "/autonav/shared/log", 10)

    def init(self) -> None:
        """
        Called when the node synchronizes with the system.
        """
        pass

    def _smart_dump_config(self, config) -> str:
        if isinstance(config, dict):
            return json.dumps(config)
        else:
            try:
                return json.dumps(config.__dict__)
            except:
                return str(config)

    def config_callback(self, msg: ConfigurationUpdate) -> None:
        """
        Callback for the configuration update topic.
        """
        if msg.device == self.get_name():
            # self.log(f"Received update on our own configuration", LogLevel.DEBUG)
            old_cfg = self.config
            new_cfg = json.loads(msg.json)
            self.apply_config(new_cfg)
            self.on_config_update(old_cfg, self.config)
        else:
            # self.log(f"Received updated on {msg.device}'s configuration", LogLevel.DEBUG)
            pass
        
        self.other_cfgs[msg.device] = json.loads(msg.json)

    def apply_config(self, config) -> None:
        """
        Apply a configuration to the node.
        Override this to use type hints for the config.
        """
        self.config = config

    def on_config_update(self, old_cfg, new_cfg) -> None:
        """
        Called when the configuration is updated.
        """
        pass

    def config_broadcast_callback(self, msg: ConfigurationBroadcast) -> None:
        """
        Callback for the configuration broadcast topic.
        """
        if msg.target_nodes == [] or self.get_name() in msg.target_nodes:
            self.broadcast_config()

    def _broadcast_config(self, name, config) -> None:
        """
        Broadcast our configuration to the system.
        """
        msg = ConfigurationUpdate()
        msg.device = name
        msg.json = self._smart_dump_config(config)
        self.config_pub.publish(msg)

    def broadcast_config(self) -> None:
        """
        Broadcast our configuration to the system.
        """
        self._broadcast_config(self.get_name(), self.config)

    def request_all_configs(self) -> None:
        """
        Request the configuration of all devices.
        """
        self.request_config(None)

    def request_config(self, device: str) -> None:
        """
        Request the configuration of a specific device.
        """
        msg = ConfigurationBroadcast()
        if device != None:
            msg.target_nodes = [device]
        else:
            msg.target_nodes = []
        self.config_broadcast_pub.publish(msg)

    def write_config(self, config) -> None:
        """
        Register a configuration object.
        """
        self.config = config
        self.broadcast_config()

    def perf_start(self, name: str) -> None:
        """
        Start a performance measurement.
        """
        self.start_times[name] = time.time_ns() // 1_000_000

    def perf_stop(self, name: str, print_to_console: bool = False) -> None:
        """
        End a performance measurement.
        """
        if name not in self.start_times:
            self.log(f"Performance tiemr {name} not found", LogLevel.ERROR)
            return
        
        duration = (time.time_ns() // 1_000_000) - self.start_times[name]
        self.start_times.pop(name)
        
        msg = Performance()
        msg.name = name
        msg.elapsed = duration
        self.performance_pub.publish(msg)
        
        if print_to_console:
            self.log(f"Performance timer {name} took {duration}ms", LogLevel.DEBUG)

    def system_state_callback(self, msg: SystemStateMsg) -> None:
        """
        Callback for the system state topic.
        """
        old_state = self.system_state
        old_mobility = self.mobility

        self.system_state = SystemState(msg.state)
        self.mobility = msg.mobility

        if old_state != self.system_state:
            self.on_system_state_updated(old_state, self.system_state)

        if old_mobility != self.mobility:
            self.on_mobility_updated(old_mobility, self.mobility)

    def on_system_state_updated(self, old: SystemState, new: SystemState) -> None:
        """
        Called when the system state is updated.
        """
        pass

    def on_mobility_updated(self, old: bool, new: bool) -> None:
        """
        Called when the mobility state is updated.
        """
        pass
        
    def device_state_callback(self, msg: DeviceStateMsg) -> None:
        """
        Callback for the device state topic.
        """
        old_state = self.device_states[msg.device] if msg.device in self.device_states else None
        self.device_states[msg.device] = DeviceState(msg.state)
        if (old_state == None or old_state == DeviceState.OFF) and DeviceState(msg.state) == DeviceState.WARMING and msg.device == self.get_name():
            self.init()

    def set_device_state(self, state: DeviceState) -> None:
        """
        Set the state of the current node.
        """
        self._set_device_state(self.get_name(), state)

    def _set_device_state(self, device: str, state: DeviceState) -> None:
        """
        Set the state of a specific device.
        """
        msg = DeviceStateMsg()
        msg.device = device
        msg.state = state
        self.device_state_pub.publish(msg)

        
    def set_system_state(self, state: SystemState) -> None:
        """
        Set the state of the system.
        """
        msg = SystemStateMsg()
        msg.state = state
        msg.mobility = self.mobility
        self.system_state_pub.publish(msg)

    def set_mobility(self, mobility: bool) -> None:
        """
        Set the mobility of the system.
        """
        self.mobility = mobility
        self.set_system_state(self.system_state)

    def get_device_state(self, device: str = None) -> DeviceState:
        """
        Get the state of a specific device.

        If no device is provided, the state of the current node is returned.
        """
        if device is None:
            device = self.get_name()

        if device not in self.device_states:
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

        # Log to topic
        msg = Log()
        msg.timestamp = time.time_ns() // 1_000_000
        msg.level = level.value
        msg.node = self.get_name()
        msg.function_caller = calling_function
        msg.line_number = line_number
        msg.message = message
        self.log_pub.publish(msg)

        r, g, b = 0, 0, 0
        match level:
            case LogLevel.DEBUG:
                r, g, b = 61, 117, 157
            case LogLevel.INFO:
                r, g, b = 255, 255, 255
            case LogLevel.WARN:
                r, g, b = 226, 174, 47
            case LogLevel.ERROR:
                r, g, b = 195, 59, 91

        # FATAL requires backgroud colors so it gets its own if statement
        if level == LogLevel.FATAL:
            print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.bg(207, 62, 97)}{level_str}{sty.bg.rs} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.bg(207, 62, 97)}{message}{sty.rs.all}")
            return
        
        print(f"{sty.fg(99, 150, 79)}{current_time} {sty.fg.white}| {sty.fg(r, g, b)}{level_str} {sty.fg.white}| {sty.fg(90, 60, 146)}{self.get_name()}{sty.fg.white}:{sty.fg(90, 60, 146)}{calling_function}{sty.fg.white}:{sty.fg(90, 60, 146)}{line_number} {sty.fg.white}- {sty.fg(r, g, b)}{message}{sty.rs.all}")

# Run Nodes currently don't exist here or anywhere else.. Pasting frm lst yrs code
    def run_node(node):
        """
        Runs the node with the correct ROS parameters and specifications

        :param node: The node to run.
        """

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        executor.remove_node(node)

    def run_nodes(nodes):
        """
        Runs the nodes with the correct ROS parameters and specifications

        :param nodes: The nodes to run.
        """

        executor = MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        executor.spin()
        for node in nodes:
            executor.remove_node(node)