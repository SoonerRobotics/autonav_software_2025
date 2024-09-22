from enum import IntEnum


class SystemState(IntEnum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2
    SHUTDOWN = 3
    
class DeviceState(IntEnum):
    OFF = 0
    WARMING = 1
    READY = 2
    OPERATING = 3
    ERROR = 4
    
class LogLevel(IntEnum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4
    