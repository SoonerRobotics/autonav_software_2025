from rclpy.node import Node as RclpyNode
from autonav_shared.types import LogLevel
import sty
import time
import inspect


class Node(RclpyNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        
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