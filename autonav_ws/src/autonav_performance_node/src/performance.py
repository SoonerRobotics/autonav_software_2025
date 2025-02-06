#!/usr/bin/env python3

import rclpy

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import HardwarePerformance

import psutil

class Config():
    
    def __init__(self):
        self.CELSIUS = False

class PerformanceNode(Node):
    
    def __init__(self):
        super().__init__("autonav_hardware_performance")
        # Initiliza config
        self.config = Config()
        
        # setup cpu and mem
        self.cpu_utilization = psutil.cpu_percent()
        self.update_cpu()
        self.memory = psutil.virtual_memory()
        
        # Publisher
        self.performance_publisher = self.create_publisher(HardwarePerformance, "/autonav/hardware_performance", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    
    def timer_callback(self):
        self.update_cpu()
        self.update_memory()
        temp = self.query_temps()
        self.log(f"{temp}, {self.cpu_utilization}, {self.memory}")
        print(f"{temp}, {self.cpu_utilization}, {self.memory}")
        #msg = HardwarePerformance()
        #msg.cpu = self.cpu_utilization
        #msg.memory = self.memory
        #self.performance_publisher.publish(msg)
        
    
    def query_temps(self):
        if self.config.CELSIUS:
            # Send in Celcius
            return psutil.sensors_temperatures()
        else:
            # Send in American
            return psutil.sensors_temperatures(True)
    
    def query_battery(self):
        battery = psutil.sensors_battery()
        if battery == None:
            return
        else:
            return battery
        
    
    def update_cpu(self, interval: float = 0.0):
        i = None
        if interval <= 0:
            i = 0.0
            self.cpu_utilization = psutil.cpu_percent(i)
            return
        self.cpu_utilization = psutil.cpu_percent(interval)

    def update_memory(self):
        self.memory = psutil.virtual_memory()

def main(args=None):
    rclpy.init(args=args)

    performance_node = PerformanceNode()

    rclpy.spin(performance_node)
    performance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()