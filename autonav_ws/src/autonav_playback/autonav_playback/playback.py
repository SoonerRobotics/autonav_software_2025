import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from autonav_msgs.msg import DeviceState, SystemState


from datetime import datetime
import os

class playback(Node):
    
    def __init__(self):
        super().__init__('autonav_playback')
        
        self.QOS = 10
        
        self.LOGPATH = "~/Documents/AutoNav/Logs"
        self.home_dir = os.path.expanduser("~")
        self.system_state = None
        
        # Topic Listeners
        self.systemStateSub = self.create_subscription(SystemState, 'autonav/shared/system', self.systemStateCallback, self.QOS)
        self.deviceStateSub = self.create_subscription(DeviceState, 'autonav/shared/device', self.deviceStateCallback, self.QOS)
        
        # Silly Goofy Code
        self.__topicList = []
        self.__typeList = []
        self.topicDict = {}
        print(len(self.topicDict))
        
        self.topicList_sub = self.create_subscription(String, 'topicList', self.topicListenerCallback, self.QOS)
    
    # Silly Goofy Method
    def topicListenerCallback(self, msg):
        
        msgdata = []
        
        # Parse Topic List
        for s in msg.data.split(' '):
            msgdata.append(s)
        
        # Topics
        for s in msgdata[0].split('/'):
            if s in self.__topicList or s == '':
                continue
            self.__topicList.append(s)
        
        # Types
        for s in msgdata[1].split('/'):
            if s in self.__typeList or s == '':
                continue
            self.__typeList.append(s)
        
        if not len(self.__topicList) == len(self.__typeList):
            return
        
        for i in range(len(self.__topicList)):
            self.topicDict.update({self.__topicList[i]: self.__typeList[i]})
    
    # Serious Stuff below ->
    def systemStateCallback(self, msg):
        print(msg.data)
    
    def deviceStateCallback(self, msg):
        print(msg.data)
    
    
    def makeTimestamp(self) -> str:
        time = datetime.now()
        frmt = time.strftime("%Y-%m-%d_%H-%M-%S")
        return frmt
    
    def checkSystemState(self):
        pass
    
        
def main(args=None):
    rclpy.init(args=args)

    playback_sub = playback()

    rclpy.spin(playback_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    playback_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()