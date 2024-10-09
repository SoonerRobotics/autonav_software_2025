import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os

class playback(Node):
    
    def __init__(self):
        super().__init__('autonav_playback')
        
        
        self.LOGPATH = ""
        self.home_dir = os.path.expanduser("~")
        
        self.__topicList = []
        self.__typeList = []
        self.topicDict = {}
        print(len(self.topicDict))
        
        self.topicList_sub = self.create_subscription(String, 'topicList', self.topicListenerCallback, 10)
        
        
        
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