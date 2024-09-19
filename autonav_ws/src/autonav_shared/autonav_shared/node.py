from rclpy.node import Node as RclpyNode

class Node(RclpyNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)