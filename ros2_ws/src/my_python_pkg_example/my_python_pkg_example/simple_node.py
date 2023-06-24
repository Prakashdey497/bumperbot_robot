#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class My_Node(Node):
    def __init__(self):
        super().__init__("simple_node")
        self.counter_ = 0
        self.create_timer(0.5,self.timer_callback)
        
        
    def timer_callback(self):
        self.get_logger().info("helo world " + str(self.counter_))
        self.counter_ +=1
        
        
def main(args = None):
    rclpy.init(args=args)
    my_node = My_Node()
    rclpy.spin(node=my_node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()