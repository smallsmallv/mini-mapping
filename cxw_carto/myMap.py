#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time
import time


class MapPub_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.puber = self.create_publisher(OccupancyGrid,'/map',10)
        self.timer = self.create_timer(0.5,self.timer_callback)
    
    def timer_callback(self):             
        self.get_logger().info('creating map msg')
        msg = OccupancyGrid()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.height = 20
        msg.info.width = 25
        msg.info.origin.position.x = 1.0
        msg.info.origin.position.y = 5.0
        msg.info.resolution = 1.0

        msg.data = [0]*msg.info.height*msg.info.width
        for i in range(100):    
            msg.data[i] = -1
            msg.data[i+100] = 0
            msg.data[i+200] = 33
            msg.data[i+300] = 66
            msg.data[i+400] = 100
        
        self.puber.publish(msg)
        self.get_logger().info('map published')




def main(args=None):
    rclpy.init(args=args)
    node = MapPub_node('map_publish')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()