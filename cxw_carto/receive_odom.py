#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class OdomNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.subber = self.create_subscription(Odometry,'/odom',self.Callback_Odom,rclpy.qos.qos_profile_sensor_data)

    def Callback_Odom(self,msg):
        self.get_logger().info('receive odom data "%s"'% msg.pose.pose.position.x)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode('receive_odom')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()