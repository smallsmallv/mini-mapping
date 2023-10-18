#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.subber = self.create_subscription(Imu,'/imu',self.Callback_Imu,rclpy.qos.qos_profile_sensor_data)

    def Callback_Imu(self,msg):
        self.get_logger().info('receive IMU data "%s"'% msg.angular_velocity.x)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode('receive_imu')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()