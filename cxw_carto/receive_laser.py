#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from cxw_carto.grid import Grid
from cxw_carto.vector import Vector


class Laser_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.suber = self.create_subscription(LaserScan,'/scan',self.Callback_laser,rclpy.qos.qos_profile_sensor_data)
    
    def Callback_laser(self,msg):             
        PointNumber = len(msg.ranges)
        angle_increment = msg.angle_increment
        time_increment = msg.time_increment
        data = [0]*PointNumber
        dAngle = -angle_increment
        for i in range(PointNumber):
            dAngle += angle_increment
            x = msg.ranges[i]*np.cos(dAngle-np.pi)
            y = msg.ranges[i]*np.sin(dAngle-np.pi)
            
        self.get_logger().info('Receving laser msg "%s"'% msg.ranges[300])
        
        

def main(args=None):
    rclpy.init(args=args)
    node = Laser_node('receive_laser')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
