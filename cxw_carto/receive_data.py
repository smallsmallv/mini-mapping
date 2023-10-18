#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

global imu_queue
global scan_queue
global odom_queue
scan_queue = []
imu_queue = []
odom_queue = []

class Receive_data_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.suber_laser = self.create_subscription(LaserScan,'/scan',self.Callback_laser,rclpy.qos.qos_profile_sensor_data)
        self.suber_imu = self.create_subscription(Imu,'/imu',self.Callback_Imu,rclpy.qos.qos_profile_sensor_data)
        self.subber_odom = self.create_subscription(Odometry,'/odom',self.Callback_Odom,rclpy.qos.qos_profile_sensor_data)
    
    def Callback_laser(self,msg):
        global scan_queue
        if len(scan_queue)<6:
            scan_queue.append(msg)
        self.get_logger().info('Receving laser msg "%s"'% len(msg.ranges))

    def Callback_Imu(self,msg):
        #self.get_logger().info('receive IMU data "%s"'% msg.angular_velocity.x)
        pass

    def Callback_Odom(self,msg):
        #self.get_logger().info('receive odom data "%s"'% msg.pose.pose.position.x)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Receive_data_node('receive_data')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()