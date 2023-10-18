#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from cxw_carto.vector import Vector

class PoseExtrapolator(Node):
    def __init__(self,name):
        super().__init__(name)
        self.subber_imu = self.create_subscription(Imu,'/imu',self.Callback_imu,rclpy.qos.qos_profile_sensor_data)
        self.subber_odom = self.create_subscription(Odometry,'/odom',self.Callback_odom,rclpy.qos.qos_profile_sensor_data)

        self.stamp = self.get_clock().now()
        self.x = 0.
        self.y = 0.
        self.theta = 0.
        self.vx = 0.
        self.vy = 0.
        self.w = 0.

        self.old_x = 0.
        self.old_y = 0.
        
    
    def Callback_imu(self,msg):
        self.stamp = msg.header.stamp
        self.w = msg.angular_velocity.z

    def Callback_odom(self,msg):
        self.stamp = msg.header.stamp        
        dx = msg.pose.pose.position.x - msg.pose.pose.position.x
        #self.vx = 
        self.old_x = self.x
        self.old_y = self.y


def main(args=None):
    rclpy.init(args=args)
    node = PoseExtrapolator('PoseExtrapolator')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()