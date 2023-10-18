#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
from cxw_carto.vector import Vector
from cxw_carto.grid import Grid
from threading import Thread
import time

class ScanCandidat:
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.score = 0

class Odom_Data:
    def __init__(self,x,y,time) -> None:
        self.x = x
        self.y = y
        self.time = time

class Scan_data:
    def __init__(self,ranges,time) -> None:
        self.ranges = ranges
        self.time = time    

class ScanMatch(Node):
    def __init__(self,name):
        super().__init__(name)
        self.suber_laser = self.create_subscription(LaserScan,'/scan',self.Scan_thread,rclpy.qos.qos_profile_sensor_data)
        self.subber_odom = self.create_subscription(Odometry,'/odom',self.Odom_thread,rclpy.qos.qos_profile_sensor_data)
        
        self.pubber_map = self.create_publisher(OccupancyGrid,'/map',10)
        self.timer = self.create_timer(1, self.timer_callback)
        
        # pose posteriori
        self.x = 0.
        self.y = 0.
        self.theta = 0.

        # pose posteriori ancien
        self.old_x = 0.
        self.old_y = 0.
        self.old_t_odom = 0.

        # vitesse de sensor
        self.vx = 0.
        self.vy = 0.
        self.w = 0.

        # param de scan
        self.scan_rece_init = False
        self.angle_step = np.pi*1/180
        self.angle_min = 0.
        self.angle_max = 0.        
        self.angle_increment = 0.
        self.point_number = 0
        self.time_increment = 0.
        self.scan_time = 0.
        self.range_max = 0.
        self.range_min = 0.

        # queue de données
        self.odom_queue =[]
        self.scan_queue=[]        

        self.grid = Grid()

    def Odom_thread(self,msg):
        t_odom_rece = Thread(target=self.Odom_rece(msg))
        t_odom_trait = Thread(target=self.Odom_trait)    
        
        t_odom_rece.start()
        t_odom_trait.start()
 
    def Odom_rece(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y     
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 
        self.odom_queue.append(Odom_Data(x, y, t))

        q = msg.pose.pose.orientation 
        self.theta = np.arctan2(2 * (q.w * q.z ), 1 - 2 * ( q.z * q.z))
    
    def Odom_trait(self):
        if len(self.odom_queue) == 0:
            self.get_logger().info("odom queue is empty")
        else:
            dx = self.odom_queue[-1].x - self.old_x 
            dy = self.odom_queue[-1].y - self.old_y
            dt = self.odom_queue[-1].time - self.old_t_odom

            self.old_x = self.odom_queue[-1].x
            self.old_y = self.odom_queue[-1].y
            self.old_t_odom = self.odom_queue[-1].time
            self.odom_queue.pop(0)
            
            self.vx = dx/dt
            self.vy = dy/dt        

            self.x += self.vx*dt
            self.y += self.vy*dt
            self.get_logger().info("odom queue is '%s" % len(self.odom_queue))

    def Scan_thread(self,msg):
        t_scan_rece = Thread(target=self.Scan_rece(msg))
        t_scan_trait = Thread(target=self.Scan_trait)

        t_scan_rece.start()
        t_scan_trait.start() 

    def Scan_rece(self,msg):
        if self.scan_rece_init == False:
            self.scan_rece_init = True
            self.point_number = len(msg.ranges)
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max        
            self.angle_increment = msg.angle_increment
            self.point_number = len(msg.ranges)
            self.time_increment = msg.time_increment
            self.scan_time = msg.scan_time
            self.range_max = msg.range_max
            self.range_min = msg.range_min       
        self.scan_queue.append(Scan_data(msg.ranges,msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9))
        self.get_logger().info('Scan queue: "%s"'% len(self.scan_queue))

    def Scan_trait(self):
        begin = self.get_clock().now()

        if len(self.scan_queue) == 0:
            self.get_logger().info("scan queue is empty")
        else:    
            ranges = self.scan_queue[0].ranges
            time = self.scan_queue[0].time
            self.scan_queue.pop(0)

            theta = self.theta

            data = [Vector.InitWithNums(0.,0.)]*self.point_number
            dAngle = -self.angle_increment + theta  
            for i in range(self.point_number):
                dAngle += self.angle_increment
                if ranges[i] == float('inf'):
                    data[i].vec[0] = self.range_max*np.cos(dAngle + self.angle_min) + self.x
                    data[i].vec[1] = self.range_max*np.sin(dAngle + self.angle_min) + self.y
                    self.grid.ImportDataInf(data[i],Vector.InitWithNums(self.x,self.y))
                elif ranges[i] == 0.:
                    pass              
                else:
                    data[i].vec[0] = ranges[i]*np.cos(dAngle + self.angle_min) + self.x
                    data[i].vec[1] = ranges[i]*np.sin(dAngle + self.angle_min) + self.y
                    self.grid.ImportData(data[i],Vector.InitWithNums(self.x, self.y))

        end = self.get_clock().now()
        delta_time = end-begin
        self.get_logger().info('Used time: "%s"'% delta_time)

    def Scan_Match(self):
        pass

    def timer_callback(self):
        self.get_logger().info('creating map msg')
        msg = OccupancyGrid()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.height = self.grid.get_Y_cells()
        msg.info.width = self.grid.get_X_cells()
        msg.info.origin.position.x = -self.grid.get_Maxx()
        msg.info.origin.position.y = -self.grid.get_Maxy()
        msg.info.resolution = self.grid.get_Resolution()

        msg.data = [0]*msg.info.height*msg.info.width
        for i in range(len(self.grid.Value)):    
            msg.data[i] = Gridvalue2Map(self.grid.Value[i])
        
        self.pubber_map.publish(msg)
        self.get_logger().info('map published')

def Normalization_rad(angle):
    while angle <= -np.pi:
        angle += 2*np.pi
    while angle > np.pi:
        angle -= 2*np.pi
    return angle

def Gridvalue2Map(value):
    if value == -1:
        return -1
    return int(100*(value-1)/32766)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatch('ScanMatch')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
