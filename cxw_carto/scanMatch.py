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
    

class ScanMatch(Node):
    def __init__(self,name):
        super().__init__(name)
        self.suber_laser = self.create_subscription(LaserScan,'/scan',self.Callback_laser2,rclpy.qos.qos_profile_sensor_data)
        self.subber_odom = self.create_subscription(Odometry,'/odom',self.odom_thread,rclpy.qos.qos_profile_sensor_data)
        
        self.pubber_map = self.create_publisher(OccupancyGrid,'/map',10)
        self.timer = self.create_timer(1, self.timer_callback)
        
        self.x = 0.
        self.y = 0.
        self.theta = 0.

        self.vx = 0.
        self.vy = 0.
        self.w = 0.

        self.grid = Grid()
        
        self.angle_step = np.pi*1/180

        self.odom_queue =[]
        self.scan_queue=[]        

    def Callback_Odom(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y     
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 
        
        if len(self.odom_queue) == 0:
            self.odom_queue.append(Odom_Data(x, y, t))
        else:
            dx = x - self.odom_queue[-1].x
            dy = y - self.odom_queue[-1].y
            dt = t - self.odom_queue[-1].time

            self.vx = dx/dt
            self.vy = dy/dt

            self.odom_queue.append(Odom_Data(x, y, t))

            self.x += self.vx*dt
            self.y += self.vy*dt

            q = msg.pose.pose.orientation 
            self.theta = np.arctan2(2 * (q.w * q.z ), 1 - 2 * ( q.z * q.z))
    
    def odom_trait(self):
        pass

    def Callback_laser_Match(self,msg):
        begin = self.get_clock().now()

        robot_x = self.x
        robot_y = self.y
        theta = self.theta

        Point_number = len(msg.ranges)
        angle_increment = msg.angle_increment
        time_increment = msg.time_increment

        data = [0]*Point_number
        dAngle = -angle_increment+theta        
        for i in range(Point_number):
            dAngle += angle_increment
            if msg.ranges[i] == float('inf'):
                distance = msg.range_max
                point_x = distance*np.cos(dAngle) + robot_x
                point_y = distance*np.sin(dAngle) + robot_y
                data[i] = Vector.InitWithNums(point_x,point_y)
                self.grid.ImportDataInf(data[i],Vector.InitWithNums(robot_x,robot_y))
            else:
                point_x = msg.ranges[i]*np.cos(dAngle) + robot_x
                point_y = msg.ranges[i]*np.sin(dAngle) + robot_y
                data[i] = Vector.InitWithNums(point_x,point_y)
                self.grid.ImportData(data[i],Vector.InitWithNums(robot_x,robot_y))
        #
        best_score = 0
        best_candidat = ScanCandidat(0,0,0)
        candidats = []        
        for dy in np.arange(-0.05,0.1,0.05):
            for dx in np.arange(-0.05,0.1,0.05):
                for dtheta in np.arange(-self.angle_step*3,self.angle_step*4,self.angle_step):
                    candidats.append(ScanCandidat(self.x+dx, self.y+dy, self.theta+dtheta))
        for candidat in candidats:
            robot_x = candidat.x
            robot_y = candidat.y
            theta = candidat.theta

            Point_number = len(msg.ranges)
            angle_increment = msg.angle_increment
            time_increment = msg.time_increment

            data = [0]*Point_number
            dAngle = -angle_increment+theta   

            for i in range(Point_number):
                dAngle += angle_increment
                if msg.ranges[i] == float('inf'):
                    pass
                else:
                    point_x = msg.ranges[i]*np.cos(dAngle) + robot_x
                    point_y = msg.ranges[i]*np.sin(dAngle) + robot_y
                    data[i] = Vector.InitWithNums(point_x,point_y)
                    candidat.score += self.grid.CheckRepeated(data[i])
                        
            if candidat.score > best_score:
                best_score = candidat.score
                best_candidat = candidat
            
        self.x = best_candidat.x
        self.y = best_candidat.y
        self.theta = best_candidat.theta
        #

        end = self.get_clock().now()
        delta_time = (end-begin).nanoseconds*1e-9
        self.get_logger().info('Used time: "%s"s'% delta_time)
              

    def Callback_laser(self,msg):
        begin = self.get_clock().now()

        robot_x = self.x
        robot_y = self.y
        theta = self.theta

        Point_number = len(msg.ranges)
        angle_increment = msg.angle_increment
        time_increment = msg.time_increment

        data = [0]*Point_number
        dAngle = -angle_increment+theta        
        for i in range(Point_number):
            dAngle += angle_increment
            if msg.ranges[i] == float('inf'):
                distance = msg.range_max
                point_x = distance*np.cos(dAngle-np.pi) + robot_x
                point_y = distance*np.sin(dAngle-np.pi) + robot_y
                data[i] = Vector.InitWithNums(point_x,point_y)
                self.grid.ImportDataInf(data[i],Vector.InitWithNums(robot_x,robot_y))
            else:
                point_x = msg.ranges[i]*np.cos(dAngle-np.pi) + robot_x
                point_y = msg.ranges[i]*np.sin(dAngle-np.pi) + robot_y
                data[i] = Vector.InitWithNums(point_x,point_y)
                self.grid.ImportData(data[i],Vector.InitWithNums(robot_x,robot_y))
        
        end = self.get_clock().now()
        delta_time = end-begin
        self.get_logger().info('Used time: "%s"'% delta_time)
    
    def Callback_laser2(self,msg):
        begin = self.get_clock().now()

        robot_x = self.x
        robot_y = self.y
        theta = self.theta

        Point_number = len(msg.ranges)
        angle_increment = msg.angle_increment
        time_increment = msg.time_increment

        data = [0]*Point_number
        dAngle = -angle_increment+theta        
        for i in range(Point_number):
            dAngle += angle_increment
            if msg.ranges[i] == float('inf'):
                distance = msg.range_max
                point_x = distance*np.cos(dAngle) + robot_x
                point_y = distance*np.sin(dAngle) + robot_y
                data[i] = Vector.InitWithNums(point_x,point_y)
                self.grid.ImportDataInf(data[i],Vector.InitWithNums(robot_x,robot_y))
            else:
                point_x = msg.ranges[i]*np.cos(dAngle) + robot_x
                point_y = msg.ranges[i]*np.sin(dAngle) + robot_y
                data[i] = Vector.InitWithNums(point_x,point_y)
                self.grid.ImportData(data[i],Vector.InitWithNums(robot_x,robot_y))   

        end = self.get_clock().now()
        delta_time = end-begin
        self.get_logger().info('Used time: "%s"'% delta_time)
            

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

    def odom_thread(self,msg):
        t_odom_rece = Thread(target=self.Callback_Odom(msg))
        t_odom_trait = Thread(target=self.odom_trait)    
        
        t_odom_rece.start()
        t_odom_trait.start()

    
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
 
