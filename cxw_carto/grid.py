#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from cxw_carto.probability_value import Probability_Value
from cxw_carto.vector import Vector
import time
import numpy as np

class Grid:
    def __init__(self):
        self.__Resolution = 0.05
        self.__X_cells = 4
        self.__Y_cells = 4
        self.__OldX_cells = 4
        self.__OldY_cells = 4
        self.__Maxx = self.__Resolution*self.__X_cells/2
        self.__Maxy = self.__Resolution*self.__Y_cells/2
        self.Value = [16384]*(self.__X_cells * self.__Y_cells)
        #self.Value = [50]*(self.__X_cells * self.__Y_cells)
        self.__offset = 0
        self.ValueTable = Probability_Value()

    def get_X_cells(self):
        return self.__X_cells
    
    def get_Y_cells(self):
        return self.__Y_cells
    
    def get_Maxx(self):
        return self.__Maxx
    
    def get_Maxy(self):
        return self.__Maxy
    
    def get_Resolution(self):
        return self.__Resolution
    
    def __str__(self):
        #print("value:",self.__Value)
        print("xcell:",self.__X_cells)
        result = "["
        result += "Resolution = " + str(self.__Resolution) + " maxP = " + str(self.__Maxx) + "; MapSize = " + "{:.2f}".format(self.__Maxx * self.__Maxy * 4) + "m2; Cells =" + str(self.__X_cells) + "*" + str(self.__Y_cells) + "; Value = \n"
        count = 0
        for item in self.Value:
            result += str(item) + "\t"
            count+=1
            if count % self.__X_cells == 0:
                result += "\n"
        result += "]"
        return result
    
    def __repr__(self):
        result = "["
        result += "Resolution = " + self.__Resolution + " maxP = " + self.__Maxx + "; MapSize = " + (self.__Maxx * self.__Maxy * 4).ToString("f2") + "m2; Cells =" + (self.__X_cells) + "*" + (self.__Y_cells) + "; Value = \n"
        count = 0
        for item in self.Value:
            esult += item + "\t"
            count+=1
            if count % self.__X_cells == 0:
                result += "\n"
        result += "]"
        return result

    def IsContained(self,point):
        return abs(point.vec[0]) <= self.__Maxx and abs(point.vec[1]) <= self.__Maxy

    def GrowLimits(self,point):
        while not self.IsContained(point):
            #OldValue = [0]*len(self.__Value)
            print("growing map")
            OldValue = self.Value.copy()
            self.__OldX_cells = self.__X_cells
            self.__OldY_cells = self.__Y_cells

            self.__Maxx *= 2
            self.__Maxy *= 2
            self.__X_cells *= 2
            self.__Y_cells *= 2

            for i in range(3*len(self.Value)):
                self.Value.append(-1)
                #self.Value.append(50)

            self.__offset = int(self.__OldX_cells/2) + int(self.__X_cells*(self.__OldY_cells/2))
            for i in range(len(OldValue)):
                self.Value[i] = -1
                #self.Value[i] = 50
            for i in range(self.__OldY_cells):
                for j in range(self.__OldX_cells):
                    self.Value[self.__offset+self.__X_cells*i+j] = OldValue[self.__OldX_cells*i+j] 

    def Point2Pixel(self,point):
        dx = point.vec[0] + self.__Maxx
        dy = self.__Maxy - point.vec[1]
        x = int(dx/self.__Resolution)+1
        y = int(dy/self.__Resolution)+1
        if x>self.__X_cells : x-=1
        if y>self.__Y_cells : y-=1
        return Vector.InitWithNums(x,y)

    def Pixel2Cell(self,point):
        return int(point.vec[1]-1)*self.__X_cells + int(point.vec[0]) - 1

    def Pixel2Normal(self,cordonate):
        return Vector.InitWithNums(cordonate.vec[0], (self.__Y_cells+1) - cordonate.vec[1])
    
    def Normal2Pixel(self,cordonate):
        return Vector.InitWithNums(cordonate.vec[0], (self.__Y_cells+1)-cordonate.vec[1])
        
    def Vector2NormalX(self,point):
        return int(self.Pixel2Normal(self.Point2Pixel(point)).vec[0])

    def Vector2NormalY(self,point):
        return int(self.Pixel2Normal(self.Point2Pixel(point)).vec[1])

    def Bresenham(self,point,robot):
        x1 = self.Vector2NormalX(robot)
        y1 = self.Vector2NormalY(robot)
        x2 = self.Vector2NormalX(point)
        y2 = self.Vector2NormalY(point)

        result = []
        #horizon
        if y1==y2:
            for x in range(min(x1,x2),max(x1,x2)):
                result.append(Vector.InitWithNums(x,y1))
        
        #vertical
        elif x1 ==x2:
            for y in range(min(y1,y2),max(y1,y2)):
                result.append(Vector.InitWithNums(x1,y))
        
        else:
            k = (y2 - y1) / (x2 - x1)
            # 1l
            if y2 > y1 and x2 > x1 and k <= 1:
                delta = (y1 + k) - (y1 + 0.5)
                y = y1
                for x in range(x1,x2):
                    result.append(Vector.InitWithNums(x,y))
                    if delta >= 0:
                        y += 1
                        delta += (k-1)
                    else:
                        delta +=k

            elif y2 > y1 and x2 > x1 and k >= 1 :
                k = (x2 - x1) / (y2 - y1)
                delta = (x1 + k) - (x1 + 0.5)
                x = x1
                for y in range(y1,y2):
                    result.append(Vector.InitWithNums(x,y))
                    if delta >= 0:
                        x+=1
                        delta +=(k-1)
                    else:
                        delta += k

            elif y2 < y1 and x2 > x1 and k >= -1:
                delta = (y1 + k) - (y1 - 0.5)
                y = y1
                for x in range(x1,x2):
                    result.append(Vector.InitWithNums(x,y))
                    if delta <= 0:
                        y -= 1
                        delta += (k+1)
                    else:
                        delta += k
                    
            elif y2 < y1 and x2 > x1 and k <= -1:
                k = (x2 - x1) / (y2 - y1)
                delta = (x1 - k) - (x1 + 0.5)
                x = x1
                for y in range(y1,y2,-1):
                    result.append(Vector.InitWithNums(x,y))
                    if delta >= 0:
                        x+=1
                        delta -= k+1
                    else:
                        delta -=k
            
            elif y2 > y1 and x2 < x1 and k >= -1:
                delta = (y1 - k) - (y1 + 0.5)
                y = y1
                for x in range(x1,x2,-1):
                    result.append(Vector.InitWithNums(x,y))
                    if delta >=0:
                        y+=1
                        delta -= k+1
                    else:
                        delta-=k

            elif y2 > y1 and x2 < x1 and k <= -1:
                k = (x2 - x1) / (y2 - y1)
                delta = (x1 + k) - (x1 - 0.5)
                x = x1
                for y in range(y1,y2):
                    result.append(Vector.InitWithNums(x,y))
                    if delta <= 0:
                        x-=1
                        delta += k+1
                    else:
                        delta +=k

            elif y2 < y1 and x2 < x1 and k <= 1:
                delta = (y1 - k) - (y1 - 0.5)
                y = y1
                for x in range(x1,x2,-1):
                    result.append(Vector.InitWithNums(x,y))
                    if delta <= 0 :
                        y-=1
                        delta += (-k+1)
                    else:
                        delta -=k

            elif y2 < y1 and x2 < x1 and k >= 1:
                k = (x2 - x1) / (y2 - y1)
                delta = (x1 - k) - (x1 - 0.5)
                x = x1
                for y in range(y1,y2,-1):
                    result.append(Vector.InitWithNums(x,y))
                    if delta <= 0:
                        x-=1
                        delta += (-k+1)
                    else:
                        delta += -k
            
        res = [0]*len(result)
        for i in range(0,len(result)):
            result[i] = self.Normal2Pixel(result[i])
            res[i]=self.Pixel2Cell(result[i])

        return res

    def ImportData(self,point,car):
        self.GrowLimits(point)
        missPoints = self.Bresenham(point,car)
        hitPoints = self.Pixel2Cell(self.Point2Pixel(point))
        for i in range(len(missPoints)):
            self.Value[missPoints[i]] = self.ValueTable.LookUpMissTable(self.Value[missPoints[i]])
        self.Value[hitPoints] = self.ValueTable.LookUpHitTable(self.Value[hitPoints])

    def ImportDataInf(self,point,car):
        self.GrowLimits(point)
        missPoints = self.Bresenham(point,car)
        hitPoints = self.Pixel2Cell(self.Point2Pixel(point))
        for i in range(len(missPoints)):
            self.Value[missPoints[i]] = self.ValueTable.LookUpMissTable(self.Value[missPoints[i]])

    def CheckRepeated(self,point):
        return self.Value[self.Pixel2Cell(self.Point2Pixel(point))]

def AddCercle(center,radius):
    Environment = []
    Resolution = 0.05
    flag = False
    x = radius + center.vec[0]
    y= center.vec[1]
    xRounded = round(x,3)
    yRounded = round(y,3)
    dAngle = 0
    long = 2*np.pi*radius
    loopTime = 360#int(long/Resolution)
    for i in range(loopTime):
        point = Vector.InitWithNums(xRounded,yRounded)
        for item in Environment:
            if item == point:
                flag = True
            else: flag = False
        if flag == False:
            Environment.append(point)
        dAngle += 2*np.pi * Resolution
        x = radius * np.cos(dAngle) + center.vec[0]
        y = radius*np.sin(dAngle) + center.vec[1]
        xRounded = round(x,3)
        yRounded = round(y,3)
    return Environment

if __name__=='__main__':
    map = Grid()
    points = AddCercle(Vector.InitWithNums(0,0),0.3)
    robot = Vector.InitWithNums(0,0)
    for i in range(20):
        begin = time.time()
        for point in points:       
            map.ImportData(point,robot)     
        end = time.time()
        delta_time = end -begin
        print(map)
        print("used time:",delta_time, "points number:" , len(points))
        time.sleep(1)
