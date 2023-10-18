#!/usr/bin/env python3
# -*- coding: utf-8 -*-

''''
begin = self.get_clock().now()
end = self.get_clock().now()
delta_time = end-begin
self.get_logger().info('Used time: "%s"'% delta_time)
'''

import numpy as np
from vector import Vector
import time

def time_compare():
    begin = time.time()
    a = []
    for i in range(360):
        a.append(Vector.InitWithNums(1.,1.))
    end = time.time()
    delta_time = end-begin
    print('Used time: "%s"'% delta_time)

    begin = time.time()
    a = [0]*360
    for i in a:
        i = Vector.InitWithNums(1.,1.)
    end = time.time()
    delta_time = end-begin
    print('Used time: "%s"'% delta_time)

    begin = time.time()
    a = [Vector.InitWithNums(0.,0.)]*360
    for i in a:
        i.vec[0] = 1.
        i.vec[1] = 1.
    end = time.time()
    delta_time = end-begin
    print('Used time: "%s"'% delta_time)

    begin = time.time()
    a = [Vector.InitWithNums(0.,0.)]*360
    for i in range(len(a)):
        a[i].vec[0] = 1.
        a[i].vec[1] = 1.
    end = time.time()
    delta_time = end-begin
    print('Used time: "%s"'% delta_time)

def exp_reserche(e):
    num = 100
    time = num // (e-1)
    somme = 0
    for i in range(time):
        somme += (e-1)*(e**i)
    print(somme)
    return somme

def compare():
    begin = time.time()
    a = np.sqrt(2)
    end = time.time()
    delta_time = end-begin
    print('one sqrt used time: "%s"'% delta_time)

    begin = time.time()
    a = np.sin(3)
    b = np.cos(3)
    c = np.arctan2(3,2)
    end = time.time()
    delta_time = end-begin
    print('one sin, one cos and one atan2 used time: "%s"'% delta_time)    

if __name__ == '__main__':
    #time_compare()
    #print(float('inf')>1)
    compare()

    