#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class Vector:
    def __init__(self,input):
        self.size = len(input)
        self.vec = input

    @classmethod
    def InitWithNums(self,*args):
        nums = []
        for i in args:
            nums.append(i)
        return self(nums)

    def __str__(self):
        result = "["
        for i in self.vec:
            result += " " + "{:.3f}".format(i) + " "
        result += "]"
        return result
    
    def __repr__(self):
        result = "["
        for i in self.vec:
            result += " " + "{:.3f}".format(i) + " "
        result += "]"
        return result

    def __eq__(self,other):
        if self.size != other.size:
            return False
        else:
            for i in range(0,self.size):
                if self.vec[i] != other.vec[i]:
                    return False
        return True
    
    def __add__(self,other):
        result = [0]*self.size
        for i in range(0,self.size):
            result[i]=self.vec[i]+other.vec[i]
        return Vector(result)
    
    def __neg__(self):
        result = [0]*self.size
        for i in range(0,self.size):
            result[i]=-self.vec[i]
        return Vector(result)

    def __sub__(self,other):
        return self + (-other)
    
    def __mul__(self,other):
        if type(other) is Vector:
            if self.size != other.size:
                print("error: vector multi: size not match")
                exit()
            result = 0
            for i in range(0,self.size):
                result += self.vec[i]*other.vec[i]
            return result
        else:
            result = [0]*self.size
            for i in range(0,self.size):
                result[i] = self.vec[i]*other
            return Vector(result)
    
    def __rmul__(self,other):
        return self*other
   
    def test(self):
        return self
    
        
        