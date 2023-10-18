#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class Probability_Value:
    def __init__(self):
        self.tableZ0 = [0]*32768#101#32768
        self.tableZ1 = [0]*32768#101#32768
        self.__oddZ0 = self.__Probability2Odd(0.49)
        self.__oddZ1 = self.__Probability2Odd(0.55)
        self.__ComputeTableZ0()
        self.__ComputeTableZ1()


    def __Probability2Odd(self,p):
        return p / (1-p)
    
    def __Odd2Probability(self,odd):
        return odd / (odd + 1)
    
    def __Value2Probability(self,value):
        return ((value - 1) / 32766.) * 0.8 + 0.1
        #return((value - 1) / 99.) * 0.8 + 0.1

    def __Probability2Value(self,p):
        if p<0.1:
            p=0.1
        if p>0.9:
            p=0.9
        return int(((p - 0.1) /0.8) * 32766)+1
        #return int(((p - 0.1) /0.8) * 99)+1

    def __OddUpdateZ0(self,odd):
        return odd * self.__oddZ0
    
    def __OddUpdateZ1(self,odd):
        return self.__oddZ1*odd


    def __ComputeTableZ0(self):
        for i in range(1,32768):
        #for i in range(1,101):
            self.tableZ0[i] = self.__Probability2Value(self.__Odd2Probability(self.__OddUpdateZ0(self.__Probability2Odd(self.__Value2Probability(i)))))

    def __ComputeTableZ1(self):
        #for i in range(1,101):
        for i in range(1,32768):
            self.tableZ1[i] = self.__Probability2Value(self.__Odd2Probability(self.__OddUpdateZ1(self.__Probability2Odd(self.__Value2Probability(i)))))

    def LookUpMissTable(self,value):
        if value == -1:
            return 16384
        return self.tableZ0[value]

    def LookUpHitTable(self,value):
        if value == -1:
            return 16384
        return self.tableZ1[value]

if __name__ == '__main__':
    a = Probability_Value()
    print(a.LookUpHitTable(100))