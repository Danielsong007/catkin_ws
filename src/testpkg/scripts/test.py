#! /usr/bin/env python
import rospy
import time

a=[1,2,3]
num=len(a)
for i in range(0,num):
    print(i)
    time.sleep(0.02)
