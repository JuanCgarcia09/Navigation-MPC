#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 08:44:47 2023

@author: k3lso
"""
import rospy
import pandas as pd
import numpy as np
from math import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

i=0
right_side = 0
left_side = 0
onetime= True

datos = list()
def main():
    rospy.init_node('save_data', anonymous=True)
    rospy.Subscriber("odom",Odometry,save_data)
    rospy.Subscriber("scan",LaserScan,data_wall)
    rospy.spin() 
def save_data(data):
    global datos, i, left_side, right_side,onetime
    #Pos1 meta Monaco
    #Pos2 Meta Vegas
    #Pos3 Meta Ex
    PosGoal=[18.500, 20.128]
    #PosGoal=[-7.831,8.903]
    #PosGoal = [-0.128,-6.595]
    poscarro = [0,0]
    poscarro[0] = data.pose.pose.position.x
    poscarro[1] = data.pose.pose.position.y
    r,y,z = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    angulocarro = z
    rsx = poscarro[0] + right_side*cos(-pi/2+angulocarro)
    rsy = poscarro[1] + right_side*sin(-pi/2+angulocarro)
    lsx = poscarro[0] + left_side*cos(pi/2+angulocarro)
    lsy = poscarro[1] + left_side*sin(pi/2+angulocarro)
    data_actual = [data.pose.pose.position.x,data.pose.pose.position.y,z,data.twist.twist.linear.x,data.twist.twist.angular.z,rsx,rsy,lsx,lsy]
    
    i=i+1
    datos.append(data_actual)
    # monaco 2.5 vegas 4.5 example 2.5
    if (sqrt((PosGoal[0]-poscarro[0])**2+(PosGoal[1]-poscarro[1])**2) <2.5) and  i > 1000 and onetime:
        df=pd.DataFrame(datos,columns = ["x","y","theta","v","phi","rsx","rsy","lsx","lsy"])
        df.to_excel('~/Documentos/Tesis2023/save_data.xlsx')
        print("Save")
        onetime = False

def data_wall(data):
    global left_side, right_side
    Grades_90 = round(90*pi/180/data.angle_increment)
    Mitad = int(round(len(data.ranges)/2))
    right_side = data.ranges[Mitad-Grades_90]
    left_side = data.ranges[Mitad+Grades_90]
        
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass