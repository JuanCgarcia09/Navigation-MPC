#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 17:07:28 2022

@author: juan
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 15:12:07 2022

@author: juan
"""
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import sys

from math import *



index_guar = 0
target_vdx_ant = 0
target_vdy_ant = 0
index_save = 0

pub = rospy.Publisher('obstacle',Odometry,queue_size=10)
Pathpub = rospy.Publisher('path_seguir',Path,queue_size=10)
Pathpub2 = rospy.Publisher('path_repeler',Path,queue_size=10)
Pathpub3 = rospy.Publisher('path_atraer',Path,queue_size=10)
angulocarro = 0
poscarro = [0,0]

def main():
    rospy.init_node('obstacle', anonymous=True)
    rospy.Subscriber("scan",LaserScan,path)
    rospy.Subscriber("odom",Odometry,odom)

    rospy.spin()
    
def path(data):
    global angulocarro, poscarro, index_guar,target_vdx_ant,target_vdy_ant,index_save
    Obstacle_msg = Odometry()
    Flag_Obstacle = 0
    Obstacle_msg.pose.pose.position.x = 0
    Obstacle_msg.pose.pose.position.y = 0
    Obstacle_msg.pose.pose.position.z = Flag_Obstacle
    
    Grades_90 = round(90*pi/180/data.angle_increment)
    Mitad = int(round(len(data.ranges)/2))
    #135 = 45 gradps
    #15 = angle_increment*x=5*pi/180
    poses = []
    poses2 = []
    poses3 = []
    magnitudes = list(data.ranges[Mitad-Grades_90:Mitad+Grades_90])
    #Step 1 find the goal (The goal is the middle point in the gap)
    i_guar = list()
    gap_guar = list()
    for i in range(0,len(magnitudes)-1):
        if abs(magnitudes[i]-magnitudes[i+1])>0.3:
            gap_guar.append(abs(magnitudes[i]-magnitudes[i+1]))
            i_guar.append(i)
            # Test print the gaps
            #print(str(magnitudes[i])+" hola "+str(magnitudes[i+1]))
            """
            pose = PoseStamped()
            pose.pose.position.x = poscarro[0] + magnitudes[i]*cos(i*data.angle_increment-pi/2+angulocarro)
            pose.pose.position.y = poscarro[1] + magnitudes[i]*sin(i*data.angle_increment-pi/2+angulocarro)
            poses.append(pose)
            pose.pose.position.x = poscarro[0] + magnitudes[i+1]*cos((i+1)*data.angle_increment-pi/2+angulocarro)
            pose.pose.position.y = poscarro[1] + magnitudes[i+1]*sin((i+1)*data.angle_increment-pi/2+angulocarro)
            poses.append(pose)
            """
    if(bool(gap_guar)):
        indice_max = gap_guar.index(max(gap_guar))
        i_max = i_guar[indice_max]
    else:
        indice_max = magnitudes.index(max(magnitudes))-1
        i_max = magnitudes.index(max(magnitudes))-1
    #print(str(magnitudes[i_max])+" hola "+str(magnitudes[i_max+1]))
    """
    pose = PoseStamped()
    pose.pose.position.x = poscarro[0]
    pose.pose.position.y = poscarro[1]
    poses.append(pose)
    pose.pose.position.x = poscarro[0] + magnitudes[i_max]*cos(i_max*data.angle_increment-pi/2+angulocarro)
    pose.pose.position.y = poscarro[1] + magnitudes[i_max]*sin(i_max*data.angle_increment-pi/2+angulocarro)
    poses.append(pose)
    print(pose)
    pose.pose.position.x = poscarro[0] + magnitudes[(i_max+1)]*cos((i_max+1)*data.angle_increment-pi/2+angulocarro)
    pose.pose.position.y = poscarro[1] + magnitudes[(i_max+1)]*sin((i_max+1)*data.angle_increment-pi/2+angulocarro)
    poses.append(pose)
    print(pose)
    """
    # Punto de goal
    vx1 = poscarro[0] + magnitudes[i_max]*cos(i_max*data.angle_increment-pi/2+angulocarro)
    vy1 = poscarro[1] + magnitudes[i_max]*sin(i_max*data.angle_increment-pi/2+angulocarro)
    vx2 = poscarro[0] + magnitudes[(i_max+1)]*cos((i_max+1)*data.angle_increment-pi/2+angulocarro)
    vy2 = poscarro[1] + magnitudes[(i_max+1)]*sin((i_max+1)*data.angle_increment-pi/2+angulocarro)
    Goalx = (vx1+vx2)/2
    Goaly= (vy1+vy2)/2

    compx = Goalx-poscarro[0]
    compy = Goaly-poscarro[1]
    mag_u = sqrt(compx**2+compy**2)
    Vx_u = compx/mag_u
    Vy_u = compy/mag_u
    
    
    # Step 2 Bubble safety
    Bubble_Radius = 1.5
    if abs(min(magnitudes)) < Bubble_Radius:
        indice_min = magnitudes.index(min(magnitudes))
        bubble_x = magnitudes[indice_min]*cos(indice_min*data.angle_increment-pi/2+angulocarro)
        bubble_y = magnitudes[indice_min]*sin(indice_min*data.angle_increment-pi/2+angulocarro)
        bubble_mag = sqrt(bubble_x**2 + bubble_y**2)
        bubble_x_u = bubble_x/ bubble_mag
        bubble_y_u = bubble_y/ bubble_mag
        
    else :
        bubble_x_u = 0
        bubble_y_u = 0
        
    
    
    #Prueba 1 con burbuja de radio 1.5 y vectores de igual magnitud
    #Prueba 2 con burbuja de radio 1.5 y vectort 1.5 veces mayor a vectorr
    Kt = 1
    Kr = 1.3
    magvd = sqrt((Kt*Vx_u - Kr*bubble_x_u)**2 + (Kt*Vy_u - Kr*bubble_y_u)**2)
    vdx = ((Kt*Vx_u - Kr*bubble_x_u) / magvd) * 2   
    vdy =  ((Kt*Vy_u - Kr*bubble_y_u) / magvd) * 2 
    #print("esto es x " + str(vdx))
    #print("esto es y " + str(vdy))
    #vdx = vdx + target_vdx_ant*0.1
    target_vdx_ant = vdx
    target_vdy_ant = vdy
    a = 5
    while(a>0):
        xpath = poscarro[0] + vdx/a
        ypath = poscarro[1] + vdy/a
        xpath2 = poscarro[0] - bubble_x_u/a
        ypath2 = poscarro[1] - bubble_y_u/a
        xpath3 = poscarro[0] + Vx_u/a
        ypath3 = poscarro[1] + Vy_u/a
        pose = PoseStamped()
        pose2 = PoseStamped()
        pose3 = PoseStamped()
        pose.pose.position.x = xpath
        pose.pose.position.y = ypath
        pose2.pose.position.x = xpath2
        pose2.pose.position.y = ypath2
        pose3.pose.position.x = xpath3
        pose3.pose.position.y = ypath3
        poses.append(pose)
        poses2.append(pose2)
        poses3.append(pose3)

        
        
        a=a-1
    path = Path()
    path.header.frame_id ="map"
    path.poses = poses
    Pathpub.publish(path)
    path2 = Path()
    path2.header.frame_id ="map"
    path2.poses = poses2
    Pathpub2.publish(path2)
    path3 = Path()
    path3.header.frame_id ="map"
    path3.poses = poses3
    Pathpub3.publish(path3)
    punto = Odometry()
    punto.pose.pose.position.x = xpath
    punto.pose.pose.position.y = ypath
    punto.pose.pose.position.z = atan(ypath/xpath)
    pub.publish(punto)
    
def odom(data):
    global angulocarro, poscarro
    poscarro[0] = data.pose.pose.position.x
    poscarro[1] = data.pose.pose.position.y
    r,y,z = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    angulocarro = z

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



