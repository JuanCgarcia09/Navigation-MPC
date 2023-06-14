#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 15:12:07 2022

@author: juan
"""
import rospy
from ackermann_msgs.msg import *
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import sys
from casadi import *
from math import *
import matplotlib.pyplot as plt
from matplotlib import rcParams
from do_mpc.data import save_results, load_results

sal0 = [0,0]

Ts=0.01 # SAMPLING TIME
Lr = 0.3302
L= Lr*2
target = [0, 0,0]
const = [0,0,0,0]
PosSave = [0,0,0,0,0]
u0 = [0,0]
i = 0
Timbre = True
#Initial_State =  np.array([0, 0,0]).reshape(-1,1)
# Import do_mpc package:
import do_mpc
salidas=np.zeros((2,20))
plt.close('all')
ite=0
CM = True # Flag to change the model

targetx = 1000 
targety = 1000
targetz = 1000

model_type = 'discrete' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

_x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
_y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
_t = model.set_variable(var_type='_x', var_name='t', shape=(1,1))# x,y,Theta
_u = model.set_variable(var_type='_u', var_name='u', shape=(2,1))#V,Delta

Beta=np.arctan((Lr/L)*np.tan(_u[1]))
X_next=_x+_u[0]*np.cos(_t+_u[1])*Ts
Y_next=_y+_u[0]*np.sin(_t+_u[1])*Ts
T_next=_t+(_u[0]*np.cos(Beta)*np.tan(_u[1]))


model.set_rhs('x', X_next)
model.set_rhs('y', Y_next)
model.set_rhs('t', T_next)


model.setup()
mpc = do_mpc.controller.MPC(model)



pub = rospy.Publisher('drive',AckermannDriveStamped,queue_size=10)
#Pathpub = rospy.Publisher('smoothed_path',Path,queue_size=10)

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

def modelo(x0,uact,target,const):
    global CM, ite, mpc, Ts
    if CM :
        mpc = do_mpc.controller.MPC(model)
        
        setup_mpc = {
            'n_robust': 0,
            'n_horizon': 10,
            't_step': 0.01,
            'state_discretization': 'discrete',
            'store_full_solution':True,
#            'ipopt.print_level':0, 
#            'ipopt.sb': 'yes', 
#            'print_time':0,
#            'ipopt.linear_solver': 'MA27'
        }
        mpc.set_param(**setup_mpc)
        mpc.set_param(nlpsol_opts = {'ipopt.print_level':0,'ipopt.sb': 'yes', 'print_time':0,'ipopt.linear_solver': 'MA27'})
        
        
        mterm = (_x-target[0])**2+(_y-target[1])**2#+(_t-target[2])**2
        lterm = (_x-target[0])**2+(_y-target[1])**2#+(_t-target[2])**2
        
        mpc.set_objective(mterm=mterm, lterm=lterm)
        
        mpc.set_rterm(u=1)
        
        max_x = const[1]
        min_x = const[0]
        
        max_y = const[3]
        min_y = const[2]
        
        max_t = pi+0.3
        
        max_u = np.array([[2.0], [pi/8]])
        min_u = np.array([[0.3], [-pi/8]])
        
        # lower bounds of the states
        mpc.bounds['lower','_x','x'] = min_x
        mpc.bounds['lower','_x','y'] = min_y
        mpc.bounds['lower','_x','t'] = -max_t
        
        # upper bounds of the states
        mpc.bounds['upper','_x','x'] = max_x
        mpc.bounds['upper','_x','y'] = max_y
        mpc.bounds['upper','_x','t'] = max_t
        
        # lower bounds of the input
        mpc.bounds['lower','_u','u'] = min_u
        
        # upper bounds of the input
        mpc.bounds['upper','_u','u'] =  max_u
        
    #    mpc.scaling['_u', 'u'] = np.array([[2], [pi/40]])
        
        mpc.setup()
        
        #simulator = do_mpc.simulator.Simulator(model)
        
        #simulator.set_param(t_step = Ts)
        #simulator.setup()
        # Initial state
        mpc.x0 = x0
        #mpc.u0['u'] = uact
        #simulator.x0 = x0
        # Use initial state to set the initial guess.
        mpc.set_initial_guess()
        CM = False
        
        
    u0 = mpc.make_step(x0)
    """
    xpre = mpc.data.prediction(('_x', 'x'))
    ypre = mpc.data.prediction(('_x', 'y'))
    xsali = []
    ysali = []
    for element in xpre[0]:
        xsali.append(element[0])
    for element in ypre[0]:
        ysali.append(element[0])
    poses = []
    for i in range(len(xsali)):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.frame_id ="/path"
        pose.pose.position.x = xsali[i]
        pose.pose.position.y = ysali[i]
        poses.append(pose)
    path = Path()
    path.header.frame_id ="map"
    path.poses = poses
    Pathpub.publish(path)
    
    #u0 = [0,0]
    print(x0)
    print(ite)
    print(u0)
    print(Cambios)
    #x0 = simulator.make_step(u0)
    """
    """
    rcParams['axes.grid'] = True
    rcParams['font.size'] = 18
    
    fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(16,9))
    graphics.plot_results()
    graphics.reset_axes()
    plt.show()
    """
    #x = mpc.data['_x']
    return u0

def main():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber("odom",Odometry,MPCF)
    rospy.Subscriber("obstacle",Odometry,OBS)
    rospy.spin()
    
def MPCF(data):
    global salidas,target,CM,targetx, targety, targetz,u0
    drive_st_msg = AckermannDriveStamped()
    drive = AckermannDrive()
    r,y,z = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    x0 =  np.array([data.pose.pose.position.x,data.pose.pose.position.y,z]).reshape(-1,1)
    const = [-1000,1000,-1000,1000]
    #print(x0)
    if(targetx<999):
            if CM:
                target =[targetx, targety, targetz]
            salidas = modelo(x0,u0,target,const)
            u0 = salidas
    if sqrt((target[0]-x0[0])**2+(target[1]-x0[1])**2) <2:
        CM = True
    
    #meta = (data.pose.pose.position.x-target[0])**2 + (data.pose.pose.position.y-target[1])**2
    #print(meta)
    """
    if salidas[0]<0.1:
        CM =True
        ite=ite+1
        """
    drive.speed = salidas[0]
    drive.steering_angle = salidas[1]
    drive_st_msg.drive = drive
    pub.publish(drive_st_msg)

    """
    if Timbre:
        drive.speed = salidas[i,0]
        drive.steering_angle = salidas[i,1]
        drive_st_msg.drive = drive
        pub.publish(drive_st_msg)
        ite = ite+1
    else:
        drive.speed = 0
        drive.steering_angle = 0
        drive_st_msg.drive = drive
        pub.publish(drive_st_msg)
        Timbre = False
        print("Hola llegó papi")
    """

def OBS(data):
    global targetx, targety, targetz
    targetx = data.pose.pose.position.x
    targety = data.pose.pose.position.y
    targetz = data.pose.pose.position.z

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



