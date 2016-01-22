#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import baxter_interface
from copy import copy
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from quat import quat_to_so3
from math import fabs
from functions import JointTrajectory, ScrewTrajectory, CartesianTrajectory, RpToTrans, TransToRp, RotInv

def jointPath(t,coeff):
    a0 = coeff[0]
    a1 = coeff[1]
    a2 = coeff[2]
    a3 = coeff[3]
    a4 = coeff[4]
    a5 = coeff[5]
    a6 = coeff[6]
    a7 = coeff[7]
    s = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5 + a6*t**6 + a7*t**7
    return s

def jointVelocity(t,coeff):
    a0 = coeff[0]
    a1 = coeff[1]
    a2 = coeff[2]
    a3 = coeff[3]
    a4 = coeff[4]
    a5 = coeff[5]
    a6 = coeff[6]
    a7 = coeff[7]
    v = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4 + 6*a6*t**5 + 7*a7*t**6
    return v

def jointAcceleration(t,coeff):
    a0 = coeff[0]
    a1 = coeff[1]
    a2 = coeff[2]
    a3 = coeff[3]
    a4 = coeff[4]
    a5 = coeff[5]
    a6 = coeff[6]
    a7 = coeff[7]
    a = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3 + 30*a6*t**4 + 42*a7*t**5
    return a

def jointJerk(t,coeff):
    a0 = coeff[0]
    a1 = coeff[1]
    a2 = coeff[2]
    a3 = coeff[3]
    a4 = coeff[4]
    a5 = coeff[5]
    a6 = coeff[6]
    a7 = coeff[7]
    j = 6*a3 + 24*a4*t + 60*a5*t**2 + 120*a6*t**3 + 210*a7*t**4
    return j

def linearSpace():
    q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
 -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
    q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
     -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
    q_end = np.array([0.9085001216251363,  -1.0089758632316308, 0.07401457301547121, 1.8768254939778037,
     0.18599517053110642, -0.8172282647459542, -0.44600491407768406])
    X_start = np.asarray(kdl_kin.forward(q_start))
    X_throw = np.asarray(kdl_kin.forward(q_throw))
    X_end = np.asarray(kdl_kin.forward(q_end))
    Vb = np.array([0,0,0,0,1,2])

    T=2;
    N=100;
    xStart = X_start
    xEnd = X_throw
    vStart = np.array([0,0,0,0,0,0])
    vEnd = Vb
    aStart = np.array([0,0,0,0,0,0])
    aEnd = np.array([0,0,0,0,0,0])
    jStart = np.array([0,0,0,0,0,0])
    jEnd = np.array([0,0,0,0,0,-2])
    cartesianPath(xStart, xEnd, vStart, vEnd, aStart, aEnd, jStart, jEnd, T, N)

    for i in range(3):

        b = np.array([X_start[i,3],X_throw[i,3],0,vEnd[3+i],0,0,jStart[i],jEnd[i]])
        coeff = np.linalg.solve(a,b)
        j1Pa = jointPath(tSpace,coeff)
        j1Va = jointVelocity(tSpace,coeff)
        j1Aa = jointAcceleration(tSpace,coeff)
        b = np.array([X_throw[i,3],X_end[i,3],vEnd[3+i],0,0,0,jEnd[i],jStart[i]])
        coeff = np.linalg.solve(a,b)
        j1Pb = jointPath(tSpace,coeff)
        j1Vb = jointVelocity(tSpace,coeff)
        j1Ab = jointAcceleration(tSpace,coeff)

        color = colors[i]
        plt.figure(1)
        plt.hold(True)
        plt.plot(tSpace,j1Pa,'-'+color,label='Joint '+str(i))
        plt.plot(tSpace+tOffset,j1Pb,'--'+color)
        plt.title('Path')
        plt.legend(loc='upper left')
        # plt.show(block = False)

        plt.figure(2)
        plt.plot(tSpace,j1Va,'-'+color,label='Joint '+str(i))
        plt.plot(tSpace+tOffset,j1Vb,'--'+color)
        plt.title('Velocity')
        plt.legend(loc='upper left')
        # plt.show(block = False)

        plt.figure(3)
        plt.hold(True)
        plt.plot(tSpace,j1Aa,'-'+color,label='Joint '+str(i))
        plt.plot(tSpace+tOffset,j1Ab,'--'+color)
        plt.title('Acceleration')
        plt.legend(loc='upper left')

        # plt.show(block = False)

    plt.show(block=False)

def cartesianPath(xStart, xEnd, vStart, vEnd, aStart, aEnd, jStart, jEnd, T, N):
    Rs, ps = TransToRp(xStart)
    Re, pe = TransToRp(xEnd)
    tSpace = np.linspace(0,T,N)
    Rse = RotInv(Rs).dot(Re)
    i = 0
    Xlist = np.empty((N,4,4))
    Xlist[0] = Xstart

    a = np.array([[1,0,0,0,0,0,0,0],[1,T,T**2,T**3,T**4,T**5,T**6,T**7],[0,1,0,0,0,0,0,0],[0,1,2*T,3*T**2,4*T**3,5*T**4,6*T**5,7*T**6],
        [0,0,2,0,0,0,0,0],[0,0,2,6*T,12*T**2,20*T**3,30*T**4,42*T**5],[0,0,0,6,0,0,0,0],[0,0,0,6,24*T,60*T**2,120*T**3,210*T**4]])
    # b = np.array([q_start[i],q_throw[i],0,q_dot[i],0,0,0,jerk])
    b = np.array([xStart[0,3],xEnd[0,3],0,vEnd[3],0,0,0,0])
    coeff = np.linalg.solve(a,b)





    for t in tSpace[1:]:
        i = i+1;
        if timeScale == 3:
            s = CubicTimeScaling(T,t)
        elif timeScale == 5:
            s = QuinticTimeScaling(T,t)
        else:
            raise ValueError('timeScale must be 3 or 5')
        p = ps + s*(pe-ps)
        R = Rs.dot(MatrixExp3(MatrixLog3(Rse)*s))
        Xlist[i] = RpToTrans(R,p)
    return Xlist



def jointSpace():
    q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
 -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
    q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
     -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
    q_dot = np.array([-0.23825794, -0.13400971,  0.04931685,  0.0264105 , -0.8301056 , 0.28080345,  0.39270727])
    q_end = np.array([0.9085001216251363,  -1.0089758632316308, 0.07401457301547121, 1.8768254939778037,
     0.18599517053110642, -0.8172282647459542, -0.44600491407768406])
    jerk = -10;

    colors = ['r','b','c','y','m','oc','k']
    plt.close('all')

    T = 3
    N = 200*T
    tSpace = np.linspace(0,T,N);
    tOffset = T
    a = np.array([[1,0,0,0,0,0,0,0],[1,T,T**2,T**3,T**4,T**5,T**6,T**7],[0,1,0,0,0,0,0,0],[0,1,2*T,3*T**2,4*T**3,5*T**4,6*T**5,7*T**6],
        [0,0,2,0,0,0,0,0],[0,0,2,6*T,12*T**2,20*T**3,30*T**4,42*T**5],[0,0,0,6,0,0,0,0],[0,0,0,6,24*T,60*T**2,120*T**3,210*T**4]])

    for i in range(7):

        b = np.array([q_start[i],q_throw[i],0,q_dot[i],0,0,0,jerk])
        coeff = np.linalg.solve(a,b)
        j1Pa = jointPath(tSpace,coeff)
        j1Va = jointVelocity(tSpace,coeff)
        j1Aa = jointAcceleration(tSpace,coeff)
        b = np.array([q_throw[i],q_end[i],q_dot[i],0,0,0,jerk,0])
        coeff = np.linalg.solve(a,b)
        j1Pb = jointPath(tSpace,coeff)
        j1Vb = jointVelocity(tSpace,coeff)
        j1Ab = jointAcceleration(tSpace,coeff)

        color = colors[i]
        plt.figure(1)
        plt.hold(True)
        plt.plot(tSpace,j1Pa,'-'+color,label='Joint '+str(i))
        plt.plot(tSpace+tOffset,j1Pb,'--'+color)
        plt.title('Path')
        plt.legend(loc='upper left')
        # plt.show(block = False)

        plt.figure(2)
        plt.plot(tSpace,j1Va,'-'+color,label='Joint '+str(i))
        plt.plot(tSpace+tOffset,j1Vb,'--'+color)
        plt.title('Velocity')
        plt.legend(loc='upper left')
        # plt.show(block = False)

        plt.figure(3)
        plt.hold(True)
        plt.plot(tSpace,j1Aa,'-'+color,label='Joint '+str(i))
        plt.plot(tSpace+tOffset,j1Ab,'--'+color)
        plt.title('Acceleration')
        plt.legend(loc='upper left')

        # plt.show(block = False)

    plt.show(block=False)

if __name__ == "__main__":
    x = 1