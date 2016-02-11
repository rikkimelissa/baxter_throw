#!/usr/bin/env python

import numpy as np
import math
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

def run_solver():

    q_throw = np.array([ .486, -.777, .919, 2.08, .544, -1.26, -2.06])
    q_catch = np.array([-.3048, -.2703, -.1848, 1.908, .758, -1.234, -3.04]) 

    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    kdl_kin_r = KDLKinematics(robot, base_link, 'right_gripper_base')
    kdl_kin_l = KDLKinematics(robot, base_link, 'left_gripper_base')

    pos = np.array([ 0.4072719 , -0.82681564,  0.58521367,  2.18285466,  0.4314321 ,
           -1.19497103, -1.83195656])
    v = np.array([ 0.01305719, -0.00549504,  0.00665054, -0.00858544,  0.02209154,
            0.00171301, -0.01284494])
    matrix([[ 0.00290611,  0.00748189,  0.00420126, -0.0027068 , -0.00166938,
         -0.00100164]])

    dt = .01
    vListAct = np.diff(posListDes,axis=0)/dt
    vCarlist = np.empty((840,6))
    posList = np.empty((840,3))
    for i in range(840):
        p = posListAct[i,:];
        v = vListAct[i,:]
        jacobian = kdl_kin_r.jacobian(pos)
        vCar = jacobian.dot(v)
        vCarlist[i,:] = vCar;
        X = np.asarray(kdl_kin_r.forward(p))
        posList[i,:] = X[0:3,3]
    if (plot==True):
        plt.figure()
        plt.title('Jacobian based cartesian velocities')
        plt.plot(vCarlist[:,0:3])
        plt.show(block=False)
    if (plot==True):
        plt.figure()
        plt.title('Jacobian based cartesian positions')
        plt.plot(posList)
        plt.show(block=False)


    # q0 = kdl_kin_r.random_joint_angles()
    # limb_interface = baxter_interface.limb.Limb('right')
    # current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # for ind in range(len(q0)):
    #     q0[ind] = current_angles[ind]

    X_throw = np.asarray(kdl_kin_r.forward(q_throw))
    X_catch = np.asarray(kdl_kin_l.forward(q_catch))

    throwPos = np.array([.75, -.08, .29])
    # catchPos = np.array([.803, .245, -.0369])
    catchPos = np.array([.8, .24, -.05])
    # throwPos = np.array([.742, -.19, .26])

    t_act = .225
    vy = .86 # nominal .8
    vz = .37 # nominal .4

    dx = catchPos[0] - throwPos[0]
    dy = catchPos[1] - throwPos[1] - .047 - .04
    dz = catchPos[2] + .12 - throwPos[2] - .018

    # z direction
    t1 = solve_quad(.5*9.8,-vz, dz)
    t2 = dy/vy
    print t1,t2

    # y direction

    t = 0;
    py0 = throwPos[1]
    pz0 = throwPos[2]
    pyList = np.empty(26)
    pzList = np.empty(26)
    for i in range(26):
        py = py0 + vy*t;
        pz = pz0 + vz*t - .5*9.8*t**2;
        pyList[i] = py;
        pzList[i] = pz;
        t += .01;

    plt.close('all')
    plt.figure()
    plt.plot(pyList, pzList,'.')
    plt.axis('equal')
    # plt.plot(pzList)
    plt.show(block=False)

    vz = np.diff(pzList)/.01
    vy = np.diff(pyList)/.01
    plt.figure()
    plt.plot(vz)
    plt.plot(vy,'r')
    plt.show(block=False)


