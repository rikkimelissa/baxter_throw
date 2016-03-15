#!/usr/bin/env python

from release_state_sample import test_pos
from functions import RpToTrans
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from math import sin, cos
import matplotlib.pyplot as plt

catch_x = .8
catch_z = -.6 # range from - .5 to 0 # lower range determined by baxter
catch_y = .5 # range from -.3 to .7, works after .5 but hard to find solutions

def main():
    plt.close('all')
    plt.figure(facecolor='w')
    plt.hold(True)
    # for i in range(30):
    #     throw_y, throw_z, vel, alpha = find_feasible_release(catch_x, catch_y, catch_z)
    # # print throw_y, throw_z, vel, alpha
    #     block_width = .047
    #     dy = catch_y - throw_y - block_width
    #     t = np.linspace(0,dy/(vel*cos(alpha)),100)
    #     traj_y = vel*cos(alpha)*t + throw_y;
    #     traj_z = -.5*9.8*t**2 + vel*sin(alpha)*t + throw_z
    #     plt.plot(traj_y,traj_z,'b',alpha=.3)
    #     plt.plot(traj_y[0],traj_z[0],'b.',markersize=15,alpha = .3)
    #     plt.ylim([(catch_z - .2), .4])
    throw_y, throw_z, vel, alpha = find_feasible_release(catch_x, catch_y, catch_z)
    block_width = .047
    dy = catch_y - throw_y - block_width
    t = np.linspace(0,dy/(vel*cos(alpha)),100)
    traj_y = vel*cos(alpha)*t + throw_y;
    traj_z = -.5*9.8*t**2 + vel*sin(alpha)*t + throw_z
    plt.plot(traj_y,traj_z,'r',linewidth=2)
    plt.plot(traj_y[0],traj_z[0],'r.',markersize=15)
    plt.ylim([-.6, .5])
    plt.xlabel('Y position (m)')
    plt.ylabel('Z position (m)')
    plt.title('Trajectories for sample release position, velocity, and angle')
    plt.show(block=False)

def find_feasible_release(catch_x, catch_y, catch_z):
    found = False;
    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')

    while not found:
        X, th_init, throw_y, throw_z, vel, alpha = sample_state(catch_x, catch_y, catch_z)
        ik_test, q_ik = test_for_ik(X, th_init, kdl_kin)
        if ik_test:
            if test_joint_vel(q_ik, vel, alpha, kdl_kin):
                found = True
        print q_ik

    return throw_y, throw_z, vel, alpha

def sample_state(catch_x, catch_y, catch_z):
    res = test_pos(catch_x, catch_y, catch_z)
    while res == None:
        res = test_pos(catch_x, catch_y, catch_z)
    throw_y = res[0]; throw_z = res[1]; vel = res[2]; alpha = res[3]

    throw_x = .68;
    throw_pos = np.array([throw_x, throw_y, throw_z])
    th_init = np.array([ 0.47668453, -0.77274282,  0.93150983,  2.08352941,  0.54149522,
       -1.26745163, -2.06742261]) # experimentally determined
 #    # flipped starting location
 #    th_init = np.array([-0.22281071, -0.36470393,  0.36163597,  1.71920897, -0.82719914,
 #       -1.16889336, .89])
 #    th_init = np.array([0.5065971551991579,
 # -0.9644904203829539,
 # 0.5472476460781214,
 # 2.2047138873883108,
 # 0.1342233189399737,
 # -1.2233496783386175,
 # -2.2518837966157874])
    R_init = np.array([[ 0.06713617, -0.05831221,  0.99603836],
        [-0.97267055, -0.22621871,  0.05231732],
        [ 0.22227178, -0.97232956, -0.07190603]])
    # # flipped ending orientation
    # R_init = np.array([[-0.05154769, -0.02369375,  0.99838942],
    #     [ 0.9421129 ,  0.33050364,  0.05648559],
    #     [-0.33130969,  0.94350726,  0.00528549]])
    # R_init = np.array([[ 0.04276406,  0.0272318 ,  0.99871401],
    #     [-0.94477517, -0.32399151,  0.04928868],
    #     [ 0.32491708, -0.94566798,  0.01187273]])
    # R_init = np.array([[ 0.04304468,  0.02723819,  0.99870178],
    #     [-0.94506274, -0.32311249,  0.0495452],
    #     [ 0.32404254, -0.9459685 ,  0.01183352]])


    X = RpToTrans(R_init, throw_pos)
    return X, th_init, throw_y, throw_z, vel, alpha

def test_for_ik(X, th_init, kdl_kin):
    seed = 0
    q_ik = kdl_kin.inverse(X, th_init)
    while q_ik == None:
        seed += 0.03
        q_ik = kdl_kin.inverse(X, th_init+seed)
        if (seed>1):
            return False;
            break
    return True, q_ik

def test_joint_vel(q_ik, vel, alpha, kdl_kin):

    v_y = vel*cos(alpha)
    v_z = vel*sin(alpha)
    jacobian = kdl_kin.jacobian(q_ik)
    inv_jac = np.linalg.pinv(jacobian)
    Vb = np.array([0,0,0,0,v_y,v_z])
    q_dot_throw = inv_jac.dot(Vb)
    for i in range(4):
        if q_dot_throw[0,i] > 1.5 or q_dot_throw[0,i] < -1.5:
            return False
    for i in range(3):
        if q_dot_throw[0,i+4] > 3.0 or q_dot_throw[0,i+4] < -3.0:
            return False
    return True

if __name__ == '__main__':
    main()