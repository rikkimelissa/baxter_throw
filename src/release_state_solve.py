#!/usr/bin/env python

from release_state_sample import test_pos
from functions import RpToTrans
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from math import sin, cos

catch_x = .68
catch_z = -.6 # range from - .5 to 0 # lower range determined by baxter
catch_y = .7 # range from .29 to .5, works after .5 but hard to find solutions


def find_feasible_release():
    found = False;
    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')

    while not found:
        X, th_init, throw_y, throw_z, vel, alpha = sample_state()
        ik_test, q_ik = test_for_ik(X, th_init, kdl_kin)
        if ik_test:
            if test_joint_vel(q_ik, vel, alpha, kdl_kin):
                found = True

    return throw_y, throw_z, vel, alpha

def sample_state():
    res = test_pos(catch_x, catch_y, catch_z)
    while res == None:
        res = test_pos(catch_x, catch_y, catch_z)
    throw_y = res[0]; throw_z = res[1]; vel = res[2]; alpha = res[3]

    throw_x = .68;
    throw_pos = np.array([throw_x, throw_y, throw_z])
    th_init = np.array([ 0.47668453, -0.77274282,  0.93150983,  2.08352941,  0.54149522,
       -1.26745163, -2.06742261]) # experimentally determined
    R_init = np.array([[-0.11121663, -0.14382586,  0.98333361], # experimentally determined
       [-0.95290138,  0.2963578 , -0.06442835],
       [-0.28215212, -0.94418546, -0.17001177]])
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
        if q_dot_throw[0,i] > 2.0 or q_dot_throw[0,i] < -2.0:
            return False
    for i in range(3):
        if q_dot_throw[0,i+4] > 4.0 or q_dot_throw[0,i+4] < -4.0:
            return False
    return True

