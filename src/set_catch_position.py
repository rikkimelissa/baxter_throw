#!/usr/bin/env python

import rospy
import baxter_interface
from copy import copy
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from quat import quat_to_so3
import numpy as np
from std_msgs.msg import Int16
from functions import JointTrajectory
from functions import RpToTrans

def sub_cb(data):
    rospy.loginfo(data)
    move_to(data.data)

def move_to(pos):

    pub_demo_state = rospy.Publisher('demo_state',Int16, queue_size = 10)

    if (pos == 1):
        catch = np.array([.65, .25, 0]) # my .7?
        R = np.array([[ 0.26397895, -0.34002068,  0.90260791],
        [-0.01747676, -0.93733484, -0.34799134],
        [ 0.96437009,  0.07608772, -0.25337913]])
    elif (pos == 2):
        catch = np.array([.68, -.05, 0])
        R = np.array([[0,0,1],[0,-1,0],[1,0,0]])
    elif (pos == 3):
        catch = np.array([.72, .1, 0])
        R = np.array([[ 0.26397895, -0.34002068,  0.90260791],
        [-0.01747676, -0.93733484, -0.34799134],
        [ 0.96437009,  0.07608772, -0.25337913]])
    else:
        pass

    th_init = np.array([-.3048, -.2703, -.1848, 1.908, .758, -1.234, -3.04]) 

    # R = np.array([[0,0,1],[0,-1,0],[1,0,0]])

    X = RpToTrans(R,catch)

    # Find end joint angles with IK
    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    kdl_kin = KDLKinematics(robot, base_link, 'left_gripper_base')
    seed = 0
    q_ik = kdl_kin.inverse(X, th_init)
    while q_ik == None:
        seed += 0.01
        q_ik = kdl_kin.inverse(X, th_init+seed)
        if (seed>1):
            # return False
            break
    q_goal = q_ik
    print q_goal

    # limb_interface = baxter_interface.limb.Limb('right')
    limb_interface = baxter_interface.limb.Limb('left')

    angles = limb_interface.joint_angles()

    for ind, joint in enumerate(limb_interface.joint_names()):
        angles[joint] = q_goal[ind]    

    limb_interface.move_to_joint_positions(angles)
    # rospy.sleep(5)
    print 'done'
    pub_demo_state.publish(1)


def main():


    rospy.init_node('set_catch_position')
    sub_pos_state = rospy.Subscriber('pos_state', Int16, sub_cb)

    # pos = rospy.get_param('~pos',1)



    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

