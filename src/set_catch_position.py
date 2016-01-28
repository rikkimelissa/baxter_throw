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

def main():

    q_set = np.array([-.3048, -.2703, -.1848, 1.908, .758, -1.234, -3.04]) 

    # robot = URDF.from_parameter_server()
    # base_link = robot.get_root()
    # kdl_kin = KDLKinematics(robot, base_link, 'left_gripper_base')
    # q0 = kdl_kin.random_joint_angles()

    rospy.init_node('set_catch_position')
    limb_interface = baxter_interface.limb.Limb('left')
    # current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # for ind in range(len(q0)):
    #     q0[ind] = q_set[ind]

    angles = limb_interface.joint_angles()

    for ind, joint in enumerate(limb_interface.joint_names()):
        angles[joint] = q_set[ind]    

    limb_interface.move_to_joint_positions(angles)
    # rospy.sleep(5)


    # rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# {'left_e0': -0.18484468494019235,
#  'left_e1': 1.9086555953264261,
#  'left_s0': -0.304878681592226,
#  'left_s1': -0.2703641138648042,
#  'left_w0': 0.7581700044123657,
#  'left_w1': -1.2340875438538152,
#  'left_w2': -3.0472528351343744}
