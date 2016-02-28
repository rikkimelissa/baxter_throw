#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from rrt_joint import find_path
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from functions import RpToTrans
import matplotlib.pyplot as plt


def publish():

    pub = rospy.Publisher('joint_state_check', numpy_msg(Float32MultiArray), queue_size = 10)
    rospy.init_node('jsc_publsher')
    rate = rospy.Rate(60)
    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')

    path = find_path(False)

    # Convert list to appropriate shape for checking
    a = Float32MultiArray()
    N = path.shape[0]
    thList_arms = np.hstack((path[:,0:7],np.zeros((N,7))))
    thList_all = np.hstack((np.zeros((N,1)),thList_arms))
    data = np.reshape(thList_all,(N*15,1))
    a.data = np.array(data, dtype = np.float32)
    rospy.loginfo(a.data)
    rospy.loginfo('publishing')
    pub.publish(a)

    rospy.spin()
    
if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        pass

