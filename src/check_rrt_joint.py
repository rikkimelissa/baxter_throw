#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int16
from rospy.numpy_msg import numpy_msg
from rrt_joint import find_path
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from functions import RpToTrans
import matplotlib.pyplot as plt

class Checker(object):
    def __init__(self):
        self._coll_checked = False
        self._pub_joints = rospy.Publisher('joint_state_check', numpy_msg(Float32MultiArray), queue_size = 10)
        self._sub = rospy.Subscriber('collision_check', Int16, self.sub_cb)
        self._pub_path = rospy.Publisher('joint_path',numpy_msg(Float32MultiArray), queue_size = 10)

    def sub_cb(self,a):
        rospy.loginfo(a)
        if (a.data):
            rospy.loginfo('Sending path')
            self.publish_path()
        else:
            rospy.loginfo('Calculating new path')
            self.find_new_path()

    def publish_path(self):
        a = Float32MultiArray()
        N = self._path.shape[0]
        data = np.reshape(self._path,(N*14,1))
        a.data = np.array(data, dtype = np.float32)
        rospy.loginfo('publishing path')
        self._pub_path.publish(a)

    def find_new_path(self):
        self._path = find_path(False)
        self.publish_joints()

    def publish_joints(self):
        a = Float32MultiArray()
        N = self._path.shape[0]
        thList_arms = np.hstack((self._path[:,0:7],np.zeros((N,7))))
        thList_all = np.hstack((np.zeros((N,1)),thList_arms))
        data = np.reshape(thList_all,(N*15,1))
        a.data = np.array(data, dtype = np.float32)
        rospy.loginfo(self._path.shape)
        rospy.loginfo('checking collision')
        self._pub_joints.publish(a)   

def main():

    rospy.init_node('jsc_publisher')
    rate = rospy.Rate(60)    
    check = Checker()
    check._path = find_path(False)
    check.publish_joints()

    rospy.spin()

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

