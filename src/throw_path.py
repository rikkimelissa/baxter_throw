#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#

# Modified by Rikki Irwin rikkiirwin@gmail.com

"""
Baxter JTAS throw_path
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

from functions import JointTrajectory
import numpy as np
from solve_linear_system import linearSpace
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg

import matplotlib.pyplot as plt


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self._sub_path = rospy.Subscriber('joint_path', numpy_msg(Float32MultiArray), self.path_cb)
        self._sub_traj = rospy.Subscriber('joint_traj', numpy_msg(Float32MultiArray), self.traj_cb)
        self._limb_interface = baxter_interface.limb.Limb('right')
        self._q_start = np.array([-0.22281071, -0.36470393,  0.36163597,  1.71920897, -0.82719914,
       -1.16889336, -0.90888362])
        self.clear(limb)

    def add_point_p(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def add_point_pv(self, positions, velocities, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def add_point(self, positions, velocities, accelerations, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.accelerations = copy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    def path_cb(self,a):
        rospy.loginfo('Received path')
        pList = a.data;
        pMat = np.reshape(pList,(pList.shape[0]/14,14))
        self._pArm = pMat[:,0:7]
        self._vArm = pMat[:,7:14]
        self.execute_path()

    def traj_cb(self,a):
        rospy.loginfo('Received trajectory')
        pList = a.data;
        pMat = np.reshape(pList,(pList.shape[0]/15,15))
        self._time = pMat[:,0]
        self._pArm = pMat[:,1:8]
        self._vArm = pMat[:,8:]

        plt.figure()
        plt.hold(True)
        plt.plot(self._time, self._pArm)
        plt.plot(self._time, self._vArm)
        plt.show(block=False)
        
        self.execute_traj()    

    def execute_path(self):
        N = self._pArm.shape[0]
        T = 10
        t_all = np.linspace(0 + self._t_delay,T + self._t_delay, N);

        current_angles = [self._limb_interface.joint_angle(joint) for joint in self._limb_interface.joint_names()]
        self.add_point_p(current_angles, 0.0)
        self.add_point_p(self._q_start,self._t_delay)

        for i in range(N):
            self.add_point_pv(self._pArm[i,:].tolist(), self._vArm[i,:].tolist(),t_all[i])
        # traj.add_point_p(thList[i,:].tolist(), t_all[i])
        self.start()
        self.wait(10)
        self.clear('right')

    def execute_traj(self):
        N = self._pArm.shape[0]

        current_angles = [self._limb_interface.joint_angle(joint) for joint in self._limb_interface.joint_names()]
        self.add_point_p(current_angles, 0.0)
        self.add_point_p(self._q_start,self._t_delay)

        for i in range(N):
            self.add_point_pv(self._pArm[i,:].tolist(), self._vArm[i,:].tolist(),self._time[i]+self._t_delay+1)
        # traj.add_point_p(thList[i,:].tolist(), t_all[i])
        self.start()
        self.wait(10)
        self.clear('right')

def main():

    print("Initializing node... ")
    rospy.init_node("throwing")

    traj = Trajectory('right')
    traj._t_delay = 5.0
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first

    rospy.spin()


    print("Exiting - Joint Trajectory Action Test Complete")


if __name__ == "__main__":
    main()
