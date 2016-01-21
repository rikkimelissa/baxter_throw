#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Trajectory Action Client Example
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
from solve_system import jointPath, jointVelocity, jointAcceleration


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
        self.clear(limb)

    def add_point_p(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
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


def main():
    """RSDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    positions = {
        'left':  [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
        'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
    }

    q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
     -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
    q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
     -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
    q_dot = np.array([-0.23825794, -0.13400971,  0.04931685,  0.0264105 , -0.8301056 , 0.28080345,  0.39270727])
    q_end = np.array([0.9085001216251363,  -1.0089758632316308, 0.07401457301547121, 1.8768254939778037,
     0.18599517053110642, -0.8172282647459542, -0.44600491407768406])

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point_p(current_angles, 0.0)
    t_delay = 3.0
    traj.add_point_p(q_start.tolist(),t_delay)

    T = .75 # set time scale
    N = 100*T # set divisions
    tSpace = np.linspace(0,T,N);
    a = np.array([[1,0,0,0,0,0],[1,1*T,1*T**2,1*T**3,1*T**4,1*T**5],[0,1,0,0,0,0],[0,1,2*T,3*T**2,4*T**3,5*T**4],[0,0,2,0,0,0],[0,0,2,6*T,12*T**2,20*T**3]])

    # Calculate trajectories
    for i in range(7):
        b = np.array([q_start[i],q_throw[i],0,q_dot[i],0,0])
        coeff = np.linalg.solve(a,b)
        jPa = jointPath(tSpace,coeff)
        jVa = jointVelocity(tSpace,coeff)
        jAa = jointAcceleration(tSpace,coeff)
        b = np.array([q_throw[i],q_end[i],q_dot[i],0,0,0])
        coeff = np.linalg.solve(a,b)
        jPb = jointPath(tSpace,coeff) 
        jVb = jointVelocity(tSpace,coeff)
        jAb = jointAcceleration(tSpace,coeff)
        jP = np.hstack((jPa,jPb))
        jV = np.hstack((jVa,jVb))
        jA = np.hstack((jAa,jAb))
        if i==0:
            jP_all = jP
            jV_all = jV
            jA_all = jA
        else:
            jP_all = np.vstack((jP_all, jP))
            jV_all = np.vstack((jV_all, jV))
            jA_all = np.vstack((jA_all, jA))
    t_all = np.linspace(0 + t_delay,2*T + t_delay, N*2);

    for i in range(int(2*N)):
        traj.add_point(jP_all[:,i].tolist(), jV_all[:,i].tolist(), jA_all[:,i].tolist(),t_all[i])
    # p2 = positionS[limb]
    # p3 = positionT[limb]
    # p4 = positionE[limb]
    # traj.add_point_p(p2, 3.0)
    # traj.add_point(p3, velT[limb], 4)
    # traj.add_point_p(p3, 4.0)
    # traj.add_point_p(p4, 5)
    # for i in range(100):
    #     traj.add_point(path1[i,:], vel1[i,:], acc1[i,:], 3 + tSpace1[i])
    # for i in range(100):
    #     traj.add_point(path2[i,:], vel2[i,:], acc2[i,:], 3 + T1 + tSpace2[i])
    traj.start()
    traj.wait(30.0)
    print("Exiting - Joint Trajectory Action Test Complete")


if __name__ == "__main__":
    main()
