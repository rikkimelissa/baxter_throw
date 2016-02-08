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
from solve_system import jointPath, jointVelocity, jointAcceleration
from solve_linear_system import linearSpace


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
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__)
    # required = parser.add_argument_group('required arguments')
    # required.add_argument(
    #     '-l', '--limb', required=True, choices=['left', 'right'],
    #     help='send joint trajectory to which limb'
    # )
    # args = parser.parse_args(rospy.myargv()[1:])
    limb = 'right'

    print("Initializing node... ")
    rospy.init_node("throwing")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    positions = {
        'left':  [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
        'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
    }

 #    q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
 # -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
 #    q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
 #     -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
 #    q_end = np.array([0.9085001216251363,  -1.0089758632316308, 0.07401457301547121, 1.8768254939778037,
 #     0.18599517053110642, -0.8172282647459542, -0.44600491407768406])


    q_throw = np.array([ 0.47668453, -0.77274282,  0.93150983,  2.08352941,  0.54149522,
       -1.26745163, -2.06742261])
    q_end = np.array([ 0.75356806, -0.89162633,  0.54648066,  2.08698086,  0.41033986,
       -1.18423317, -1.15815549])
    q_start = np.array([-0.22281071, -0.36470393,  0.36163597,  1.71920897, -0.82719914,
       -1.16889336, -0.90888362])

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb('right')
    # gripper = baxter_interface.gripper.Gripper('right')
    # gripper.calibrate()
    # # gripper.set_velocity(50)
    # gripper.close()
    # gripper.close()
    # gripper.close()
    # gripper.close()
    rospy.sleep(.5)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point_p(current_angles, 0.0)
    t_delay = 5.0
    # traj.add_point_p(q_start.tolist(),t_delay)


    T = 1.7
    N = 50*T
    dt = float(T)/(N-1)
    vy = .8 # nominal .8
    vz = .4 # nominal .4
    jerk = -20 #nominal -5

    # plt.close('all')

    thList, vList = linearSpace(False, T, N, vy, vz, jerk)
    # thList = thList[1:,:]
    # vList = np.diff(thList,axis=0)/dt
    # vList = np.vstack((np.array([0,0,0,0,0,0,0]),vList))
    aList = np.diff(vList,axis=0)/dt
    aList = np.vstack((np.array([0,0,0,0,0,0,0]),aList))
    t_all = np.linspace(0 + t_delay,2*T + t_delay, N*2);

    # plt.figure()
    # plt.plot(thList)
    # plt.title('Position (joint)')
    # plt.show(block=False)
    # plt.figure()
    # plt.title('Velocity (joint)')
    # plt.plot(vList)
    # plt.show(block=False)
    # plt.figure()
    # plt.title('Acceleration (joint)')
    # plt.plot(aList)
    # plt.show(block=False)

    traj.add_point_p(thList[0,:].tolist(),t_delay)

    for i in range(int(2*N)-2):
        traj.add_point(thList[i,:].tolist(), vList[i,:].tolist(), aList[i,:].tolist(),t_all[i])
        # traj.add_point_p(thList[i,:].tolist(), t_all[i])

    traj.start()
    rospy.sleep(T+t_delay+.005)
    # gripper.open()
    # gripper.open()
    # gripper.open()
    # gripper.open()
    # gripper.open()

    traj.wait(30.0)
    traj.clear(limb)


    print("Exiting - Joint Trajectory Action Test Complete")


if __name__ == "__main__":
    main()
