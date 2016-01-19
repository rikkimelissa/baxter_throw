#!/usr/bin/env python
"""
Baxter RSDK Joint Trajectory Action Client Example
rosrun baxter_interface joint_trajectory_action_server.py
rosrun baxter_examples joint_recorder.py -f "test"
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

import numpy as np

import rosbag

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.2)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, velocities, accelerations,time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.accelerations = copy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def add_point_p(self, positions,time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        print('started')
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

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    # limb_interface = baxter_interface.limb.Limb(limb)
    # current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # traj.add_point(current_angles, 0.0)

    # p1 = positions[limb]
    # traj.add_point(p1, 7.0)
    # traj.add_point([x * 0.75 for x in p1], 9.0)
    # traj.add_point([x * 1.25 for x in p1], 12.0)

    q_start = np.array([-0.61159692 , 0.75344562 , 1.05147772 , 1.60237397 ,  2.13362129 ,  1.12027284, 
      1.95830825])
    q_throw = np.array([ 0.55703127,  1.02672179,  2.36138413,  1.08580171,  3.05891801,
     -1.42679103, -0.42898074])
    q_end = np.array([ 0.84862266 ,  0.97046182 ,  2.15649338 ,  1.77949586,   2.03984311 ,  1.74583931, 
      1.06574319])
    N = 1

    path1 = JointTrajectorySpeedUp(q_start,q_throw,N,100,1);
    path2 = JointTrajectorySlowDown(q_throw,q_start,N,100,1)
    vel1 = np.diff(path1,axis=0)
    vel1 = np.vstack(([0,0,0,0,0,0,0],vel1))
    vel2 = np.diff(path2,axis=0)
    vel2 = np.vstack(([0,0,0,0,0,0,0],vel2))
    acc1 = np.diff(vel1,axis=0)
    acc1 = np.vstack(([0,0,0,0,0,0,0],acc1))
    acc2 = np.diff(vel2,axis=0)
    acc2 = np.vstack(([0,0,0,0,0,0,0],acc2))
    tSpace = np.linspace(0,N,100)

    traj.add_point_p(path1[0,:],3)
    traj.start()
    traj.wait(7)

    for i in range(100):
        pos = path1[i,:]
        vel = vel1[i,:]
        acc = acc1[i,:]
        time = tSpace[i]
        # traj.add_point(pos,vel,acc,time)
        traj.add_point_p(pos,time)

    # for i in range(100):
    #     pos = path2[i,:]
    #     vel = vel2[i,:]
    #     acc = acc2[i,:]
    #     time = tSpace[i] + N
    #     traj.add_point(pos,vel,acc,7 + N + time)

    traj.start()
    print(traj.result())

    traj.wait(15.0)
    print("Exiting - Joint Trajectory Action Test Complete")

def JointTrajectorySpeedUp(thStart, thEnd, T, N, timeScale):
    thList = [thStart]
    if (N < 2):
        raise ValueError('N must be 2 or greater')
    tSpace = np.linspace(0,T,N)
    for t in tSpace[1:]:
        s = jointUp(t,T)
        th = (1-s)*thStart + s*thEnd
        thList = np.concatenate((thList,[th]), axis=0)
    return thList

def JointTrajectorySlowDown(thStart, thEnd, T, N, timeScale):
    thList = [thStart]
    if (N < 2):
        raise ValueError('N must be 2 or greater')
    tSpace = np.linspace(0,T,N)
    for t in tSpace[1:]:
        s = jointDown(t,T)
        th = (1-s)*thStart + s*thEnd
        thList = np.concatenate((thList,[th]), axis=0)
    return thList

def jointUp(t,T):
    s = (-3*t**5*(-2+T))/T**5 - (t**4*(15-7*T))/T**4  - (2*t**3*(-5+2*T))/T**3
    return s

def jointDown(t,T):
    s = t - (3*t**5*(-2+T))/T**5 - (t**4*(15-8*T))/T**4  - (2*t**3*(-5+3*T))/T**3
    return s

if __name__ == "__main__":
    main()
