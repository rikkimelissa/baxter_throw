#!/usr/bin/env python
from random import random
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos
import time
from release_state_solve import find_feasible_release
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from functions import RpToTrans
import baxter_interface

catch_x = .8
# catch_y = .1 # .7
catch_z = -.6 # -.6 # range from - .5 to 0 # lower range determined by baxter

def find_path(plot, pos):

    # Set initial position
    q_start = np.array([-0.22281071, -0.36470393,  0.36163597,  1.71920897, -0.82719914,
       -1.16889336, -0.90888362])

    limb_interface = baxter_interface.limb.Limb('right')
    angles = limb_interface.joint_angles()
    for ind, joint in enumerate(limb_interface.joint_names()):
        q_start[ind] = angles[joint] 
    print q_start

    # Find goal for throwing
    if pos == 1:
        catch_y = .7
    elif pos == 2:
        catch_y = .1
    elif pos == 3:
        catch_y = .3
    pos_goal = find_feasible_release(catch_x, catch_y, catch_z, pos)
    block_width = .047
    throw_y = pos_goal[0]
    throw_z = pos_goal[1]
    dy = catch_y - throw_y - block_width
    vel = pos_goal[2]
    alpha = pos_goal[3]
    t = np.linspace(0,dy/(vel*cos(alpha)),100)
    traj_y = vel*cos(alpha)*t + throw_y;
    traj_z = -.5*9.8*t**2 + vel*sin(alpha)*t + throw_z

    if (plot == True):
        plt.close('all')
        plt.figure()
        plt.hold(False)
        plt.plot(traj_y,traj_z,'r',linewidth=2)
        plt.hold(True)
        plt.plot(traj_y[0],traj_z[0],'r.',markersize=15)
        plt.ylim([-.8, .5])
        plt.xlim([-.2, .8])
        plt.xlabel('Y position (m)')
        plt.ylabel('Z position (m)')
        plt.title('Desired trajectory')
        plt.show(block=False)
        wm = plt.get_current_fig_manager()
        wm.window.wm_geometry("800x500+1000+0")
        # wm.window.setGeometry(800,500,0,0)
        raw_input("Press enter to continue...")
    # plt.show(block=False)


    print('found release state')
    print pos_goal

    # Add rotation to position and convert to rotation matrix    
    R = np.array([[0.11121663, -0.14382586,  0.98333361],
       [-0.95290138,  -0.2963578 , 0.06442835],
       [0.28215212, -0.94418546, -0.17001177]])

    p = np.hstack((0.68,pos_goal[0:2]));
    X = RpToTrans(R,p)

    # Find end joint angles with IK
    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
    thInit = q_start
    seed = 0
    q_ik = kdl_kin.inverse(X, thInit)
    while q_ik == None:
        seed += 0.01
        q_ik = kdl_kin.inverse(X, thInit+seed)
        if (seed>1):
            # return False
            break
    q_goal = q_ik
    # print q_goal

    # Transform to joint velocities using Jacobian
    jacobian = kdl_kin.jacobian(q_ik)
    inv_jac = np.linalg.pinv(jacobian)
    Vb = np.array([0,0,0,0,pos_goal[2],pos_goal[3]])
    q_dot_goal = inv_jac.dot(Vb)


    iter = 1000;

    treeA = np.empty((iter+1,14))
    edgesA = np.empty((iter,2))
    treeB = np.empty((iter+1,14))
    edgesB = np.empty((iter,2))
    treeA[0] = np.hstack((q_start,np.array([0,0,0,0,0,0,0])))
    treeB[0] =  np.hstack((q_goal,q_dot_goal.tolist()[0]))
    treeA, treeB, edgesA, edgesB, indA, indB = build_tree(iter,treeA, treeB ,edgesA, edgesB, plot, kdl_kin) 

    np.linalg.norm(q_start - q_goal)
    pathA = np.empty((edgesA.shape[0],14))
    pathA[0] = treeA[indA]
    curEdge = edgesA[indA-1]
    ind = 1
    atOrigin = False
    while atOrigin == False:
        pathA[ind] = treeA[curEdge[0]]
        curEdge = edgesA[curEdge[0] - 1]
        ind += 1
        if curEdge[0] == 0:
            atOrigin = True
    pathA[ind] = treeA[curEdge[0]]
    pathA = pathA[0:ind+1,:]

    pathB = np.empty((edgesB.shape[0],14))
    pathB[0] = treeB[indB]
    curEdge = edgesB[indB-1]
    ind = 1
    atOrigin = False
    # print treeB, pathB
    while atOrigin == False:
        pathB[ind] = treeB[curEdge[0]]
        curEdge = edgesB[curEdge[0] - 1]
        if curEdge[0] == 0:
            atOrigin = True
            # ind += 1
        else:
            ind += 1
    pathB[ind] = treeB[curEdge[0]]
    pathB = pathB[0:ind+1,:]

    path = np.vstack((pathA[::-1],pathB))

    if (plot):
        stepList = np.empty((path.shape[0],3))
        for i in range(path.shape[0]):
            stepList[i] = kdl_kin.forward(path[i,:7])[0:3,3].transpose()
        plt.plot(stepList[:,1],stepList[:,2],'g',linewidth=2)
        plt.show(block=False)
        raw_input('Press enter to continue...')
        plt.close('all')


    # if (plot):
    #     plt.figure()
    #     plt.plot(path[:,0:7],'.')
    #     plt.show(block=False)
    #     print(path.shape)

    return path

def build_tree(iter,treeA, treeB ,edgesA, edgesB, plot, kdl_kin):
    i = 0
    while i < iter:
        # print(i)
        jointsA = np.random.rand(1,7)[0]*[2.5, 2.5, 4, 1.5, 4, 2.5, 4] - [1.25, 1.25, 2.0, .75, 2.0, .75, 2.0]
        velA = np.random.rand(1,7)[0]*[3.0,3.0,3.0,3.0,6.0,3.0,6.0] - [1.5,1.5,1.5,1.5,2,1.5,2]
        node = nearest_neighbor(jointsA, velA, treeA, i)
        treeA = insert_vertex(node,treeA,jointsA,velA,i,1)
        edgesA = insert_edge(edgesA, node, i)

        jointsB = np.random.rand(1,7)[0]*[2.5, 2.5, 4, 1.5, 4, 2.5, 4] - [1.25, 1.25, 2.0, .75, 2.0, .75, 2.0]
        velB = np.random.rand(1,7)[0]*[3.0,3.0,3.0,3.0,6.0,3.0,6.0] - [1.5,1.5,1.5,1.5,2,1.5,2]
        node = nearest_neighbor(jointsB, velB, treeB, i)
        treeB = insert_vertex(node,treeB,jointsB,velB,i,1)
        edgesB = insert_edge(edgesB, node, i)
        result = connect_trees(treeA, treeB, i, kdl_kin)
        # print(result[0],i)
        if (result[0]):
            print "iterated up to: "
            print i
            break

        i = i + 1

    iterations = i

    if (plot):
        # plt.close('all')

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.scatter(treeA[0,0],treeA[0,1],treeA[0,2],'r')
        # ax.plot(treeA[0:i+2,0],treeA[0:i+2,1],treeA[0:i+2,2],'r.')
        # ax.scatter(treeB[0,0],treeB[0,1],treeB[0,2],'b')
        # ax.plot(treeB[0:i+2,0],treeB[0:i+2,1],treeB[0:i+2,2],'b.')
        # plt.show(block=False)
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        pos3A = np.empty((iterations,3))
        for i in range(iterations):
            pos3A[i,:]=kdl_kin.forward(treeA[i,:7])[0:3,3].transpose()
        pos3B = np.empty((iterations,3))
        for i in range(iterations):
            pos3B[i,:]=kdl_kin.forward(treeB[i,:7])[0:3,3].transpose()
        fig = plt.figure(1)
        plt.hold(False)
        # ax = fig.gca(projection='3d')
        plt.plot(pos3A[:,1],pos3A[:,2],'r.')
        plt.hold(True)
        plt.plot(pos3A[0,1],pos3A[0,2],'r.',markersize=15)
        plt.plot(pos3B[:,1],pos3B[:,2],'b.')
        plt.plot(pos3B[0,1],pos3B[0,2],'b.',markersize=15)
        plt.xlabel('EE y-coordinate')
        plt.ylabel('EE z-coordinate')
        plt.title('RRT in end-effector space')
        # plt.show(block=False)


    if (result[0]):
        indA = result[1];
        indB = result[2];
        # if (plot):
            # ax.scatter([treeA[indA,0], treeB[indB,0]],[treeA[indA,1], treeB[indB,1]],[treeA[indA,2], treeB[indB,2]],'g')
    else:
        indA = 0
        indB = 0
    if (plot):
        # plt.xlim([-1.5, 1.5])
        # plt.ylim([-1.5, 1.5])
        for k in range(iterations+1):
            edge1 = kdl_kin.forward(treeA[edgesA[k,0],:7])[0:3,3].transpose()
            edge2 = kdl_kin.forward(treeA[edgesA[k,1],:7])[0:3,3].transpose()
            plt.plot([edge1[0,1],edge2[0,1]],[edge1[0,2],edge2[0,2]],'r',linewidth=3)
            edge1 = kdl_kin.forward(treeB[edgesB[k,0],:7])[0:3,3].transpose()
            edge2 = kdl_kin.forward(treeB[edgesB[k,1],:7])[0:3,3].transpose()
            plt.plot([edge1[0,1],edge2[0,1]],[edge1[0,2],edge2[0,2]],'b',linewidth=3)
        # plt.show(block=False)

    # raw_input('Enter...')

    return treeA[0:iterations+2,:], treeB[0:iterations+2,:], edgesA[0:iterations+1,:], edgesB[0:iterations+1,:], indA, indB

def connect_trees(treeA, treeB, iter, kdl_kin):

    for i in range(iter+1):
        j = 0
        connect = True;
        while j < 7:
            error = (treeA[i,j] - treeB[iter+1,j])**2
            if error > .1:
                connect = False
                break
            j += 1
        if connect == True:        
            return True, i, iter+1
        # if ((treeA[i,0] - treeB[iter+1,0])**2 + (treeA[i,1] - treeB[iter+1,1])**2) < .001:
        #     return True, i, iter+1
    for k in range(iter+1):
        j = 0
        connect = True
        while j < 7:
            error = (treeA[iter+1,j] - treeB[k,j])**2
            if error > .1:
                connect = False
                break
            j += 1
        if connect == True:
            return True, iter+1, k

    return False, 0, 0

def insert_vertex(node,tree,joints,vels,i,dir):

    p1 = tree[node,:]
    p2 = np.hstack((joints,vels))
    step_size = .1
    if dir > 0:
        p = (p2-p1)*step_size + p1
    else:
        p = p2 - (p2-p1)*step_size

    tree[i+1] = p
    # tree[i+1] = [xc,yc,vc[0],vc[1]]
    return tree

def insert_edge(edges, node, i):
    edges[i] = [node, i+1]
    return edges

def nearest_neighbor(joints, vels, tree, nodes):
    min_dist = 100000000
    min_node = 0
    wp = .5
    wv = 1 - wp
    for i in range(nodes+1):
        distPos = 0
        distVel = 0
        for j in range(7):
            distPos += (joints[j] - tree[i,j])**2
            distVel += (vels[j] - tree[i,j+7])**2
        dist = wp*distPos + wv*distVel;
        if (dist < min_dist):
            min_dist = dist
            min_node = i
    return min_node

if __name__ == "__main__":
    start = time.time()
    find_path(True, 1)
    end = time.time()
    print end-start
