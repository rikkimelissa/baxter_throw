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


catch_x = .68
catch_z = -.6 # range from - .5 to 0 # lower range determined by baxter
catch_y = .7 

def find_path(plot):

    # Set initial position
    pos_init = [-.62, -.1, 0, 0]
    q_start = np.array([-0.22281071, -0.36470393,  0.36163597,  1.71920897, -0.82719914,
       -1.16889336, -0.90888362])

    # Find goal for throwing
    pos_goal = find_feasible_release(catch_x, catch_y, catch_z)

    # Add rotation to position and convert to rotation matrix    
    R = np.array([[-0.11121663, -0.14382586,  0.98333361],
       [-0.95290138,  0.2963578 , -0.06442835],
       [-0.28215212, -0.94418546, -0.17001177]])
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


    if (plot):
        fig = plt.figure(1)
        ax = fig.gca(projection='3d')
        # ax.plot(pathA[:,0],pathA[:,1],pathA[:,2],'g',linewidth=5)

    pathB = np.empty((edgesB.shape[0],14))
    pathB[0] = treeB[indB]
    curEdge = edgesB[indB-1]
    ind = 1
    atOrigin = False
    print treeB, pathB
    while atOrigin == False:
        pathB[ind] = treeB[curEdge[0]]
        curEdge = edgesB[curEdge[0] - 1]
        if curEdge[0] == 0:
            atOrigin = True
        else:
            ind += 1
    pathB[ind] = treeB[curEdge[0]]
    pathB = pathB[0:ind+1,:]

    path = np.vstack((pathA[::-1],pathB))

    if (plot):
        ax.plot(path[:,0],path[:,1],path[:,2],'g',linewidth=5)
        plt.show(block=False)

    if (plot):
        plt.figure()
        plt.plot(path[:,0:7],'.')
        plt.show(block=False)
        print(path.shape)

    return path

def build_tree(iter,treeA, treeB ,edgesA, edgesB, plot, kdl_kin):
    i = 0
    while i < iter:

        # jointsA = np.random.rand(1,7)[0]*[3.4033, 3.194, 6.1083, 2.67, 6.117, 3.6647, 6.117] - [1.7016, 2.147, 3.05, .05, 3.059, 1.57, 3.059]
        jointsA = np.random.rand(1,7)[0]*[2.5, 2.5, 4, 1.5, 4, 2.5, 4] - [1.25, 1.25, 2.0, .75, 2.0, 1.25, 2.0]
        velA = np.random.rand(1,7)[0]*[3.0,3.0,3.0,3.0,6.0,6.0,6.0] - [1.5,1.5,1.5,1.5,2,2,2]
        node = nearest_neighbor(jointsA, velA, treeA, i)
        treeA = insert_vertex(node,treeA,jointsA,velA,i,1)
        edgesA = insert_edge(edgesA, node, i)

        jointsB = np.random.rand(1,7)[0]*[2.5, 2.5, 4, 1.5, 4, 2.5, 4] - [1.25, 1.25, 2.0, .75, 2.0, 1.25, 2.0]
        velB = np.random.rand(1,7)[0]*[3.0,3.0,3.0,3.0,6.0,6.0,6.0] - [1.5,1.5,1.5,1.5,2,2,2]
        node = nearest_neighbor(jointsB, velB, treeB, i)
        treeB = insert_vertex(node,treeB,jointsB,velB,i,1)
        edgesB = insert_edge(edgesB, node, i)
        result = connect_trees(treeA, treeB, i, kdl_kin)
        # print(result[0],i)
        if (result[0]):
            print i
            break

        i = i + 1


    if (plot):
        plt.close('all')

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(treeA[0,0],treeA[0,1],treeA[0,2],'r')
        ax.plot(treeA[0:i+2,0],treeA[0:i+2,1],treeA[0:i+2,2],'r.')
        ax.scatter(treeB[0,0],treeB[0,1],treeB[0,2],'b')
        ax.plot(treeB[0:i+2,0],treeB[0:i+2,1],treeB[0:i+2,2],'b.')
        plt.show(block=False)

    if (result[0]):
        indA = result[1];
        indB = result[2];
        if (plot):
            ax.scatter([treeA[indA,0], treeB[indB,0]],[treeA[indA,1], treeB[indB,1]],[treeA[indA,2], treeB[indB,2]],'g')
    else:
        indA = 0
        indB = 0
    if (plot):
        plt.xlim([-1.5, 1.5])
        plt.ylim([-1.5, 1.5])
        for k in range(i):
            ax.plot([treeA[edgesA[k,0]][0],treeA[edgesA[k,1]][0]],[treeA[edgesA[k,0]][1],treeA[edgesA[k,1]][1]],[treeA[edgesA[k,0]][2],treeA[edgesA[k,1]][2]],'r')
            ax.plot([treeB[edgesB[k,0]][0],treeB[edgesB[k,1]][0]],[treeB[edgesB[k,0]][1],treeB[edgesB[k,1]][1]],[treeB[edgesB[k,0]][2],treeB[edgesB[k,1]][2]],'b')
        plt.show(block=False)

    return treeA[0:i+2,:], treeB[0:i+2,:], edgesA[0:i+1,:], edgesB[0:i+1,:], indA, indB

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

    # forceControls = np.array(([1,0],[-1,0],[0,1],[0,-1]))
    # mass = 1 #???
    # dt = .01 # seconds
    # error = 100000000

    # # choose the control that pulls the tree toward the random point
    # for force in forceControls:
    #     a = force/mass;
    #     v_new = [vx,vy] + a*dt
    #     if dir > 0:
    #         p = tree[node,0:2] + v_new*dt
    #     else:
    #         p = tree[node,0:2] - v_new*dt
    #     error_new = (p[0] - x)**2 + (p[1] - y)**2
    #     if error_new < error:
    #         error = error_new
    #         xc = p[0]
    #         yc = p[1]
    #         vc = v_new

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
    find_path(True)
    end = time.time()
    print end-start
