#!/usr/bin/env python
from random import random
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos
import time
from release_state_solve import find_feasible_release



# def dynamic_rrt():
#     # catch_x = .68
#     # catch_z = -.6 # range from - .5 to 0 # lower range determined by baxter
#     # catch_y = .7 # range from .29 to .5, works after .5 but hard to find solutions
#     # throw_y, throw_z, vel, alpha = find_feasible_release(catch_x, catch_y, catch_z)
#     # example end point
#     throw_y = .20516
#     throw_z = .3073
#     vel = 1.03
#     alpha = 1.55

def find_path():
    pos_init = [2,2,0,0]
    pos_goal = [95,95,15,20]
    iter = 500;

    treeA = np.empty((iter+1,4))
    edgesA = np.empty((iter+1,2))
    treeB = np.empty((iter+1,4))
    edgesB = np.empty((iter+1,2))
    treeA[0] = pos_init
    treeB[0] = pos_goal
    treeA, treeB, edgesA, edgesB = build_tree(iter,treeA, treeB ,edgesA, edgesB) 

    plt.figure()
    plt.plot(treeB)
    plt.show(block=False)

    pathA = np.empty((edgesA.shape[0],4))
    length = edgesA.shape[0]
    pathA[0] = treeA[length]
    curEdge = edgesA[length-1]
    ind = 1
    atOrigin = False
    while atOrigin == False:
        pathA[ind] = treeA[curEdge[0]]
        curEdge = edgesA[curEdge[0] - 1]
        ind += 1
        if curEdge[0] == 0:
            atOrigin = True
    pathA = pathA[0:ind,:]

    plt.figure()
    plt.plot(pathA[:,0],pathA[:,1])
    plt.show(block=False)

    pathB = np.empty((edgesB.shape[0],4))
    length = edgesB.shape[0]
    pathB[0] = treeB[length]
    curEdge = edgesB[length-1]
    ind = 1
    atOrigin = False
    while atOrigin == False:
        pathB[ind] = treeB[curEdge[0]]
        curEdge = edgesB[curEdge[0] - 1]
        ind += 1
        if curEdge[0] == 0:
            atOrigin = True
    pathB = pathB[0:ind,:]

    plt.figure()
    plt.plot(pathB[:,0],pathB[:,1])
    plt.show(block=False)










def build_tree(iter,treeA, treeB ,edgesA, edgesB):
    i = 0
    while i < iter:
        x = random()*100
        y = random()*100
        vx = random()*40 - 15
        vy = random()*40 - 15
        node = nearest_neighbor(x,y,vx,vy,treeA, i)
        treeA = insert_vertex(node,treeA,x,y,vx,vy,i,1)
        edgesA = insert_edge(edgesA, node, i)

        x = random()*100
        y = random()*100
        vx = random()*40 - 15
        vy = random()*40 - 15
        node = nearest_neighbor(x,y,vx,vy,treeB, i)
        treeB = insert_vertex(node,treeB,x,y,vx,vy,i,-1)
        edgesB = insert_edge(edgesB, node, i)
        result = connect_trees(treeA, treeB, i)
        if (result[0]):
            print i
            break
        # print x, y, node, treeA[0:i+2, :], edgesA[0:i+1,:]
        # plt.plot(treeA[0:i+2,0],treeA[0:i+2,1],'.')
        # for k in range(i+1):
        #     plt.plot([treeA[edgesA[k,0]][0],treeA[edgesA[k,1]][0]],[treeA[edgesA[k,0]][1],treeA[edgesA[k,1]][1]])
        # plt.xlim([1.5, 3])
        # plt.ylim([1.5, 3])
        # plt.show(block=False)
        i = i + 1

    plt.close('all')
    plt.figure()
    plt.plot(treeA[0,0],treeA[0,1],'ro',markersize = 10)
    plt.plot(treeA[0:i+2,0],treeA[0:i+2,1],'r.')
    plt.plot(treeB[0,0],treeB[0,1],'ro',markersize = 10)
    plt.plot(treeB[0:i+2,0],treeB[0:i+2,1],'b.')
    if (result[0]):
        indA = result[1];
        indB = result[2];
        plt.plot([treeA[indA,0], treeB[indB,0]],[treeA[indA,1], treeB[indB,1]],'g',linewidth = 3)
    plt.xlim([0,100])
    plt.ylim([0,100])
    for k in range(i+1):
        plt.plot([treeA[edgesA[k,0]][0],treeA[edgesA[k,1]][0]],[treeA[edgesA[k,0]][1],treeA[edgesA[k,1]][1]],'r')
        plt.plot([treeB[edgesB[k,0]][0],treeB[edgesB[k,1]][0]],[treeB[edgesB[k,0]][1],treeB[edgesB[k,1]][1]],'b')
    plt.show(block=False)
    # plt.figure()
    # plt.plot(treeA[0,2],treeA[0,3],'ro',markersize = 10)
    # plt.plot(treeA[0:i+2,2],treeA[0:i+2,3],'r.')
    # # plt.plot(treeB[0,0],treeB[0,1],'ro',markersize = 10)
    # # plt.plot(treeB[0:i+2,0],treeB[0:i+2,1],'b.')
    # # if (result[0]):
    # #     indA = result[1];
    # #     indB = result[2];
    # #     plt.plot([treeA[indA,0], treeB[indB,0]],[treeA[indA,1], treeB[indB,1]],'g',linewidth = 3)
    # plt.xlim([-25, 25])
    # plt.ylim([-25, 25])
    # for k in range(i):
    #     plt.plot([treeA[edgesA[k,0]][2],treeA[edgesA[k,1]][2]],[treeA[edgesA[k,0]][3],treeA[edgesA[k,1]][3]],'r')
    #     # plt.plot([treeB[edgesB[k,0]][0],treeB[edgesB[k,1]][0]],[treeB[edgesB[k,0]][1],treeB[edgesB[k,1]][1]],'b')
    # plt.show(block=False)
    return treeA[0:i+2,:], treeB[0:i+2,:], edgesA[0:i+1,:], edgesB[0:i+1,:]

def connect_trees(treeA, treeB, iter):
    for i in range(iter+1):
        for k in range(iter+1):
            if ((treeA[i,0] - treeB[k,0])**2 + (treeA[i,1] - treeB[k,1])**2) < .5:
                return True, i, k
    return False, 0, 0

def insert_vertex(node,tree,x,y,vx,vy,i,dir):
    forceControls = np.array(([1,0],[-1,0],[0,1],[0,-1]))
    mass = 10 #???
    dt = .1 # seconds
    error = 1000000
    for force in forceControls:
        a = force/mass;
        v_new = [vx,vy] + a*dt
        if dir > 0:
            p = tree[node,0:2] + v_new*dt
        else:
            p = tree[node,0:2] - v_new*dt
        # print a, v_new, p
        error_new = (p[0] - x)**2 + (p[1] - y)**2
        if error_new < error:
            error = error_new
            xc = p[0]
            yc = p[1]
            vc = v_new
    # alpha = atan2(y - tree[node,1], x - tree[node,0])
    # xc = 2*cos(alpha) + tree[node,0]
    # yc = 2*sin(alpha) + tree[node,1]
    # print xc, yc, vc

    tree[i+1] = [xc,yc,vc[0],vc[1]]
    return tree

def insert_edge(edges, node, i):
    edges[i] = [node, i+1]
    return edges

# def choose_control(node, x, y):
#     angle = 

def nearest_neighbor(x, y, vx, vy, tree, nodes):
    min_dist = 100000000
    min_node = 0
    wp = .5
    wv = 1 - wp
    for i in range(nodes+1):
        distPos = (x - tree[i,0])**2 + (y-tree[i,1])**2
        distVel = (vx - tree[i,2])**2 + (vy-tree[i,3])**2
        dist = wp*distPos + wv*distVel;
        # print i, dist
        if (dist < min_dist):
            min_dist = dist
            min_node = i
    return min_node

if __name__ == "__main__":
    x = 1
    # start = time.time()
    # build_tree(iter,treeA, treeB ,edgesA, edgesB)
    # end = time.time()
    # print end-start
