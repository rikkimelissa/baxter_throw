#!/usr/bin/env python
from random import random
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos
import time
from release_state_solve import find_feasible_release

pos_init = [2,2]
pos_goal = [95,95]
iter = 1000;

treeA = np.empty((iter+1,2))
edgesA = np.empty((iter+1,2))
treeB = np.empty((iter+1,2))
edgesB = np.empty((iter+1,2))
treeA[0] = pos_init
treeB[0] = pos_goal

def dynamic_rrt():
    # catch_x = .68
    # catch_z = -.6 # range from - .5 to 0 # lower range determined by baxter
    # catch_y = .7 # range from .29 to .5, works after .5 but hard to find solutions
    # throw_y, throw_z, vel, alpha = find_feasible_release(catch_x, catch_y, catch_z)
    # example end point
    throw_y = .20516
    throw_z = .3073
    vel = 1.03
    alpha = 1.55

def build_tree(iter,treeA, treeB ,edgesA, edgesB):
    i = 0
    vA = np.array([0,0])
    vB = np.array([0,0])
    while i < iter:
        x = random()*100
        y = random()*100
        node = nearest_neighbor(x,y, treeA, i)
        # choose_control
        treeA, vA = insert_vertex(node,treeA,x,y,i, vA, 1)
        edgesA = insert_edge(edgesA, node, i)
        # x = random()*100
        # y = random()*100
        # node = nearest_neighbor(x,y, treeB, i)
        # # choose_control
        # treeB, vB = insert_vertex(node,treeB,x,y,i, vB, -1)
        # edgesB = insert_edge(edgesB, node, i)
        # result = connect_trees(treeA, treeB, i)
        # if (result[0]):
        #     print i
        #     break
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
    # if (result[0]):
    #     indA = result[1];
    #     indB = result[2];
    #     plt.plot([treeA[indA,0], treeB[indB,0]],[treeA[indA,1], treeB[indB,1]],'g',linewidth = 3)
    plt.xlim([0,100])
    plt.ylim([0,100])
    for k in range(i-1):
        print k
        plt.plot([treeA[edgesA[k,0]][0],treeA[edgesA[k,1]][0]],[treeA[edgesA[k,0]][1],treeA[edgesA[k,1]][1]],'r')
        # plt.plot([treeB[edgesB[k,0]][0],treeB[edgesB[k,1]][0]],[treeB[edgesB[k,0]][1],treeB[edgesB[k,1]][1]],'b')
    plt.show(block=False)

def connect_trees(treeA, treeB, iter):
    for i in range(iter+1):
        for k in range(iter+1):
            if ((treeA[i,0] - treeB[k,0])**2 + (treeA[i,1] - treeB[k,1])**2) < .5:
                return True, i, k
    return False, 0, 0

def insert_vertex(node,tree,x,y,i,v,dir):
    forceControls = np.array(([1,0],[-1,0],[0,1],[0,-1]))
    mass = .1 #???
    dt = .01 # seconds
    error = 1000000
    for force in forceControls:
        a = force/mass;
        v_new = v + a*dt
        if dir > 0:
            p = tree[node] + v_new*dt
        else:
            p = tree[node] - v_new*dt
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

    tree[i+1] = [xc,yc]
    return tree, vc

def insert_edge(edges, node, i):
    edges[i] = [node, i+1]
    return edges

# def choose_control(node, x, y):
#     angle = 

def nearest_neighbor(x, y, tree, nodes):
    min_dist = 100000000
    min_node = 0
    for i in range(nodes+1):
        dist = (x - tree[i,0])**2 + (y-tree[i,1])**2
        # print i, dist
        if (dist < min_dist):
            min_dist = dist
            min_node = i
    return min_node

if __name__ == "__main__":
    start = time.time()
    build_tree(iter,treeA, treeB ,edgesA, edgesB)
    end = time.time()
    print end-start
