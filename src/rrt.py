#!/usr/bin/env python
from random import random
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos

pos_init = [50,50]
iter = 1000;

tree = np.empty((iter+1,2))
edges = np.empty((iter+1,2))
tree[0] = pos_init

def build_tree(iter, tree, edges):
    for i in range(iter):
        x = random()*100
        y = random()*100
        node = nearest_neighbor(x,y, tree, i)
        # choose_control
        tree = insert_vertex(node,tree,x,y,i)
        edges = insert_edge(edges, node, i)

    plt.close('all')
    plt.figure()
    plt.plot(tree[0:i+2,0],tree[0:i+2,1],'.')
    plt.xlim([0,100])
    plt.ylim([0,100])
    for k in range(i+1):
        plt.plot([tree[edges[k,0]][0],tree[edges[k,1]][0]],[tree[edges[k,0]][1],tree[edges[k,1]][1]])
    plt.show(block=False)


def insert_vertex(node,tree,x,y,i):
    alpha = atan2(y - tree[node,1], x - tree[node,0])
    xc = 2*cos(alpha) + tree[node,0]
    yc = 2*sin(alpha) + tree[node,1]
    tree[i+1] = [xc,yc]
    return tree

def insert_edge(edges, node, i):
    edges[i] = [node, i+1]
    return edges

# def choose_control(node, x, y):
#     angle = 

def nearest_neighbor(x, y, tree, nodes):
    min_dist = 10000
    min_node = 0
    for i in range(nodes+1):
        dist = (x - tree[i,0])**2 + (y-tree[i,1])**2
        if (dist < min_dist):
            min_dist = dist
            min_node = i
    return min_node

if __name__ == "__main__":
    build_tree(1000,tree,edges)
