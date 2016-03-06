#!/usr/bin/env python

import rospy
from rrt_joint import find_path
from functions import JointTrajectory
import numpy as np
import matplotlib.pyplot as plt
from solve_system import jointPath, jointVelocity
from random import random
from math import sqrt

def main():
    path = find_path(False);
    traj, path_orig = path2traj(path);
    # plt.close('all')
    # plt.figure()
    # plt.plot(traj[:,0],traj[:,1:8])
    # plt.hold('on')
    # plt.plot(path_orig[:,0], path_orig[:,1:8], marker='o');
    # plt.show(block=False)
    # plt.figure()
    # plt.plot(traj[:,0],traj[:,9:])
    # plt.hold('on')
    # plt.plot(path_orig[:,0], path_orig[:,9:], marker='o');
    # plt.show(block=False)
    iter = 1
    for i in range(iter):
        path_length = traj.shape[0]
        ind1 = round(random()*path_length)
        ind2 = round(random()*path_length)
        vertex1 = traj[ind1,:]
        vertex2 = traj[ind2,:]
        s = shortcut(vertex1,vertex2)
        # if (collision_free(s)):
        #     traj = np.delete(traj,range(ind1+1,ind2),0)
        #     traj = np.insert(traj,ind1+1,s,0)

def shortcut(vertex1, vertex2):
    a_max = 4.0
    v_max = 2.0
    for i in range(7):
        if i > 3:
            v_max = 4
        time = execution_time(vertex1[i+1], vertex2[i+1],vertex1[i+8],vertex2[i+8],v_max,a_max)


def execution_time(x1,x2,v1,v2,v_max,a_max):

    T1 = 20
    T2 = 20
    T3 = 20
    T4 = 20
    T = 20

    # P+P-
    tp1, tp2 = quad_solve(a_max, 2*v1, (v1-v2)*(v1+v2)/(2*a_max) + x1 - x2)
    if tp1 >= 0 and tp1 > (v2-v1)/a_max and v1+tp1*a_max < v_max:
        T1 = 2*tp1 + (v1-v2)/a_max 
        print T1
    if tp2 > 0 and tp2 > (v2-v1)/a_max and v1+tp2*a_max < v_max:
        T1 = 2*tp2 + (v1-v2)/a_max 
        print ("t1"),T1

    # P-P+
    tp1, tp2 = quad_solve(-a_max, 2*v1, (v1-v2)*(v1+v2)/(2*-a_max) + x1 - x2)
    if tp1 > 0 and tp1 > (v2-v1)/-a_max and v1+tp1*-a_max > -v_max:
        T2 = 2*tp2 + (v1-v2)/-a_max
        print T2
    if tp2 > 0 and tp2 > (v2-v1)/-a_max and v1+tp2*-a_max > -v_max:
        T2 = 2*tp2 + (v1-v2)/-a_max
        print ("t2"),T2

    # P+L+P-
    t1 = (v_max - v1)/a_max
    t2 = (v2 - v_max)/(-a_max)
    xp1 = a_max/2*t1**2 + v1*t1 + x1
    xp2 = x2 - a_max/2*t2**2 - v_max*t2
    tl = (xp2 - xp1)/v_max
    if t1 > 0 and t2 > 0 and tl > 0:
        T3 = t1 + t2 + tL
        print ("t3"),T3

    # P-L-P+
    t1 = (-v_max - v1)/-a_max
    t2 = (v2 + v_max)/(a_max)
    xp1 = -a_max/2*t1**2 + v1*t1 + x1
    xp2 = x2 + a_max/2*t2**2 - v_max*t2
    tl = (xp2 - xp1)/-v_max
    if t1 > 0 and t2 > 0 and tl > 0:
        T4 = t1 + t2 + tl
        print ("t4"), T4

    if (T1 < T):
        T = T1
    if (T2 < T):
        T = T2;
    if (T3 < T):
        T = T3
    if (T4 < T):
        T = T4



def quad_solve(a,b,c):
    if b**2 - 4*a*c > 0:
        sol1 = (-b + sqrt(b**2 - 4*a*c))/(2*a)
        sol2 = (-b - sqrt(b**2 - 4*a*c))/(2*a)
        return sol1, sol2
    else:
        return -1, -1

# Convert vertices to time optimal trajectory by interpolating between them according to 
# velocity or acceleration limit and a third-order polynomial time scaling
def path2traj(path):
    N = path.shape[0]
    traj_comp = np.empty(((N-1)*9,15))
    path_orig = np.empty((N-1,15))
    ind = 0;
    t_delay = 0;
    for vertex1, vertex2 in zip(path[:-1,:],path[1:,:]):
        T = 0;
        # Find shortest time so that vel/acc limits are met
        for i in range(7):
            a_max = 4
            if i < 4:
                v_max = 2
            else:
                v_max = 4
            p1 = vertex1[i];
            p2 = vertex2[i];
            v1 = vertex1[i+7];
            v2 = vertex2[i+7];
            T1 = abs(p2-p1)/v_max
            T2 = abs(v2-v1)/a_max
            if max(T1,T2) > T:
                T = max(T1,T2)


        path_orig[ind, 1:] = vertex1;
        # solve system so that vel and position state are met at end
        # lim = False
        i = 0
        while i < 6:
            tSpace = np.linspace(0,T,10)
            a = np.array([[1,0,0,0,0,0],[1,T,T**2,T**3,T**4,T**5],[0,1,0,0,0,0],[0,1,2*T,3*T**2,4*T**3,5*T**4],
            [0,0,2,0,0,0],[0,0,2,6*T,12*T**2,20*T**3]])
            colors = ['r','b','c','y','m','oc','k']
 
            for i in range(7):
                # print i
                b = np.array([vertex1[i],vertex2[i],vertex1[i+7],vertex2[i+7],0,0])
                coeff = np.linalg.solve(a,b)
                jP = jointPath(tSpace,coeff)
                jV = jointVelocity(tSpace,coeff)

                if i < 4:
                    v_max = 2
                else:
                    v_max = 4
                if abs(jV).max() > v_max or abs(np.diff(jV)).max()*(tSpace[1]-tSpace[0]) > a_max:
                    # print ('increasing T')
                    T *= 1.5                    
                    break;
                traj_comp[9*ind:9*ind+9,i+1] = jP[:-1]   
                traj_comp[9*ind:9*ind+9,i+8] = jV[:-1]  
                # print ('Met')


 

                # color = colors[i]
                # plt.figure(1)
                # plt.hold(True)
                # plt.plot(tSpace,jP,'-'+color,label='Joint '+str(i))     
                # plt.figure(2)
                # plt.hold(True)
                # plt.plot(tSpace,jV,'-'+color,label='Joint '+str(i))
                # plt.show(block=False)

        traj_comp[9*ind:9*ind+9,0] = tSpace[:-1] + t_delay
        path_orig[ind,0] = t_delay    

        # traj = JointTrajectory(vertex1, vertex2, T, 10, 3)
        # time = np.linspace(t_delay,t_delay + T, 10);
        ind += 1;
        t_delay += T;
    return traj_comp, path_orig

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# Step 3-5: Selects two random states
# Step 6: Computes time-optimal interpolant, IV-B
# Step 7: Tests for collisions, IV-E
# Step 8: Path splicing

# f(x,v,vmax, amax) = time of the interpolant between x1 and x2
# g(x,v,vmax,T,t) = state at time t of T

# 1) compute optimal time = max of times for joints. For each joint, min of 4 primitives
# 2) Find acceleration of each trajectory
# 3) Pick min-acceleration trajectory