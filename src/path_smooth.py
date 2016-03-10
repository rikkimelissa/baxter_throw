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
    plt.close('all')
    plt.figure()
    plt.plot(traj[:,0],traj[:,1:8],'.')
    plt.show(block=False)
    plt.figure()
    plt.plot(traj[:,0],traj[:,9:])
    plt.show(block=False)
    iter = 10
    for i in range(iter):
        print i
        path_length = traj.shape[0]
        # if (path_length == 0):
        #     break
        ind1 = round(random()*(path_length-1))
        ind2 = round(random()*(path_length-1))
        if (ind1 > ind2):
            temp = ind1;
            ind1 = ind2;
            ind2 = temp
        if (ind1 != ind2):
            vertex1 = traj[ind1,:]
            vertex2 = traj[ind2,:]
            s = shortcut(vertex1,vertex2)
            # if (collision_free(s)):
            # replace segment in path
            old_dur = vertex2[0] - vertex1[0]
            new_dur = s[-1,0] - s[0,0]
            if new_dur < old_dur:
                traj = np.delete(traj,range(int(ind1),int(ind2+1)),0)
                traj[ind1:,0] += new_dur - old_dur
                traj = np.insert(traj,ind1,s,0)
    plt.figure()
    plt.plot(traj[:,0],traj[:,1:8])
    plt.show(block=False)
    plt.figure()
    plt.plot(traj[:,0],traj[:,8:])
    plt.show(block=False)


def shortcut(vertex1, vertex2):
    a_max = 4.0
    v_max = 2.0
    T = 0
    s = np.zeros((30,15))
    # plt.figure(1)
    # plt.plot(np.vstack((vertex1[1:8],vertex2[1:8])),'.')
    # plt.figure(2)
    # plt.plot(np.vstack((vertex1[8:],vertex2[8:])),'.')    
    for i in range(7):
        # print i
        if i > 3:
            v_max = 4.0
        t = execution_time(vertex1[i+1], vertex2[i+1],vertex1[i+8],vertex2[i+8],v_max,a_max)
        if t > T:
            T = t
    v_max = 2.0
    for i in range(7):
        if i > 3:
            v_max = 4.0
        time, pos, vel = traj_min_acc(vertex1[i+1], vertex2[i+1],vertex1[i+8],vertex2[i+8],v_max,T)
        s[:,i+1] = pos
        s[:,i+8] = vel
        # plt.figure()
        # plt.plot(time,pos)
        # plt.plot(time,vel)
        # plt.show(block=False)
    s[:,0] = time + vertex1[0]
    return s
    # for i in range(7):
    #     plt.figure(1)
    #     plt.plot(s[:,0], s[:,i+1])
    #     plt.figure(2)
    #     plt.plot(s[:,0], s[:,i+7])
    # plt.show(block=False)

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
        # print ("t1"), T1
    if tp2 > 0 and tp2 > (v2-v1)/a_max and v1+tp2*a_max < v_max:
        T1 = 2*tp2 + (v1-v2)/a_max 
        # print ("t1"),T1

    # P-P+
    tp1, tp2 = quad_solve(-a_max, 2*v1, (v1-v2)*(v1+v2)/(2*-a_max) + x1 - x2)
    if tp1 > 0 and tp1 > (v2-v1)/-a_max and v1+tp1*-a_max > -v_max:
        T2 = 2*tp2 + (v1-v2)/-a_max
        # print ("t2"),T2
    if tp2 > 0 and tp2 > (v2-v1)/-a_max and v1+tp2*-a_max > -v_max:
        T2 = 2*tp2 + (v1-v2)/-a_max
        # print ("t2"),T2

    # P+L+P-
    t1 = (v_max - v1)/a_max
    t2 = (v2 - v_max)/(-a_max)
    tL = (v2**2 + v1**2 - 2*v_max**2)/(2*v_max*a_max) + (x2-x1)/v_max
    if t1 > 0 and t2 > 0 and tL > 0:
        T3 = t1 + t2 + tL
        # print ("t3"),T3

    # P-L-P+
    t1 = (-v_max - v1)/-a_max
    t2 = (v2 + v_max)/(a_max)
    tL = (v2**2 + v1**2 - 2*v_max**2)/(2*v_max*a_max) + (x2-x1)/-v_max
    if t1 > 0 and t2 > 0 and tL > 0:
        T4 = t1 + t2 + tL
        # print ("t4"),T4

    if (T1 < T):
        T = T1
    if (T2 < T):
        T = T2;
    if (T3 < T):
        T = T3
    if (T4 < T):
        T = T4

    return T # returns T

def traj_min_acc(x1,x2,v1,v2,v_max,T):

    a1 = 20
    a2 = 20
    a3 = 20
    a4 = 20
    a = 20
    tSpace = np.linspace(0,T,30)

    #P+P-
    sig = 1
    ap1,ap2 = quad_solve(T**2, sig*(2*T*(v1+v2)+4*(x1-x2)),-(v1-v2)**2)
    if ap1 > 0:
        ts = 1/2.*(T+(v2-v1)/ap1)
        if ts > 0 and ts < T and v1+ap1*ts < (v_max + .05):
            a1 = ap1
            tpp1 = tSpace[tSpace < ts]
            tpp2 = tSpace[tSpace > ts]
            xp1 = a1/2*tpp1**2 + v1*tpp1 + x1
            vp1 = a1*tpp1 + v1
            xp = a1/2*ts**2 + v1*ts + x1
            vp = a1*ts + v1
            xp2 = -a1/2*(tpp2 - ts)**2 + vp*(tpp2-ts) + xp 
            vp2 = -a1*(tpp2-ts) + vp

            xt1 = np.hstack((xp1,xp2))
            tt1 = np.hstack((tpp1,tpp2))
            vt1 = np.hstack((vp1,vp2))   
    if ap2 > 0:
        ts = 1/2.*(T+(v2-v1)/ap2)
        if ts > 0 and ts < T and v1+ap2*ts < (v_max + .05):
            a1 = ap2
            tpp1 = tSpace[tSpace < ts]
            tpp2 = tSpace[tSpace > ts]
            xp1 = a1/2*tpp1**2 + v1*tpp1 + x1
            vp1 = a1*tpp1 + v1
            xp = a1/2*ts**2 + v1*ts + x1
            vp = a1*ts + v1
            xp2 = -a1/2*(tpp2 - ts)**2 + vp*(tpp2-ts) + xp 
            vp2 = -a1*(tpp2-ts) + vp

            xt1 = np.hstack((xp1,xp2))
            tt1 = np.hstack((tpp1,tpp2))
            vt1 = np.hstack((vp1,vp2))   

    #P-P+
    sig = -1
    ap1,ap2 = quad_solve(T**2, sig*(2*T*(v1+v2)+4*(x1-x2)),-(v1-v2)**2)
    if ap1 > 0:
        ts = 1/2.*(T+(v1-v2)/ap1)
        print ('P-P+'),v1-ap1*ts
        if ts > 0 and ts < T and v1-ap1*ts > (-v_max - .05):
            a2 = ap1
            tpp1 = tSpace[tSpace <= ts]
            tpp2 = tSpace[tSpace > ts]
            xp1 = -a2/2*tpp1**2 + v1*tpp1 + x1
            vp1 = -a2*tpp1 + v1
            xp = -a2/2*ts**2 + v1*ts + x1
            vp = -a2*ts + v1
            xp2 = a2/2*(tpp2 - ts)**2 + vp*(tpp2-ts) + xp 
            vp2 = a2*(tpp2-ts) + vp
            xt2 = np.hstack((xp1,xp2))
            tt2 = np.hstack((tpp1,tpp2))
            vt2 = np.hstack((vp1,vp2))   
    if ap2 > 0:
        ts = 1/2.*(T+(v1-v2)/ap2)
        print ('P-P+'), v1-ap2*ts
        if ts > 0 and ts < T and v1-ap2*ts > (-v_max - .05):
            a2 = ap2
            tpp1 = tSpace[tSpace <= ts]
            tpp2 = tSpace[tSpace > ts]
            xp1 = -a2/2*tpp1**2 + v1*tpp1 + x1
            vp1 = -a2*tpp1 + v1
            xp = -a2/2*ts**2 + v1*ts + x1
            vp = -a2*ts + v1
            xp2 = a2/2*(tpp2 - ts)**2 + vp*(tpp2-ts) + xp 
            vp2 = a2*(tpp2-ts) + vp
            xt2 = np.hstack((xp1,xp2))
            tt2 = np.hstack((tpp1,tpp2))
            vt2 = np.hstack((vp1,vp2)) 

    # P+L+P-
    a3temp = (v_max**2 - v_max*(v1+v2) + .5*(v1**2 + v2**2))/(T*v_max - (x2-x1))
    t1 = (v_max - v1)/a3temp
    t2 = (v2 - v_max)/(-a3temp)
    tL = (v2**2 + v1**2 - 2*v_max**2)/(2*v_max*a3temp) + (x2-x1)/v_max
    if t1 > 0 and t2 > 0 and tL > 0:
        a3 = a3temp
        tpp1 = tSpace[tSpace <= t1]
        tpp2 = tSpace[np.array([c and d for c,d in zip(tSpace >= t1,tSpace <= (t1+tL))])]
        tpp3 = tSpace[tSpace >= (t1+tL)]
        xp1 = a3/2*tpp1**2 + v1*tpp1 + x1
        vp1 = a3*tpp1 + v1
        xp1e = a3/2*t1**2 + v1*t1 + x1
        xp2 = v_max*(tpp2-t1)+xp1e
        xp2e = v_max*(tL) + xp1e
        vp2 = v_max*np.ones((tpp2.shape[0]))
        xp3 = -a3/2*(tpp3-tL-t1)**2 + v_max*(tpp3-tL-t1) + xp2e
        vp3 = -a3*(tpp3-tL-t1) + v_max
        xt3 = np.hstack((xp1,xp2,xp3))
        tt3 = np.hstack((tpp1,tpp2,tpp3))
        vt3 = np.hstack((vp1,vp2,vp3))

    #P-L-P+
    a4temp = (v_max**2 + v_max*(v1+v2) + .5*(v1**2 + v2**2))/(T*-v_max - (x2-x1))
    a4temp = -a4temp
    t1 = (-v_max - v1)/-a4temp
    t2 = (v2 + v_max)/(a4temp)
    tL = (v2**2 + v1**2 - 2*v_max**2)/(2*v_max*a4temp) + (x2-x1)/-v_max
    if t1 > 0 and t2 > 0 and tL > 0:
        a4 = a4temp
        tpp1 = tSpace[tSpace <= t1]
        tpp2 = tSpace[np.array([c and d for c,d in zip(tSpace >= t1,tSpace <= (t1+tL))])]
        tpp3 = tSpace[tSpace >= (t1+tL)]
        xp1 = -a4/2*tpp1**2 + v1*tpp1 + x1
        vp1 = -a4*tpp1 + v1
        xp1e = -a4/2*t1**2 + v1*t1 + x1
        xp2 = -v_max*(tpp2-t1)+xp1e
        xp2e = -v_max*(tL) + xp1e
        vp2 = -v_max*np.ones((tpp2.shape[0]))
        xp3 = a4/2*(tpp3-tL-t1)**2 + -v_max*(tpp3-tL-t1) + xp2e
        vp3 = a4*(tpp3-tL-t1) + -v_max
        xt4 = np.hstack((xp1,xp2,xp3))
        tt4 = np.hstack((tpp1,tpp2,tpp3))
        vt4 = np.hstack((vp1,vp2,vp3))

    print a1, a2, a3, a4

    if (a1 < a):
        time = tt1
        pos = xt1
        vel = vt1
        a = a1
    if (a2 < a):
        time = tt2
        pos = xt2
        vel = vt2
        a = a2
    if (a3 < a):
        time = tt3
        pos = xt3
        vel = vt3
        a = a3
    if (a4 < a):
        time = tt4
        pos = xt4
        vel = vt4
        a = a4

    # print a, T

    # if (a == 20):
    #     T += .05
    #     time, pos, vel = traj_min_acc(x1,x2,v1,v2,v_max,T)

    return time, pos, vel 
    # returns time, pos, vel paths

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