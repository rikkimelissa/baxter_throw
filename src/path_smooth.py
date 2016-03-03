#!/usr/bin/env python

import rospy
from rrt_joint import find_path
from functions import JointTrajectory
import numpy as np
import matplotlib.pyplot as plt
from solve_system import jointPath, jointVelocity

def main():
    path = find_path(False);
    traj = path2traj(path);
    plt.close('all')
    plt.figure()
    plt.plot(traj[:,0],traj[:,1:8])
    plt.show(block=False)
    plt.figure()
    plt.plot(traj[:,0],traj[:,9:],'b')
    plt.show(block=False)
    plt.figure()
    plt.plot(traj[:,0],np.diff(traj[:,9:]))
    plt.show(block=False)
    iter = 1
    for i in range(iter):
        x=1


# Convert vertices to time optimal trajectory by interpolating between them according to 
# velocity or acceleration limit and a third-order polynomial time scaling
def path2traj(path):
    N = path.shape[0]
    traj_comp = np.empty(((N-1)*9,15))
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

        # traj = JointTrajectory(vertex1, vertex2, T, 10, 3)
        # time = np.linspace(t_delay,t_delay + T, 10);
        ind += 1;
        t_delay += T;
    return traj_comp

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