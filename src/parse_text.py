import matplotlib.pyplot as plt
import numpy as np
from numpy import loadtxt
from solve_linear_system import linearSpace
import rospy

# rospy.init_node('test')

#ind = 661

num = 1
s = "data/pos_actual" + str(num) + ".txt"
f = open(s, "r")

lines=f.readlines()
posListAct = np.empty((len(lines)/2,7))

for i in range(len(lines)/2):
    pos = eval(lines[2*i]);
    posListAct[i,:] = pos

s = "data/pos_desired" + str(num) + ".txt"
f = open(s, "r")

lines=f.readlines()
posListDes = np.empty((len(lines)/2,7))

for i in range(len(lines)/2):
    pos = eval(lines[2*i]);
    posListDes[i,:] = pos

s = "data/time" + str(num) + ".txt"
f = open(s, "r")
lines = f.readlines()
timeList = np.empty((len(lines)/4,7))

for i in range(len(lines)/4):
    sec = eval(lines[4*i+1].split()[1])
    nsec = eval(lines[4*i+2].split()[1])
    timeList[i] = sec + nsec*1e-9

T = 1.7
N = 50*T
dt = float(T)/(N-1)
vy = .8 # nominal .8
vz = .4 # nominal .4
jerk = -5

# plt.close('all')

thList, vList = linearSpace(False, T, N, vy, vz, jerk)
t_delay = 5.0
t_all = np.linspace(0 + t_delay,2*T + t_delay, N*2);

dTime = .01

# plt.close('all')
fig = plt.figure()
fig.patch.set_facecolor('white')
l0 = plt.plot(t_all[2:],thList,'r-')
l1 = plt.plot(timeList, posListAct,'b--',label = 'Actual')
l2 = plt.plot(timeList, posListDes,'g-.',label = 'Desired')
plt.legend([l0[0], l2[0], l1[0]],["Calculated", "Commanded", "Actual"],loc=2)
plt.axis([5,8.5, -2, 2.5])
plt.title('Positions for 7 DOF')
plt.xlabel('Angle (rad)')
plt.ylabel('Time (sec)')
plt.show(block=False)

# print(np.diff(timeList,axis=0))
dtP = .01;
vList1 = np.diff(thList,axis=0)/dt
vList1 = np.vstack((np.array([0,0,0,0,0,0,0]),vList1))
vList2 = np.diff(posListAct,axis=0)/dtP
vList2 = np.vstack((np.array([0,0,0,0,0,0,0]),vList2))
vList3 = np.diff(posListDes,axis=0)/dtP
vList3 = np.vstack((np.array([0,0,0,0,0,0,0]),vList3))
fig = plt.figure()
fig.patch.set_facecolor('white')
l0 = plt.plot(t_all[2:],vList1,'-')
l1 = plt.plot(timeList, vList2,'--',label = 'Actual')
l2 = plt.plot(timeList, vList3,'-.',label = 'Desired')
plt.legend([l0[0], l2[0], l1[0]],["Calculated", "Commanded", "Actual"],loc=2)
plt.axis([5,8.5, -2, 2.5])
plt.title('Velocities for 7 DOF')
plt.ylabel('Time (sec)')
plt.show(block=False)
