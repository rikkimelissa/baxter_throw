#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

def jointPath(t,coeff):
	a0 = coeff[0]
	a1 = coeff[1]
	a2 = coeff[2]
	a3 = coeff[3]
	a4 = coeff[4]
	a5 = coeff[5]
	s = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
	return s

def jointVelocity(t,coeff):
	a0 = coeff[0]
	a1 = coeff[1]
	a2 = coeff[2]
	a3 = coeff[3]
	a4 = coeff[4]
	a5 = coeff[5]
	v = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
	return v

def jointAcceleration(t,coeff):
	a0 = coeff[0]
	a1 = coeff[1]
	a2 = coeff[2]
	a3 = coeff[3]
	a4 = coeff[4]
	a5 = coeff[5]
	a = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
	return a

q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
 -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
 -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
q_dot = np.array([-0.23825794, -0.13400971,  0.04931685,  0.0264105 , -0.8301056 , 0.28080345,  0.39270727])
q_end = np.array([0.9085001216251363,  -1.0089758632316308, 0.07401457301547121, 1.8768254939778037,
 0.18599517053110642, -0.8172282647459542, -0.44600491407768406])

colors = ['r','b','c','y','m','oc','k']
plt.close('all')

T = 4
tSpace = np.linspace(0,T,101);
tOffset = T
a = np.array([[1,0,0,0,0,0],[1,1*T,1*T**2,1*T**3,1*T**4,1*T**5],[0,1,0,0,0,0],[0,1,2*T,3*T**2,4*T**3,5*T**4],[0,0,2,0,0,0],[0,0,2,6*T,12*T**2,20*T**3]])

for i in range(7):

	b = np.array([q_start[i],q_throw[i],0,q_dot[i],0,0])
	coeff = np.linalg.solve(a,b)
	j1Pa = jointPath(tSpace,coeff)
	j1Va = jointVelocity(tSpace,coeff)
	j1Aa = jointAcceleration(tSpace,coeff)
	b = np.array([q_throw[i],q_end[i],q_dot[i],0,0,0])
	coeff = np.linalg.solve(a,b)
	j1Pb = jointPath(tSpace,coeff)
	j1Vb = jointVelocity(tSpace,coeff)
	j1Ab = jointAcceleration(tSpace,coeff)

	color = colors[i]
	plt.figure(1)
	plt.hold(True)
	plt.plot(tSpace,j1Pa,'-'+color,label='Joint '+str(i))
	plt.plot(tSpace+tOffset,j1Pb,'--'+color)
	plt.title('Path')
	plt.legend(loc='upper left')
	# plt.show(block = False)

	plt.figure(2)
	plt.plot(tSpace,j1Va,'-'+color,label='Joint '+str(i))
	plt.plot(tSpace+tOffset,j1Vb,'--'+color)
	plt.title('Velocity')
	plt.legend(loc='upper left')
	# plt.show(block = False)

	plt.figure(3)
	plt.hold(True)
	plt.plot(tSpace,j1Aa,'-'+color,label='Joint '+str(i))
	plt.plot(tSpace+tOffset,j1Ab,'--'+color)
	plt.title('Acceleration')
	plt.legend(loc='upper left')

	# plt.show(block = False)

plt.show(block=False)

