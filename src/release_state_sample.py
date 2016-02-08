#!/usr/bin/env python

from math import sqrt, cos, sin, tan, atan, pi, atan2
from random import random
import matplotlib.pyplot as plt

# Define range and static variables
x_range = [.6,.8] # near x position of catcher
y_range = [-.62,.24] # between start position and catch position
z_range = [.1,.4] # between catch position and high position of hand
vel_range_x = [0,.1] # need to experiment with these ranges
vel_range_y = [.3, 1.2] # needs minimum velocity for liftoff but can't be so high that JTAS will fail
vel_range_z = [0, .6]
cup_height = .12
block_height = .018
block_width = .047
g = 9.8 # m/s^2

# These are set by user
catch_x = .68
catch_z = 0
catch_y = .29 

throw_x = .65
throw_y = -.08
throw_z = .313

dx = catch_x - throw_x
dy = catch_y - throw_y - block_width
dz = catch_z + cup_height - throw_z - block_height

def plot_results():
    throw_x_list = [None]*100
    throw_y_list = [None]*100
    throw_z_list = [None]*100
    vel_list = [None]*100
    alpha_list = [None]*100
    for i in range(100):
        res = test_pos(catch_x, catch_y, catch_z)
        while res == None:
            res = test_pos(catch_x, catch_y, catch_z)
        throw_x = res[0]; throw_y = res[1]; throw_z = res[2]; vel = res[3]; alpha = res[4]
        throw_x_list[i] = throw_x;
        throw_y_list[i] = throw_y;
        throw_z_list[i] = throw_z;
        vel_list[i] = vel;
        alpha_list[i] = alpha;
    N = 100
    ind = range(N)
    width = 1
    fig, ax = plt.subplots()
    rect1 = ax.bar(ind,throw_x_list, width, color = 'r')
    # rect2 = ax.bar([x + y for x, y in zip(ind, [width]*100)],throw_y_list, width, color = 'r')
    rect2 = ax.bar([x+100 for x in ind],throw_y_list, width, color = 'y')
    rect3 = ax.bar([x+200 for x in ind],throw_z_list, width, color = 'g')
    rect4 = ax.bar([x+300 for x in ind],vel_list, width, color = 'b')
    rect5 = ax.bar([x+400 for x in ind],alpha_list, width, color = 'c')
    plt.show(block=False)

    # print "x = " + str(throw_x) + "  y = " + str(throw_y) + "  z = " + str(throw_z)
    # print "vel = " + str(vel) + "     alpha = " + str(alpha)

def test_pos(catch_x, catch_y, catch_z):
    # rand = random()
    # print "rand = " + str(rand)
    throw_x = random()*(x_range[1] - x_range[0]) + x_range[0]
    throw_y = random()*(y_range[1] - y_range[0]) + y_range[0]
    throw_z = random()*(z_range[1] - z_range[0]) + z_range[0]
    dx = catch_x - throw_x
    dy = catch_y - throw_y - block_width
    dz = catch_z + cup_height - throw_z - block_height
    rand = random()
    if find_velocity_param(dx, dy, dz, rand):
        vel, alpha = find_velocity_param(dx, dy, dz, rand)
        return [throw_x, throw_y, throw_z, vel, alpha]

def find_velocity_param(dx,dy,dz, rand):
    alpha_min = atan2(dz,dy)
    if alpha_min <= (-pi/2 + .01):
        alpha_min = pi/2 + .01
    alpha_max = 75*pi/180
    inc = 0
    alpha_list = []
    v_list = []
    for i in range(1000):
        alpha = rand*(alpha_max - alpha_min) + alpha_min
        if ((g*dy**2/(2*cos(alpha)**2*(dy*tan(alpha)-dz))) < 0):
            print rand, alpha, dy, dz
        v = sqrt(g*dy**2/(2*cos(alpha)**2*(dy*tan(alpha)-dz)))
        v_y = v*cos(alpha)
        v_z = v*sin(alpha)
        t = dy/v_y
        p_y = v_y*t
        p_z = t*v_z - .5*g*t**2
        if (v_y >= vel_range_y[0] and v_y <= vel_range_y[1] and v_z >= vel_range_z[0] and v_z <= vel_range_z[1]):
            # print inc, v_y, v_z, p_y, p_z  
            alpha_list.append(alpha)
            v_list.append(v)
            inc += 1 
            return alpha, v
    # return alpha_list, v_list
