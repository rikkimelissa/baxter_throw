#!/usr/bin/env python

from math import sqrt, cos, sin, tan, atan, pi
from random import random

# Define range and static variables
x_range = [.6,.8] # near x position of catcher
y_range = [-.62,.29] # between start position and catch position
z_range = [0,.4] # between catch position and high position of hand
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
    throw_x, throw_y, throw_z, vel, alpha = test_pos(catch_x, catch_y, catch_z)

def test_pos(catch_x, catch_y, catch_z):
    # rand = random()
    # print "rand = " + str(rand)
    throw_x = random()*(x_range[1] - x_range[0]) + x_range[0]
    throw_y = random()*(y_range[1] - y_range[0]) + y_range[0]
    throw_z = random()*(z_range[1] - z_range[0]) + z_range[0]
    dx = catch_x - throw_x
    dy = catch_y - throw_y - block_width
    dz = catch_z + cup_height - throw_z - block_height
    if find_velocity_param(dx, dy, dz):
        vel, alpha = find_velocity_param(dx, dy, dz)
        # print "x = " + str(throw_x) + "y = " + str(throw_y) + "z = " + str(throw_z)
        # print "vel = " + str(vel) + "     alpha = " + str(alpha)
        return throw_x, throw_y, throw_z, vel, alpha

def find_velocity_param(dx,dy,dz):
    alpha_min = atan(dz/dy)
    alpha_max = pi/2
    inc = 0
    alpha_list = []
    v_list = []
    for i in range(1000):
        alpha = random()*(alpha_max - alpha_min) + alpha_min
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
