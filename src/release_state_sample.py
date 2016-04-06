#!/usr/bin/env python

from math import sqrt, cos, sin, tan, atan, pi, atan2
from random import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define static variables
cup_height = .12
block_height = .018
block_width = .047
g = 9.8 # m/s^2

def plot_results():
    throw_y_list = [None]*100
    throw_z_list = [None]*100
    vel_list = [None]*100
    alpha_list = [None]*100
    for i in range(100):
        res = test_pos(catch_x, catch_y, catch_z)
        while res == None:
            res = test_pos(catch_x, catch_y, catch_z)
        throw_y = res[0]; throw_z = res[1]; vel = res[2]; alpha = res[3]
        throw_y_list[i] = throw_y;
        throw_z_list[i] = throw_z;
        vel_list[i] = vel;
        alpha_list[i] = alpha;
    N = 100
    ind = range(N)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(throw_y_list, throw_z_list, vel_list)
    plt.show(block=False)
    ax.set_xlabel('Y position (m)')
    ax.set_ylabel('Z position (m)')
    ax.set_zlabel('Velocity (m/s^2)')
    title = "Throw position vs throw velocity for catch position of " + str(catch_y) + "," + str(catch_z)
    ax.set_title(title)
    plt.show(block=False)

def test_pos(catch_x, catch_y, catch_z, pos):
    if pos == 3:
        x_range = [.6,.8] # near x position of catcher
        y_range = [-.1,0] # between start position and catch position
        z_range = [.2,.3] # between catch position and high position of hand
    elif pos == 1:
        # print 1
        x_range = [.6,.8] # near x position of catcher
        y_range = [-.15,0] # between start position and catch position
        z_range = [.3,.4] # between catch position and high position of hand
    if pos == 2:
        # print 2
        x_range = [.6,.8] # near x position of catcher
        y_range = [-.3,0] # between start position and catch position
        z_range = [.25,.4] # between catch position and high position of hand

    throw_x = random()*(x_range[1] - x_range[0]) + x_range[0]
    throw_y = random()*(y_range[1] - y_range[0]) + y_range[0]
    throw_z = random()*(z_range[1] - z_range[0]) + z_range[0]
    dx = catch_x - throw_x
    dy = catch_y - throw_y - block_width
    dz = catch_z + cup_height - throw_z - block_height
    rand = random()
    if find_velocity_param(dx, dy, dz, rand, pos):
        vel, alpha = find_velocity_param(dx, dy, dz, rand, pos)
        return [throw_y, throw_z, vel, alpha]

def find_velocity_param(dx,dy,dz, rand, pos):
    if pos == 3:
        # print 3
        vel_range_x = [0,.1] # need to experiment with these ranges
        vel_range_y = [.6, 1] # needs minimum velocity for liftoff but can't be so high that JTAS will fail
        vel_range_z = [0, .5]
        alpha_max = 10*pi/180
    elif pos == 1:
        # print 1
        vel_range_x = [0,.1] # need to experiment with these ranges
        vel_range_y = [.3, 1.5] # needs minimum velocity for liftoff but can't be so high that JTAS will fail
        vel_range_z = [0, 1.5]
        alpha_max = 25*pi/180
    elif pos == 2:
        # print 2
        vel_range_x = [0,.1] # need to experiment with these ranges
        vel_range_y = [.3, 1.5] # needs minimum velocity for liftoff but can't be so high that JTAS will fail
        vel_range_z = [0, 1.5]
        alpha_max = 10*pi/180

    alpha_min = atan2(dz,dy)
    if alpha_min <= (-pi/2 + .01):
        alpha_min = -pi/2 + .01
    inc = 0
    alpha_list = []
    v_list = []
    for i in range(1000):
        alpha = rand*(alpha_max - alpha_min) + alpha_min
        if ((g*dy**2/(2*cos(alpha)**2*(dy*tan(alpha)-dz))) < 0):
            x = 1;
            print rand, alpha, dy, dz
        v = sqrt(g*dy**2/(2*cos(alpha)**2*(dy*tan(alpha)-dz)))
        v_y = v*cos(alpha)
        v_z = v*sin(alpha)
        t = dy/v_y
        p_y = v_y*t
        p_z = t*v_z - .5*g*t**2
        if (v_y >= vel_range_y[0] and v_y <= vel_range_y[1] and v_z >= vel_range_z[0] and v_z <= vel_range_z[1]):
            alpha_list.append(alpha)
            v_list.append(v)
            inc += 1 
            return v, alpha

if __name__ == '__main__':
    plot_results()