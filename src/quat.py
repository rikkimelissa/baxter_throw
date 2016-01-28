#!/usr/bin/env python
import numpy as np
import tf.transformations as tr
from math import pi
from random import gauss

# Written by Jarvis Schultz

ERRRED = '\033[91m'
ENDC = '\033[0m'

def matmult(*x):
  return reduce(np.dot, x)


def test_quat_props(Q, verbose=False):
    if verbose: print "Testing quaternion dimensions..."
    if len(np.ravel(Q)) is 4:
        if verbose: print "Passed"
    else:
        print ERRRED+"Failed"+ENDC
    if verbose: print "Testing quaternion norm...."
    if np.abs(np.linalg.norm(Q) - 1) < 1e-12:
        if verbose: print "Passed"
    else:
        print ERRRED+"Failed"+ENDC


def test_so3_props(R, verbose=False):
    R = np.array(R)
    if verbose: print "Testing SO(3) dimensions..."
    if R.shape[0] is 3 and R.shape[1] is 3:
        if verbose: print "Passed"
    else:
        print ERRRED+"Failed"+ENDC
    if verbose: print "Testing SO(3) inverse...."
    if np.max(matmult(R.transpose(),R)-np.eye(3)) < 1e-12:
        if verbose: print "Passed"
    else:
        print ERRRED+"Failed"+ENDC
    if verbose: print "Testing SO(3) determinate...."
    if np.linalg.det(R) - 1 < 1e-12:
        if verbose: print "Passed"
    else:
        print ERRRED+"Failed"+ENDC


def axis_angle_to_quat(w, th):
    return np.array(np.hstack((np.cos(th/2.0), np.array(w)*np.sin(th/2.0))))


def quat_to_axis_angle(Q):
    th = 2*np.arccos(Q[0])
    if np.abs(th) < 1e-12:
        w = np.zeros(3)
    else:
        w = Q[1:]/np.sin(th/2.0)
    return w, th


def hat(w):
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]])


def unhat(what):
    return np.array([-what[1,2], what[0,2], -what[0,1]])


def axis_angle_to_so3(w, th):
    return np.eye(3) + matmult(hat(w),np.sin(th)) + matmult(hat(w),hat(w))*(1-np.cos(th))


def so3_to_axis_angle(R):
    th = np.arccos((R.trace() - 1)/2)
    w = 1/(2*np.sin(th))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    if any(map(np.isinf, w)) or any(map(np.isnan, w)):
        th = 0
        w = 3*[1/np.sqrt(3)]
    return w, th


def quat_to_so3(Q):
    w, th = quat_to_axis_angle(Q)
    return axis_angle_to_so3(w, th)


def so3_to_quat(R):
    w,th = so3_to_axis_angle(R)
    return axis_angle_to_quat(w, th)


def quat_mult(Q,P):
    q0 = Q[0]
    p0 = P[0]
    q = Q[1:]
    p = P[1:]
    return np.hstack((q0*p0-np.dot(q,p), q0*p + p0*q + np.cross(q,p)))


def quat_conj(Q):
    return np.hstack((Q[0], -1*Q[1:]))


def quat_norm(Q):
    return np.linalg.norm(Q)


def quat_inv(Q):
    return quat_conj(Q)/quat_norm(Q)**2.0


def quat_transform(Q, p):
    pq = np.hstack((0,p))
    return quat_mult(quat_mult(Q,pq),quat_inv(Q))[1:]


def make_rand_unit_vector(dims):
    vec = [gauss(0, 1) for i in range(dims)]
    mag = sum(x**2 for x in vec) ** .5
    return [x/mag for x in vec]


def run_example_transforms():
    # create a vector in the "a" frame
    pa = [1/np.sqrt(2),1/np.sqrt(2),0]
    # create a rotation matrix from the "a" to the "b" frame (rotate pi/4 around z-axis):
    Rab = axis_angle_to_so3([0,0,1], pi/4.0)
    # get original vector expressed in the "b" frame:
    pb = matmult(Rab.T,pa)
    # create a quaternion for the same rotation:
    Qab = so3_to_quat(Rab)
    # transform using quaternion:
    pb_q = quat_transform(quat_inv(Qab), pa)
    # print results:
    print "Original Vector pa =",pa
    print "pb = Rba*pa =",pb
    print "Pbq = Qab*Pa*Qab^* =",pb_q
    print "Rab =",Rab
    print "Qab =",Qab
    return


def perform_random_axis_and_angle_tests():
    for i in range(100):
        # generate random axis:
        w = make_rand_unit_vector(3)
        pa = 20*np.random.random_sample((3,)) - 10
        for th in np.linspace(0, 2*pi, 30):
            Rab = axis_angle_to_so3(w, th)
            test_so3_props(Rab)
            pb = matmult(Rab.T, pa)
            Qab = axis_angle_to_quat(w, th)
            test_quat_props(Qab)
            pb_q = quat_transform(quat_inv(Qab), pa)
            if np.max(np.abs(pb-pb_q)) > 1e-12:
                print ERRRED+"[ERROR]"+ENDC
                print "pa = ",pa
                print "w = ",w
                print "th = ",th,"\r\n"
                break


if __name__ == '__main__':
    run_example_transforms()
    perform_random_axis_and_angle_tests()
