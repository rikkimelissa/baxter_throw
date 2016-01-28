#!/usr/bin/env python

import numpy as np
from math import sin, cos, pi, tan, acos
from numpy.linalg import inv

'''
magnitude(x) takes in a numpy array and returns the scalar magnitude
Example usage:
x = np.array([3,-1,2])
magnitude(x)
Out: 3.7416573867739413
'''
def magnitude(x):
	return np.linalg.norm(x)

'''
normalize(x) takes in a numpy array and returns a unit numpy array
Example usage:
x = np.array([3,-1,2])
normalize(x)
Out[185]: array([ 0.80178373, -0.26726124,  0.53452248])
'''
def normalize(x):
	mag = magnitude(x)
	if mag==0:
		return x
	return x/mag

''' 
RotInv(x) takes in a rotation array in the form of a numpy array and returns its inverse
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
RotInv(R)
Out[187]: 
array([[ 1,  0,  0],
       [ 0,  0,  1],
       [ 0, -1,  0]])
'''
def RotInv(x):
	return x.T

'''
VecToso3(x) takes in a numpy array representing an angular velocity and returns the 3x3 skew-symmetric array
Example usage:
w = np.array([3,-1,2])
VecToso3(w)
Out[189]: 
array([[ 0, -2, -1],
       [ 2,  0, -3],
       [ 1,  3,  0]])
'''
def VecToso3(x):
	w1 = x[0]
	w2 = x[1]
	w3 = x[2]
	return np.array([[0, -w3, w2], [w3, 0,-w1], [-w2, w1, 0]])

''' 
so3ToVec(x) takes in a 3x3 skew-symmetric array of the form numpy array and returns the corresponding 3-vecotr as a numpy array
Example usage:
w_hat = np.array([[0,-2,-1],[2,0,-3],[1,3,0]])
so3ToVec(w_hat)
Out[193]: array([ 3, -1,  2])
'''
def so3ToVec(x):
	return np.array([x[2,1], x[0,2], x[1,0]])
	
''' 
AxisAng3(x) takes in a numpy array of the rotation vector and returns the unit rotation axis w and the rotation amount theta
Example usage:
w=np.array([0,0,1])
omega,theta = AxisAng3(w*pi/2)
omega
Out[203]: array([ 0.,  0.,  1.])
theta
Out[204]: 1.5707963267948966
'''
def AxisAng3(x):
    if np.all(x==0):
        theta = 0
        omega = x*0
    else:
	    theta = magnitude(x)
	    omega = x/theta
    return omega,theta

''' 
MatrixExp3(x) takes in a numpy array of exponential coordinates and returns the matrix exponential
Example usage:
w=np.array([0,0,1])
MatrixExp3(w*pi/2)
Out[242]: 
array([[  6.123e-17,  -1.000e+00,   0.000e+00],
       [  1.000e+00,   6.123e-17,   0.000e+00],
       [  0.000e+00,   0.000e+00,   1.000e+00]])
'''
def MatrixExp3(x):
	omega,th = AxisAng3(x)
	w1 = omega[0]
	w2 = omega[1]
	w3 = omega[2]
	return np.array([[cos(th)+w1**2*(1-cos(th)), w1*w2*(1-cos(th))-w3*sin(th), w1*w3*(1-cos(th))+w2*sin(th)], [w1*w2*(1-cos(th))+w3*sin(th), cos(th)+w2**2*(1-cos(th)), w2*w3*(1-cos(th))-w1*sin(th)], [w1*w3*(1-cos(th))-w2*sin(th), w2*w3*(1-cos(th))+w1*sin(th), cos(th)+w3**2*(1-cos(th))]])

''' 
MatrixLog3(x) takes in a matrix exponential of the form numpy array and returns a rotation vector as a numpy array
Example usage:
w = np.array([0,0,1])
a = MatrixExp3(w*pi/2)
 MatrixLog3(a)
Out[259]: array([ 0.   ,  0.   ,  1.571])
'''
def MatrixLog3(x):
    th = acos((x.trace()-1)/2)
    if (abs(sin(th))<.005):
        if (np.array_equal(x,np.identity(3))):
            th = 0
            w1=w2=w3 = 0
        else:
            th = pi
            w1 = 1/sqrt(2*(1+x[2,2]))*x[0,2]
            w2 = 1/sqrt(2*(1+x[2,2]))*x[1,2]
            w3 = 1/sqrt(2*(1+x[2,2]))*(1+x[2,2])
    else:
        w1 = 1/(2*sin(th))*(x[2,1] - x[1,2])
        w2 = 1/(2*sin(th))*(x[0,2] - x[2,0])
        w3 = 1/(2*sin(th))*(x[1,0] - x[0,1])
    return th*np.array([w1, w2, w3])

''' 
RpToTrans(rot,pos) takes in a rotation matrix as a numpy array and a postion vector as a numpy array and returns a transformation matrix as a numpy array
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
RpToTrans(rot,pos)
Out[262]: 
array([[ 1,  0,  0, -3],
       [ 0,  0, -1,  1],
       [ 0,  1,  0,  2],
       [ 0,  0,  0,  1]])
'''
def RpToTrans(rot,pos):
    trans = np.concatenate((rot,pos[np.newaxis].T), axis=1)
    c = ([[0,0,0,1]])
    trans = np.concatenate((trans,c),axis=0)
    return np.array(trans)
	
''' 
TransToRp(x) takes in a transformation matrix and returns a rotation matrix and a position vector as numpy arrays
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
T = RpToTrans(rot,pos)
TransToRp(T)
Out[264]: 
(array([[ 1,  0,  0],
       [ 0,  0, -1],
       [ 0,  1,  0]]),
 array([-3,  1,  2]))
'''
def TransToRp(x):
    rot = x[0:3,0:3]
    pos = x[0:3,3]
    return rot, pos
    
''' 
TransInv(x) takes a transformation matrix as a numpy array and returns its inverse
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
T = RpToTrans(rot,pos)
TransInv(T)
Out[267]: 
array([[ 1,  0,  0,  3],
       [ 0,  0,  1, -2],
       [ 0, -1,  0,  1],
       [ 0,  0,  0,  1]])
'''
def TransInv(x):
    rot, pos = TransToRp(x)
    rot_inv = rot.T
    pos_inv = -rot_inv.dot(pos)
    return RpToTrans(rot_inv, pos_inv)

''' 
VecTose3(x) takes in a spatial velocity in the form of 6x1 numpy array and returns the corresponding 4x4 matrix in the form of a numpy array
Example usage:
S = np.array([0,0,-1,-3,0,-.1])
VecTose3(S)
array([[ 0. ,  1. ,  0. , -3. ],
       [-1. ,  0. , -0. ,  0. ],
       [-0. ,  0. ,  0. , -0.1],
       [ 0. ,  0. ,  0. ,  1. ]])
'''
def VecTose3(x):
    w_mat = VecToso3(x[0:3]);
    pos = x[3:6]
    return RpToTrans(w_mat, pos)

''' 
se3ToVec(x) takes in an element of se(3) as a numpy array and returns the corresponding spatial velocity as a 6-element numpy array
Example usage:
S = np.array([0,0,-1,-3,0,-.1])
s = VecTose3(S)
se3ToVec(s)
Out[273]: array([ 0. ,  0. , -1. , -3. ,  0. , -0.1])
'''    
def se3ToVec(x):
    w_mat, vel = TransToRp(x)
    w = so3ToVec(w_mat)
    return np.concatenate((w,vel),axis=1)

''' 
Adjoint(x) takes in a transformation matrix as a numpy array and returns the 6x6 adjoint representation as a numpy array
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
T = RpToTrans(rot,pos)
Adjoint(T)
Out[274]: 
array([[ 1.,  0.,  0.,  0.,  0.,  0.],
       [ 0.,  0., -1.,  0.,  0.,  0.],
       [ 0.,  1.,  0.,  0.,  0.,  0.],
       [ 0.,  1.,  2.,  1.,  0.,  0.],
       [ 2.,  3.,  0.,  0.,  0., -1.],
       [-1.,  0.,  3.,  0.,  1.,  0.]])
'''
def Adjoint(x):
    rot, pos = TransToRp(x)
    top = np.concatenate((rot,np.zeros((3,3))),axis = 1)
    p = VecToso3(pos)
    corner = p.dot(rot)
    bot = np.concatenate((corner,rot),axis = 1)
    return np.concatenate((top,bot))
    
''' 
ScrewToAxis(q,s,h) takes in a point q in R3, a unit axis s in R3 and a screw pitch and returns the screw axis as a numpy array
Example usage:
q=np.array([3,0,0])
s=np.array([0,0,1])
h=2
ScrewToAxis(q,s,h)
Out[279]: array([ 0,  0,  1,  0, -3,  2])
'''
def ScrewToAxis(q, s, h):
    v = np.cross(-s,q) + h*s
    return np.concatenate((s,v),axis=1)   
     

''' 
AxisAng6(x) takes a 6 vector of exponential coordinates for rigid body motion and returns the screw axis, S, and the distance traveled, theta
Example usage:
S = np.array([0, 0, 1, 2, 0, 0])
AxisAng6(S*pi/2)
Out[310]: (array([ 0.,  0.,  1.,  2.,  0.,  0.]), 1.5707963267948966)
'''
def AxisAng6(x):
    omega,theta = AxisAng3(x[0:3])
    if theta==0:
        theta=magnitude(x[3:6])
        if theta==0:
            v = x[3:6]
        else:
            v = x[3:6]/theta
    else:
        v = x[3:6]/theta
    return np.concatenate((omega,v),axis=1), theta 

''' 
MatrixExp6(x) takes a 6 vector of exponential coordinates and returns T' that is achieved by traveling along S a distance theta
Example usage:
S=np.array([0,1/sqrt(2),1/sqrt(2),1,2,3])
theta = 1
MatrixExp6(S*theta)
Out[282]: 
array([[  1.1e-16,   1.0e+00,   0.0e+00,  -3.0e+00],
       [ -1.0e+00,   1.1e-16,   0.0e+00,   3.0e+00],
       [  0.0e+00,   0.0e+00,   1.0e+00,  -1.6e-01],
       [  0.0e+00,   0.0e+00,   0.0e+00,   1.0e+00]])
'''
def MatrixExp6(x):
    S, th = AxisAng6(x)
    omega = S[0:3]
    v = S[3:6]
    omega_mat = VecToso3(omega)
#    if th==0:
#        v = x[3:6]/magnitude(x[3:6])
#    else:
#        v = x[3:6]/th
    I = np.identity(3)
    w1 = omega[0]
    w2 = omega[1]
    w3 = omega[2]
    eot = e_omega_theta(omega_mat,th)
    return e_s_theta(eot, omega_mat,th,v)
    
'''
e_omega_theta(omega_mat, th) takes in a skew symmetrix matrix and a distance traveled, theta, and returns the matrix exponential for rotations
Example usage:
omega_mat = np.array([[0,-2,-1],[2,0,-3],[1,3,0]])
th = pi/2
e_omega_theta(omega_mat, th)
Out[314]: 
array([[ -4.,  -5.,   5.],
       [ -1., -12.,  -5.],
       [  7.,   1.,  -9.]])
'''
def e_omega_theta(omega_mat,th):
    I = np.identity(3)
    return I+sin(th)*omega_mat + omega_mat.dot(omega_mat)*(1-cos(th))

'''
e_s_theta(eot, omega_mat, th) takes in a matrix exponential for rotations, a skew symmetrix matrix, a distance traveled theta, and a velocity vector, and returns the matrix exponential for transformations
Example usage:
omega_mat = np.array([[0,-2,-1],[2,0,-3],[1,3,0]])
th = pi/2
eot = e_omega_theta(omega_mat, th)
v = np.array([2, 0, 0])
e_s_theta(eot, omega_mat,th,v)
Out[321]: 
array([[ -4. ,  -5. ,   5. ,  -2.6],
       [ -1. , -12. ,  -5. ,   0.6],
       [  7. ,   1. ,  -9. ,   8.8],
       [  0. ,   0. ,   0. ,   1. ]])
'''
def e_s_theta(eot, omega_mat,th,v):
    I = np.identity(3)
    pos_no_v = (I*th + (1-cos(th))*omega_mat + (th-sin(th))*omega_mat.dot(omega_mat))
    pos = pos_no_v.dot(v)
    top = np.concatenate((eot,pos[np.newaxis].T),axis=1)
    bot = np.array([[0,0,0,1]])
    return np.concatenate((top,bot))
    

''' 
MatrixLog6(x) takes a transformation matrix and returns the corresponding 6-vector of exponential coordinates 
Example usage:  
T = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,3],[0,0,0,1]]) 
MatrixLog6(T)
Out[284]: array([ 1.6,  0. ,  0. ,  0. ,  2.4,  2.4])
'''
def MatrixLog6(x):
    rot, pos = TransToRp(x)
    I = np.identity(3)
    if magnitude(I-rot) < .05:
        omega = np.array([0,0,0])
        v = pos/magnitude(pos)
        theta = magnitude(pos)
    elif (np.trace(rot)==-1):
        theta = pi
        omega = MatrixLog3(rot)
        omega_mat = VecToso3(omega)
        G_inv = (1/theta)*I - .5*omega_mat + (1/theta-.5/tan(theta/2))*omega_mat.dot(omega_mat)
        v = G_inv.dot(pos)
    else:
        theta = acos((np.trace(rot)-1)/2)
        omega_mat = 1/(2*sin(theta))*(rot-rot.T)
        omega = so3ToVec(omega_mat)
        G_inv = (1/theta)*I - .5*omega_mat + (1/theta-.5/tan(theta/2))*omega_mat.dot(omega_mat)
        v = G_inv.dot(pos)                
    return np.concatenate((omega,v),axis=1)*theta

''' 
FKinFixed takes M, the position and orientation of the end-effector frame when the manipulator is at its home position, a list of screw axes in the form of a numpy array matrix expressed in world frame, and a list of joint coordinates as a numpy array, and returns the transformation of the end-effector frame at the given joint angles
Example usage:
L0 = 2
L1 = 1
L2 = 1
L3 = .5
M = np.array([[-1,0,0,0],[0,1,0,L0+L2],[0,0,-1,L1-L3],[0,0,0,1]])
S1 = np.array([0,0,1,L0,0,0])
S2 = np.array([0,0,0,0,1,0])
S3 = np.array([0,0,-1,-L0-L2,0,-.1])
screw_axes = np.vstack((S1,S2,S3))
joints = ([pi/2,3,pi])
np.around(FKinFixed(M,screw_axes,joints),decimals=3)
Out[306]: 
array([[-0. ,  1. ,  0. , -1. ],
       [ 1. ,  0. ,  0. ,  2. ],
       [ 0. ,  0. , -1. ,  0.2],
       [ 0. ,  0. ,  0. ,  1. ]])
'''
def FKinFixed(M, screw_axes, joints):
    T = MatrixExp6(screw_axes[0]*joints[0])
    for S, theta in zip(screw_axes[1:],joints[1:]):
        T = T.dot(MatrixExp6(S*theta))           
    return T.dot(M)

''' 
FKinBody takes M, the position and orientation of the end-effector frame when the manipulator is at its home position, a list of screw axes in the form of a numpy array matrix expressed in body frame, and a list of joint coordinates as a numpy array, and returns the transformation of the end-effector frame at the given joint angles 
Example usage:
L0 = 2
L1 = 1
L2 = 1
L3 = .5
M = np.array([[-1,0,0,0],[0,1,0,L0+L2],[0,0,-1,L1-L3],[0,0,0,1]])
B1 = np.array([0,0,-1,L2,0,0])
B2 = np.array([0,0,0,0,1,0])
B3 = np.array([0,0,1,0,0,.1])
screw_axes = np.vstack((B1,B2,B3))
joints = ([pi/2,3,pi])
np.around(FKinBody(M,screw_axes,joints),decimals=3)
Out[295]: 
array([[-0. ,  1. ,  0. , -1. ],
       [ 1. ,  0. ,  0. ,  2. ],
       [ 0. ,  0. ,  0. ,  1. ]])
'''
def FKinBody(M, screw_axes, joints):
    T = M
    for S, theta in zip(screw_axes, joints):
        T = T.dot(MatrixExp6(S*theta))
    return T

'''    
FixedJacobian takes a set of screw axes and joint angles for a robot's joints expressed in the fixed space frame and returns the space Jacobian.
ss1 = np.array([[0,0,1,0,0,0],[0,0,1,0,-L1,0],[0,0,1,0,-L1-L2,0],[0,0,0,0,0,1]])
js1 = np.array([0,pi/2,0,0])
FixedJacobian(ss1,js1)
Out[207]: 
array([[ 0.,  0.,  0.,  0.],
       [ 0.,  0.,  0.,  0.],
       [ 1.,  1.,  1.,  0.],
       [ 0.,  0.,  2.,  0.],
       [ 0., -1., -1.,  0.],
       [ 0.,  0.,  0.,  1.]])
'''
def FixedJacobian(screw_axes, joints):
    J = np.array([screw_axes[0]])
    for i,S in enumerate(screw_axes[1:]):
        T = MatrixExp6(screw_axes[0]*joints[0])
        for S_next,th_next in zip(screw_axes[1:i+1],joints[1:i+1]):
            T = T.dot(MatrixExp6(S_next*th_next)) 
        V = Adjoint(T).dot(S)
        J = np.concatenate((J,[V]), axis=0)
    return J.T

'''
BodyJacobian takes a set of screw axes and joint angles for a robot's joints expressed in the body space frame and returns the body Jacobian.
ss1 = np.array([[0,0,1,0,0,0],[0,0,1,0,-L1,0],[0,0,1,0,-L1-L2,0],[0,0,0,0,0,1]])
js1 = np.array([0,pi/2,0,0])
BodyJacobian(ss1,js1)
Out[138]: 
array([[ 0.,  0.,  0.,  0.],
       [ 0.,  0.,  0.,  0.],
       [ 1.,  1.,  1.,  0.],
       [-1.,  0.,  0.,  0.],
       [-1., -1., -2.,  0.],
       [ 0.,  0.,  0.,  1.]])
'''
def BodyJacobian(screw_axes, joints):
    n = joints.size
    J = np.array([screw_axes[0]])
    for i,Bi in enumerate(screw_axes[:n-1]): 
        T = MatrixExp6(-screw_axes[n-1]*joints[n-1])
        for B, th in zip(reversed(screw_axes[i+1:n-1]),reversed(joints[i+1:n-1])):
            T = T.dot(MatrixExp6(-B*th))
        V = Adjoint(T).dot(Bi)    
        J = np.concatenate((J,[V]), axis=0)
    J = np.concatenate((J,[screw_axes[n-1]]),axis=0)
    J = np.delete(J,0,0)
    return J.T

'''
IKinBody takes a set of screw axes expressed in the body frame, the end-effector zero configuration, the desired end-effector configuration, a initial guess for the joints, and error limits for the final solution. The bottom row of the returned matrix is the final solution.
L0 = 2
L1 = 1
L2 = 1
L3 = .5
B1 = np.array([0,0,-1,L2,0,0])
B2 = np.array([0,0,0,0,1,0])
B3 = np.array([0,0,1,0,0,.1])
screw_axes = np.vstack((B1,B2,B3))
M = np.array([[-1,0,0,0],[0,1,0,L0+L2],[0,0,-1,L1-L3],[0,0,0,1]])
Tsd = FKinBody(M, screw_axes, np.array([pi/2,pi/2,pi/8]))
joints = np.array([0,0,0])
err_omega = .01
err_vel = .001
maxiterates = 100
IKinBody(screw_axes,M,Tsd,joints,err_omega,err_vel,maxiterates)
array([[ 0.  ,  0.  ,  0.  ],
       [ 2.84,  0.63,  1.65],
       [ 1.34, -0.24,  0.15],
       [ 2.11,  1.5 ,  0.92],
       [ 1.58,  1.21,  0.39],
       [ 1.57,  1.57,  0.38]])
'''
def IKinBodyMat(screw_axes,M,Tsd,joints,err_omega,err_vel,maxiterates):
    i = 0
    Tsb = FKinBody(M, screw_axes, joints)
    Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
    omega_b = Vb[0:3]
    vel_b = Vb[3:6]
    th_old = joints
    th_mat = [th_old]
    while (magnitude(omega_b) > err_omega or magnitude(vel_b) > err_vel) and i <= maxiterates:
        th_new = th_old + MPJ(BodyJacobian(screw_axes,th_old)).dot(Vb)
        i += 1
        Tsb = FKinBody(M, screw_axes, th_new)
        Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
        omega_b = Vb[0:3]
        vel_b = Vb[3:6]
        th_old = th_new
        th_mat = np.concatenate((th_mat,[th_new]), axis=0)
    return th_mat

''' 
MP takes a non-square Jacobian and returns the Moore-Penrose pseudoinverse.
If the inverse is singular, this function finds a new random position to start with by taking the transpose of the Jacobian.
joints = np.array([pi/4,pi/2,pi/8])
B1 = np.array([0,0,-1,L2,0,0])
B2 = np.array([0,0,0,0,1,0])
B3 = np.array([0,0,1,0,0,.1])
screw_axes = np.vstack((B1,B2,B3))
J = BodyJacobian(screw_axes,joints)
MPJ(J)
Out[18]: 
array([[ 0.   ,  0.   , -0.001,  0.359, -0.149,  0.015],
       [ 0.   ,  0.   ,  0.   ,  0.383,  0.924,  0.   ],
       [ 0.   ,  0.   ,  0.989,  0.355, -0.147,  0.114]])
'''
def MPJ(J):
    m = J.shape[0]
    n = J.shape[1]
    if n>m:
        try:
            Jhat = (J.T).dot(inv(J.dot(J.T)))
        except:
            Jhat = J.T
    elif n<m:
        try:
            Jhat = (inv((J.T).dot(J))).dot(J.T)
        except:
            Jhat = J.T
    else:
        try:
            Jhat = inv(J)
        except:
            Jhat = J
    return Jhat

'''
IKinFixed takes a set of screw axes expressed in the spaceS frame, the end-effector zero configuration, the desired end-effector configuration, a initial guess for the joints, and error limits for the final solution. The bottom row of the returned matrix is the final solution.
Derivation: This algorithm is basically equivalent to the algorithm for IKinBody. Differences show up for the forward kinematics, where the screw axes are expressed in space frame and therefore the function FKinFixed is used, and in calculating the spatial velocity. Since MatrixLog6(Tbs.dot(Tsd)) returns spatial velocity in the body frame, this is converted to space frame spatial velocity using the Adjoint Matrix in the following form: Vs = Adjoint(Tsb).dot(Vb).
L1 = .55
L2 = .3
L3 = .06
W1 = .045
M_wam = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,L1+L2+L3],[0,0,0,1]])
S1 = np.array([0,0,1,0,0,0])
S2 = np.array([0,1,0,0,0,0])
S3 = np.array([0,0,1,0,0,0])
S4 = np.array([0,1,0,-L1,0,W1])
S5 = np.array([0,0,1,0,0,0])
S6 = np.array([0,1,0,-L1-L2,0,0])
S7 = np.array([0,0,1,0,0,0])
screw_axes_wam_S = np.vstack((S1,S2,S3,S4,S5,S6,S7))
Tsd = np.array([[1,0,0,.4],[0,1,0,0],[0,0,1,.4],[0,0,0,1]])
joints = np.array([0,0,0,0,0,0,0])
err_omega = .01
erro_vel = .001
maxiterates = 100
np.around(IKinFixed(screw_axes_wam_S, M_wam, Tsd, joints, err_omega, err_vel, maxiterates),decimals=3)
Out[193]: 
array([[ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ],
       [ 0.   ,  0.   ,  0.   , -0.243,  0.   , -0.34 ,  0.   ],
       [ 0.   ,  2.157,  0.   , -4.49 ,  0.   ,  2.333,  0.   ],
       [ 0.   ,  0.668,  0.   , -1.516,  0.   ,  0.847,  0.   ],
       [ 0.   ,  1.327,  0.   , -2.099,  0.   ,  0.772,  0.   ],
       [ 0.   ,  1.418,  0.   , -1.706,  0.   ,  0.288,  0.   ],
       [ 0.   ,  1.354,  0.   , -1.714,  0.   ,  0.359,  0.   ],
       [ 0.   ,  1.354,  0.   , -1.71 ,  0.   ,  0.356,  0.   ]])
'''
def IKinFixedMat(screw_axes,M,Tsd,joints,err_omega,err_vel,maxiterates):
    i = 0
    Tsb = FKinFixed(M, screw_axes, joints)
    Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
    Vs = Adjoint(Tsb).dot(Vb)
    omega_b = Vs[0:3]
    vel_b = Vs[3:6]
    th_old = joints
    th_mat = [th_old]
    while (magnitude(omega_b) > err_omega or magnitude(vel_b) > err_vel) and i <= maxiterates:
        th_new = th_old + MPJ(FixedJacobian(screw_axes,th_old)).dot(Vs)
        i += 1
        Tsb = FKinFixed(M, screw_axes, th_new)
        Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
        Vs = Adjoint(Tsb).dot(Vb)
        omega_b = Vs[0:3]
        vel_b = Vs[3:6]
        th_old = th_new
        print th_old
        th_mat = np.concatenate((th_mat,[th_new]), axis=0)
    return th_mat

def IKinBody(screw_axes,M,Tsd,joints,err_omega,err_vel,maxiterates):
    i = 0
    Tsb = FKinBody(M, screw_axes, joints)
    Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
    omega_b = Vb[0:3]
    vel_b = Vb[3:6]
    th_old = joints
    th_mat = [th_old]
    while (magnitude(omega_b) > err_omega or magnitude(vel_b) > err_vel) and i <= maxiterates:
        th_new = th_old + MPJ(BodyJacobian(screw_axes,th_old)).dot(Vb)
        i += 1
        Tsb = FKinBody(M, screw_axes, th_new)
        Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
        omega_b = Vb[0:3]
        vel_b = Vb[3:6]
        th_old = th_new
    return th_old%(2*pi)

'''
IKinFixed takes a set of screw axes expressed in the spaceS frame, the end-effector zero configuration, the desired end-effector configuration, a initial guess for the joints, and error limits for the final solution. The bottom row of the returned matrix is the final solution.

Derivation: This algorithm is basically equivalent to the algorithm for IKinBody. Differences show up for the forward kinematics, where the screw axes are expressed in space frame and therefore the function FKinFixed is used, and in calculating the spatial velocity. Since MatrixLog6(Tbs.dot(Tsd)) returns spatial velocity in the body frame, this is converted to space frame spatial velocity using the Adjoint Matrix in the following form: Vs = Adjoint(Tsb).dot(Vb).

L1 = .55
L2 = .3
L3 = .06
W1 = .045
M_wam = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,L1+L2+L3],[0,0,0,1]])
S1 = np.array([0,0,1,0,0,0])

S2 = np.array([0,1,0,0,0,0])
S3 = np.array([0,0,1,0,0,0])
S4 = np.array([0,1,0,-L1,0,W1])
S5 = np.array([0,0,1,0,0,0])
S6 = np.array([0,1,0,-L1-L2,0,0])
S7 = np.array([0,0,1,0,0,0])

screw_axes_wam_S = np.vstack((S1,S2,S3,S4,S5,S6,S7))
Tsd = np.array([[1,0,0,.4],[0,1,0,0],[0,0,1,.4],[0,0,0,1]])
joints = np.array([0,0,0,0,0,0,0])
err_omega = .01
erro_vel = .001

maxiterates = 100
np.around(IKinFixed(screw_axes_wam_S, M_wam, Tsd, joints, err_omega, err_vel, maxiterates),decimals=3)

Out[193]: 
array([[ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ],
       [ 0.   ,  0.   ,  0.   , -0.243,  0.   , -0.34 ,  0.   ],

       [ 0.   ,  2.157,  0.   , -4.49 ,  0.   ,  2.333,  0.   ],
       [ 0.   ,  0.668,  0.   , -1.516,  0.   ,  0.847,  0.   ],
       [ 0.   ,  1.327,  0.   , -2.099,  0.   ,  0.772,  0.   ],
       [ 0.   ,  1.418,  0.   , -1.706,  0.   ,  0.288,  0.   ],
       [ 0.   ,  1.354,  0.   , -1.714,  0.   ,  0.359,  0.   ],

       [ 0.   ,  1.354,  0.   , -1.71 ,  0.   ,  0.356,  0.   ]])
'''
def IKinFixed(screw_axes,M,Tsd,joints,err_omega,err_vel,maxiterates):
    i = 0
    Tsb = FKinFixed(M, screw_axes, joints)
    Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
    Vs = Adjoint(Tsb).dot(Vb)
    omega_b = Vs[0:3]
    vel_b = Vs[3:6]
    th_old = joints
    th_mat = [th_old]
    while (magnitude(omega_b) > err_omega or magnitude(vel_b) > err_vel) and i <= maxiterates:
        th_new = th_old + MPJ(FixedJacobian(screw_axes,th_old)).dot(Vs)
        i += 1
        Tsb = FKinFixed(M, screw_axes, th_new)
        Vb = MatrixLog6(TransInv(Tsb).dot(Tsd))
        Vs = Adjoint(Tsb).dot(Vb)
        omega_b = Vs[0:3]
        vel_b = Vs[3:6]
        th_old = th_new        
    return th_old%(2*pi)


'''
CubicTimeScaling takes a total travel time T and the current time t in [0,T] and returns the path parameter s corresponding to a motion that begins and ends at zero velocity
'''
def CubicTimeScaling(T,t):
    a2 = 3./(T**2)
    a3 = -2./(T**3)
    s = a2*t**2+a3*t**3
    return s
    
'''
QuinticTimeScaling takes a total travel time T and the current time t in [0,T] and returns the path parameter s corresponding to a motion that begins and ends at zero velocity and zero acceleration
'''
def QuinticTimeScaling(T,t):
    if (t > T) or (t < 0):
        raise ValueError('Out of time range')
    a3 = 10./(T**3)
    a4 = -15./(T**4)
    a5 = 6./(T**5)
    s = a3*t**3 + a4*t**4 + a5*t**5
    return s

'''
JointTrajectory takes initial and final joint variables, the time of the motion, the number of points in the trajectory, and the time scaling method as 3 or 5 represeting cubic and quintic time scaling. The elapsed time between each row is T/(N-1). The trajectory is a straight-line motion in joint space
'''
def JointTrajectory(thStart, thEnd, T, N, timeScale):
    thList = [thStart]
    if (N < 2):
        raise ValueError('N must be 2 or greater')
    tSpace = np.linspace(0,T,N)
    for t in tSpace[1:]:
        if timeScale == 3:
            s = CubicTimeScaling(T,t)
        elif timeScale == 5:
            s = QuinticTimeScaling(T,t)
        else:
            raise ValueError('timeScale must be 3 or 5')
        th = (1-s)*thStart + s*thEnd
        thList = np.concatenate((thList,[th]), axis=0)
    return thList
        
'''
ScrewTrajectory takes initial and final end-effectory configurations, the time of the motion, the number of points in the trajectory, and the time scaling method as 3 or 5 represeting cubic and quintic time scaling. The elapsed time between each row is T/(N-1). This represents a discretized trajectory of the screw motion.
''' 
def ScrewTrajectory(Xstart, Xend, T, N, timeScale):
    tSpace = np.linspace(0,T,N)
    Xse = TransInv(Xstart).dot(Xend)
    i = 0
    Xlist = np.empty((N,4,4))
    Xlist[0] = Xse;
    for t in tSpace[1:]:
        i = i+1;
        if timeScale == 3:
            s = CubicTimeScaling(T,t)
        elif timeScale == 5:
            s = QuinticTimeScaling(T,t)
        else:
            raise ValueError('timeScale must be 3 or 5')
        X = Xstart.dot(MatrixExp6(MatrixLog6(Xse)*s))
        Xlist[i] = X
    return Xlist

'''
CartesianTrajectory takes initial and final end-effectory configurations, the time of the motion, the number of points in the trajectory, and the time scaling method as 3 or 5 represeting cubic and quintic time scaling. The elapsed time between each row is T/(N-1). This represents a straight line trajectory, where the origin's motion is decoupled from the rotational motion.
''' 
def CartesianTrajectory(Xstart, Xend, T, N, timeScale):
    Rs, ps = TransToRp(Xstart)
    Re, pe = TransToRp(Xend)
    tSpace = np.linspace(0,T,N)
    Rse = RotInv(Rs).dot(Re)
    i = 0
    Xlist = np.empty((N,4,4))
    Xlist[0] = Xstart
    for t in tSpace[1:]:
        i = i+1;
        if timeScale == 3:
            s = CubicTimeScaling(T,t)
        elif timeScale == 5:
            s = QuinticTimeScaling(T,t)
        else:
            raise ValueError('timeScale must be 3 or 5')
        p = ps + s*(pe-ps)
        R = Rs.dot(MatrixExp3(MatrixLog3(Rse)*s))
        Xlist[i] = RpToTrans(R,p)
    return Xlist

''' 
InverseDynamics takes in a vector of joint positions, velocities, and accelerations, a three-vector g indicating the direction and magnitude of gravitational acceleration, a spatial force indicating the force at the tip, and a description of the robot
M01 = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0.089159],[0,0,0,1]])
M12 = np.array([[0, 0, 1, 0.28],[0, 1, 0, 0.13585],[-1, 0, 0, 0],[0,0,0,1]])
M23 = np.array([[1, 0, 0, 0],[0, 1, 0, -0.1197],[0, 0, 1, 0.395],[0,0,0,1]])
M34 = np.array([[0, 0, 1, 0],[0, 1, 0, 0],[-1, 0, 0, 0.14225],[0,0,0,1]])
M45 = np.array([[1, 0, 0, 0],[0, 1, 0, 0.093],[0, 0, 1, 0],[0,0,0,1]])
M56 = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0.094565],[0,0,0,1]])
M67 = np.array([[1, 0, 0, 0],[0, 1, 0, .082],[0, 0, 1, 0],[0,0,0,1]])
M = [M01, M12, M23, M34, M45, M56, M67]
S1 = np.array([0,0,1,0,0,0])
S2 = np.array([0,1,0,-.089159,0,0])
S3 = np.array([0,1,0,-.089159,0,.425])
S4 = np.array([0,1,0,-.089159,0,.81725])
S5 = np.array([0,0,-1,-.10915,.81725,0])
S6 = np.array([0,1,0,.005491,0,.81725])
S = np.vstack((S1,S2,S3,S4,S5,S6))
m = 3.7; Ixx=0.01026749; Ixy = 0; Ixz = 0; Iyy = 0.01026749; Iyz = 0; Izz = 0.00666
G1 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 8.393; Ixx=0.22689; Iyy = Ixx; Izz = 0.0151
G2 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 2.275; Ixx=0.04944; Iyy = Ixx; Izz = 0.004095
G3 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 1.219; Ixx=0.1111727; Iyy = Ixx; Izz = 0.21942
G4 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 1.219; Ixx=0.1111727; Iyy = Ixx; Izz = 0.21942
G5 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = .1879; Ixx=0.017136; Iyy = Ixx; Izz = 0.033822
G6 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
G = [G1, G2, G3, G4, G5, G6]
Ftip = np.array([0,0,0,0,0,0])
g = np.array([0,0,-9.8])
thStart = np.array([0,0,0,0,0,0])
torque = InverseDynamics(thStart, thStart, thStart, g, Ftip, M, G, S)
'''

def InverseDynamics(th, vel, acc, g, Ftip, M, G, S):
    n = S.shape[0]
    V = np.empty([n,6])
    Vdot = np.empty([n,6])
    A = np.empty([n,6])
    F = np.empty([n,6])
    T = np.empty([n,4,4])
    Torque = np.empty([n])
    for i in range(n):
        Mi_m1_i = M[i]
        Mi = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0,0,0,1]])
        for m in range(i+1):
            Mi = Mi.dot(M[m])
        A[i] = Adjoint(TransInv(Mi)).dot(S[i])
        Ti_m1_i = Mi_m1_i.dot(MatrixExp6(A[i]*th[i]))
        T[i] = Ti_m1_i; 
        Ti_i_m1 = TransInv(Ti_m1_i)
        if i == 0:
            V[i] = A[i]*th[i]
            Vdot[i] = Adjoint(Ti_i_m1).dot(np.array([0,0,0,-g[0],-g[1],-g[2]])) + A[i]*acc[i] + Lie(V[i],A[i]*vel[i])
        else:
            V[i] = Adjoint(Ti_i_m1).dot(V[i-1]) + A[i]*vel[i]
            Vdot[i] = Adjoint(Ti_i_m1).dot(Vdot[i-1]) + A[i]*acc[i] + Lie(V[i],A[i]*vel[i])
    for i in range(n-1,-1,-1):
        if i == n-1:
            Ti_p1_i = TransInv(M[i+1])
            F[i] = (Adjoint(Ti_p1_i).transpose()).dot(Ftip) + G[i].dot(Vdot[i]) - (ad(V[i]).transpose()).dot(G[i].dot(V[i]))
            Torque[i] = (F[i].transpose()).dot(A[i])
        else:
            Ti_i_p1 = T[i+1]
            Ti_p1_i = TransInv(Ti_i_p1) 
            F[i] = (Adjoint(Ti_p1_i).transpose()).dot(F[i+1]) + G[i].dot(Vdot[i]) - (ad(V[i]).transpose()).dot(G[i].dot(V[i]))
            Torque[i] = (F[i].transpose()).dot(A[i])
    return Torque
        
'''
Lie takes in two 6-vectors and computes the Lie bracket
'''
def Lie(V1,V2):
    return ad(V1).dot(V2)

'''
ad computes the adjoint of a vector
'''    
def ad(V):
    what = VecToso3(V[0:3])
    vhat = VecToso3(V[3:6])
    top = np.concatenate((what,np.zeros((3,3))),axis=1)
    bot = np.concatenate((vhat, what),axis=1)
    return np.concatenate((top,bot))

'''
InertiaMatrix computes the numerical inertia matrix of a n-joint serial chain at a given configuration
'''    
def InertiaMatrix(th, M, G, S):
    n = S.shape[0]
    Imat = np.empty([6,n])
    for i in range(n):
        vel = np.zeros(n)
        acc = np.zeros(n)
        acc[i] = 1
        g = np.array([0,0,0])
        Ftip = np.array([0,0,0,0,0,0])
        I = InverseDynamics(th, vel, acc, g, Ftip, M, G, S)
        Imat[:,i]=I
    return Imat

'''
CoriolisForces computer the vector of Coriolis and centripetal terms for a given configuration
'''
def CoriolisForces(th, vel, M, G, S):
    n = S.shape[0]
    acc = np.zeros(n)
    g = np.array([0,0,0])
    Ftip = np.array([0,0,0,0,0,0])
    C = InverseDynamics(th, vel, acc, g, Ftip, M, G, S)
    return C

'''
GravityForces computes the vector g(theta) for a given configuration
'''   
def GravityForces(th, g, M, G, S):
    n = S.shape[0]
    vel = np.zeros(n)
    acc = np.zeros(n)
    Ftip = np.array([0,0,0,0,0,0])
    f = InverseDynamics(th, vel, acc, g, Ftip, M, G, S)
    return f

'''
EndEffectorForces computes the jacobian multiplied by the tip force for a given configuration
'''
def EndEffectorForces(th, Ftip, S):
    J = FixedJacobian(S, th)
    return (J.transpose()).dot(Ftip)

'''
ForwardDynamics computes the acceleration vector for a robot given configuration and torque
'''    
def ForwardDynamics(th, vel, torque, g, Ftip, M, G, S):
    c = CoriolisForces(th, vel, M, G, S)
    fg = GravityForces(th, g, M, G, S)
    fj = EndEffectorForces(th, Ftip, S)
    m = InertiaMatrix(th, M, G, S)
    RH = torque - c - fg - fj
    return np.linalg.pinv(m).dot(RH)

'''
EulerStep takes the current state and returns the state after a change in time
'''    
def EulerStep(th0, vel0, acc0, del_t):
    th = th0 + del_t*vel0
    vel = vel0 + del_t*acc0
    return th, vel
    
'''
InverseDynamicsTrajectory takes a robot trajectory specified as a set of N+1 points for position, velocity, and acceleration and output a set of N torques
thStart = np.array([0,0,0,0,0,0])
thEnd = np.array([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2])
T = 1
del_t = .001
N = T/del_t
tSpace = np.linspace(0,T,N)
thList = JointTrajectory(thStart, thEnd, T, N, 5)
g = np.array([0,0,-9.8])
Ftips = np.zeros([N, 6])
velList = np.zeros([N,6])
accList = np.zeros([N,6])
for i in range(6):
    pos = thList[:,0]
    vel = np.append(0,np.diff(pos)/del_t)
    acc = np.append(0,np.diff(vel)/del_t)
    velList[:,i] = vel
    accList[:,i] = acc    
torques = InverseDynamicsTrajectory(thList, velList, accList, g, Ftips, M, G, S)
'''
def InverseDynamicsTrajectory(ths, vels, accs, g, Ftips, M, G, S):
    N = ths.shape[0]
    n = S.shape[0]
    torques = np.empty([N, n])
    for i in range(N):
        th = ths[i]
        vel = vels[i]
        acc = accs[i]
        Ftip = Ftips[i]
        torque = InverseDynamics(th, vel, acc, g, Ftip, M, G, S)
        torques[i] = torque
    return torques

'''
ForwardDynamicsTrajectory takes an initial robot state and a torque history and computes the robot state as a function of time
'''
def ForwardDynamicsTrajectory(th, vel, torques, del_t, g, Ftip, M, G, S):
    N = torques.shape[0]
    n = S.shape[0]
    ths = np.empty([N, n])
    vels = np.empty([N, n])
    ths[0] = th
    vels[0] = vel
    for i in range(N):
        torque = torques[i]
        acc = ForwardDynamics(th, vel, torque, g, Ftip, M, G, S)
        th, vel = EulerStep(th, vel, acc, del_t)
        ths[i] = th
        vels[i] = vel
    return ths, vels
    
