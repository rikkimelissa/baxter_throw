	# x = 0:.001:2*pi;
	# y = -sin(x);
	# plot(x,y)
	t_fin = 10;
	t = 0:.001:t_fin;
	x = (2*pi)/t_fin*t;
	y = -sin(x);
	plot(x,y)
	vx = diff(x);
	vy = diff(y);
	ay = diff(vy);

	# specs: x0, y0, v0 = 0
	# vr = max v
	# ve = 0
	# constant rotation

	jerk to be zero at beginning and end? 5th order
	β(∆t) = a j0 + a j1 ∆t + a j2 ∆t 2 + a j3 ∆t 3

	s(t) = a 0 + a 1 t + a 2 t 2 + a 3 t 3 .

	Vb = np.array([0,0,0,0,1,2])
	MPJ(BodyJacobian(screw_axes,th_old)).dot(Vb)



	pos.position.x = .7
	pos.position.y = -.5
	pos.position.z = -.1
	pos.orientation.x = .99
	pos.orientation.y = -.024
	pos.orientation.z = .024
	pos.orientation.w = .0133

	R_throw = np.array([[ 1.        ,  0.        ,  0.        ],
		[ 0.        ,  0.86781918,  0.49688014],
		[ 0.        , -0.49688014,  0.86781918]])
	R_throw = np.array([[0,.4968,.8678],[0,.8678,-.4968],[-1,0,0]])
	quat_throw = so3_to_quat(R_throw)
	p_throw = np.array([.7, 0, .3])
	p_start = np.array([.7, -.5, 0])
	p_end = np.array([.7, 0, 0])
	X_throw = RpToTrans(R_throw, p_throw)
	thList = np.array([0,0,0,0,0,0,0])
	seed = 0
	q_ik = kdl_kin.inverse(X_throw, thList)
	tries = 0
	while q_ik == None:
		seed += 0.3
		tries += 1
		if (tries > 200):
			break
		q_ik = kdl_kin.inverse(X_throw, q0+seed)
	print(q_ik)

        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        # Create seed with current position
        q0 = kdl_kin.random_joint_angles()
        limb_interface = baxter_interface.limb.Limb('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]

    q_end = np.array([ 0.55703127,  1.02672179,  2.36138413,  1.08580171,  3.05891801,
       -1.42679103, -0.42898074])
    #q_start = fill in

    jacobian = kdl_kin.jacobian(q_end)
    inv_jac = np.linalg.pinv(jacobian)
    Vb = np.array([0,0,0,0,1,2])
	q_dot_end = inv_jac.dot(Vb)
	q_dot_norm = normalize(q_dot)

def joint1(t,T):
	s = (0.198*t**5*(30.303 + T))/T**5 - (0.462*t**4*(32.4675 + T))/T**4 + (0.264*t**3*(37.8788+T))/T**3
	return s

def jointUp(t,T):
	s = (-3*t**5*(-2+T))/T**5 - (t**4*(15-7*T))/T**4  - (2*t**3*(-5+2*T))/T**3
	return s

def jointDown(t,T):
	s = t - (3*t**5*(-2+T))/T**5 - (t**4*(15-8*T))/T**4  - (2*t**3*(-5+3*T))/T**3
	return s

def JointTrajectory(thStart, thEnd, T, N, timeScale):
    thList = [thStart]
    if (N < 2):
        raise ValueError('N must be 2 or greater')
    tSpace = np.linspace(0,T,N)
    for t in tSpace[1:]:
        s = joint1(t,T)
        th = (1-s)*thStart + s*thEnd
        thList = np.concatenate((thList,[th]), axis=0)
    return thList

def JointTrajectorySpeedUp(thStart, thEnd, T, N, timeScale):
    thList = [thStart]
    if (N < 2):
        raise ValueError('N must be 2 or greater')
    tSpace = np.linspace(0,T,N)
    for t in tSpace[1:]:
        s = jointUp(t,T)
        th = (1-s)*thStart + s*thEnd
        thList = np.concatenate((thList,[th]), axis=0)
    return thList

def JointTrajectorySlowDown(thStart, thEnd, T, N, timeScale):
    thList = [thStart]
    if (N < 2):
        raise ValueError('N must be 2 or greater')
    tSpace = np.linspace(0,T,N)
    for t in tSpace[1:]:
        s = jointDown(t,T)
        th = (1-s)*thStart + s*thEnd
        thList = np.concatenate((thList,[th]), axis=0)
    return thList

path = JointTrajectory(0,.557,2,100,1)
tSpace = np.linspace(0,2,100)
pylab.plot(tSpace,path,'-r')
pylab.show(block=False)

vel = np.diff(path)
tSpace = np.linspace(0,2,99)
pylab.plot(tSpace,vel,'-r')
pylab.show(block=False)


# random trajectory
q_start = np.array([-0.61159692 , 0.75344562 , 1.05147772 , 1.60237397 ,  2.13362129 ,  1.12027284, 
  1.95830825])
q_throw = np.array([ 0.55703127,  1.02672179,  2.36138413,  1.08580171,  3.05891801,
       -1.42679103, -0.42898074])
q_end = np.array([ 0.84862266 ,  0.97046182 ,  2.15649338 ,  1.77949586,   2.03984311 ,  1.74583931, 
  1.06574319])
N = 10

path1 = JointTrajectorySpeedUp(q_start,q_throw,N,100,1);
path2 = JointTrajectorySlowDown(q_throw,q_start,N,100,1)
vel1 = np.diff(path1,axis=0)
np.insert(vel1,0,0)
vel2 = np.diff(path2,axis=0)
np.insert(vel2,0,0)
acc1 = np.diff(vel1,axis=0)
np.insert(acc1,0,0)
acc2 = np.diff(vel2,axis=0)
np.insert(acc2,0,0)
tSpace = np.linspace(0,N,100)

pylab.figure()
pylab.plot(tSpace,path1[:,0],'-r',label='Joint 0')
pylab.plot(tSpace,path1[:,1],'--b',label='Joint 1')
pylab.plot(tSpace,path1[:,2],':g',label='Joint 2')
pylab.plot(tSpace,path1[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace,path1[:,4],'.y',label='Joint 4')
pylab.plot(tSpace,path1[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Joint Trajectory')
pylab.legend(loc='upper left')
pylab.show(block=False)

tSpace = np.linspace(0,2,99)
pylab.figure()
pylab.plot(tSpace,vel1[:,0],'-r',label='Joint 0')
pylab.plot(tSpace,vel1[:,1],'--b',label='Joint 1')
pylab.plot(tSpace,vel1[:,2],':g',label='Joint 2')
pylab.plot(tSpace,vel1[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace,vel1[:,4],'.y',label='Joint 4')
pylab.plot(tSpace,vel1[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Joint Trajectory')
pylab.legend(loc='upper left')
pylab.show(block=False)

pylab.figure()
pylab.plot(tSpace,vel2[:,0],'-r',label='Joint 0')
pylab.plot(tSpace,vel2[:,1],'--b',label='Joint 1')
pylab.plot(tSpace,vel2[:,2],':g',label='Joint 2')
pylab.plot(tSpace,vel2[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace,vel2[:,4],'.y',label='Joint 4')
pylab.plot(tSpace,vel2[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Joint Trajectory')
pylab.legend(loc='upper left')
pylab.show(block=False)


