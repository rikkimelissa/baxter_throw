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

    jacobian = kdl_kin.jacobian(q_throw)
    inv_jac = np.linalg.pinv(jacobian)
    Vb = np.array([0,0,0,0,1,2])
    q_dot_throw = inv_jac.dot(Vb)
    q_dot_norm = normalize(q_dot_throw)

    q_dot_norm:[-0.26208373, -0.14741068,  0.05424854,  0.02905155, -0.91311616,
          0.3088838 ,  0.43197799]

def joint1(t,T):
	s = (0.198*t**5*(30.303 + T))/T**5 - (0.462*t**4*(32.4675 + T))/T**4 + (0.264*t**3*(37.8788+T))/T**3
	return s

def jointUp(t,T):
	s = (-3*t**5*(-2+T))/T**5 - (t**4*(15-7*T))/T**4  - (2*t**3*(-5+2*T))/T**3
	return s

def jointDown(t,T):
	s = t - (3*t**5*(-2+T))/T**5 - (t**4*(15-8*T))/T**4  - (2*t**3*(-5+3*T))/T**3
	return s

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

def JointTrajectoryJoint(thStart, thEnd, T, N, timeScale, part):
	thList = [thStart]
	if (N < 2):
		raise ValueError('N must be 2 or greater')
	tSpace = np.linspace(0,T,N)
	for t in tSpace[1:]:
		if timeScale == 1:
			if part == 1:
				s = joint1A(t,T)
			else:
				s = joint1B(t,T)
		elif timeScale == 5:
			s = QuinticTimeScaling(T,t)
		else:
			raise ValueError('timeScale must be 3 or 5')
		th = (1-s)*thStart + s*thEnd
		thList = np.concatenate((thList,[th]), axis=0)
	return thList

# def joint1A(t,T):
# 	s = (0.714*t**5 *(8.40336 + T))/T**5 - (1.666*t**4 *(9.0036 + T))/T**4 + ( 0.952* t**3 *(10.5042 + T))/T**3
# 	return s
# def joint1B(t,T):
# 	s = -0.238*t + (1.428*t**3*(7.0028 + T))/T**3 - ( 1.904* t**4 *(7.87815 + T))/T**4 + (0.714 *t**5* (8.40336 + T))/T**5
# 	return s

def joint1A(t,T=1):
	s = 0.233 + 7.882 *t**3 - 12.061 *t**4 + 4.872 *t**5
	return s
def joint1B(t,T=1):
	s = 0.926 - 0.238 *t - 5.502 *t**3 + 8.491 *t**4 - 3.444 *t**5
	return s

# random trajectory
q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
 -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
 -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
q_dot = np.array([-0.23825794, -0.13400971,  0.04931685,  0.0264105 , -0.8301056 ,
          0.28080345,  0.39270727])
q_end = np.array([1.015495281580144,  -0.9315098334434175,  0.0839854481367264,  1.7901555794622779,
 0.28186896977394477,  -0.8682331259431442,  -0.6389029981542749])

T = 3
path1 = JointTrajectorySpeedUp(q_start,q_throw,T,100,1);
path2 = JointTrajectorySlowDown(q_throw,q_end,T,100,1)
# j1A = JointTrajectoryJoint(.233, .926, T, 100, 1, 1)
# j1B = JointTrajectoryJoint(.926, 1.015, T, 100, 1, 2)

arr = np.fromiter(xrange(301), dtype="int")
j1A = joint1A(arr/300.)
j1B = joint1B(arr/300.)

path = np.vstack((path1,path2[1:,:]))
vel1 = np.diff(path1,axis=0)
vel1 = np.vstack(([0,0,0,0,0,0,0],vel1))
vel2 = np.diff(path2,axis=0)
vel2 = np.vstack((vel2,[0,0,0,0,0,0,0]))
acc1 = np.diff(vel1,axis=0)
acc1 = np.vstack(([0,0,0,0,0,0,0],acc1))
acc2 = np.diff(vel2,axis=0)
acc2 = np.vstack(([0,0,0,0,0,0,0],acc2))
tSpace = np.linspace(0,1,101)
tSpace2 = np.linspace(0,T,100)

pylab.figure()
pylab.plot(tSpace,j1A,'-r')
pylab.plot(tSpace+3,j1B,'--b')
pylab.show(block = False)
v1A = np.diff(j1A,axis=0)
v1B = np.diff(j1B,axis=0)
pylab.figure()
pylab.plot(tSpace[1:],v1A,'-r')
pylab.plot(tSpace[1:]+3,v1B,'--b')
pylab.show(block=False)
a1A = np.diff(v1A,axis=0)
a1B = np.diff(v1B,axis=0)
pylab.figure()
pylab.plot(tSpace[2:],a1A,'-r')
pylab.plot(tSpace[2:]+3,a1B,'--b')
pylab.show(block=False)

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

pylab.figure()
pylab.plot(tSpace,vel1[:,0],'-r',label='Joint 0')
pylab.plot(tSpace,vel1[:,1],'--b',label='Joint 1')
pylab.plot(tSpace,vel1[:,2],':g',label='Joint 2')
pylab.plot(tSpace,vel1[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace,vel1[:,4],'.y',label='Joint 4')
pylab.plot(tSpace,vel1[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Velocity')
pylab.legend(loc='upper left')
pylab.show(block=False)

pylab.figure()
pylab.plot(tSpace2,acc1[:,0],'-r',label='Joint 0')
pylab.plot(tSpace2,acc1[:,1],'--b',label='Joint 1')
pylab.plot(tSpace2,acc1[:,2],':g',label='Joint 2')
pylab.plot(tSpace2,acc1[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace2,acc1[:,4],'.y',label='Joint 4')
pylab.plot(tSpace2,acc1[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Acceleration')
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


