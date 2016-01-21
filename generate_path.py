
'''	
original function definition for path
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
'''

	Vb = np.array([0,0,0,0,1,2])
	MPJ(BodyJacobian(screw_axes,th_old)).dot(Vb)

'''
Trying to calculate a throw position

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
'''

# connecting to robot

        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        # Create seed with current position
        q0 = kdl_kin.random_joint_angles()
        limb_interface = baxter_interface.limb.Limb('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]


# finding velocity from jacobian
    jacobian = kdl_kin.jacobian(q_throw)
    inv_jac = np.linalg.pinv(jacobian)
    Vb = np.array([0,0,0,0,1,2])
    q_dot_throw = inv_jac.dot(Vb)
    q_dot_norm = normalize(q_dot_throw)






