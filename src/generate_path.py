import rospy
import baxter_interface
from copy import copy
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from quat import quat_to_so3
import numpy as np
from std_msgs.msg import Int16
from functions import JointTrajectory, normalize
import matplotlib.pyplot as plt


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


    q_start = np.array([0.2339320701525256,  -0.5878981369570848,  0.19903400722813244,  1.8561167533413507,
     -0.4908738521233324,  -0.97752925707998,  -0.49547579448698864])
    q_throw = np.array([0.9265243958827899,  -0.7827136970185323,  -0.095490304045867,  1.8338740319170121,
     -0.03681553890924993,  -0.9909515889739773,  -0.5840631849873713])
    # q_dot = np.array([-0.23825794, -0.13400971,  0.04931685,  0.0264105 , -0.8301056 , 0.28080345,  0.39270727])
    q_dot = np.array([-0.23825794, -0.13400971,  0.04931685,  0.0264105 , -1.9301056 , 1.95080345,  1.9270727])
    q_end = np.array([0.9085001216251363,  -1.0089758632316308, 0.07401457301547121, 1.8768254939778037,
     0.18599517053110642, -0.8172282647459542, -0.44600491407768406])

# finding velocity from jacobian
    jacobian = kdl_kin.jacobian(q_throw)
    inv_jac = np.linalg.pinv(jacobian)
    Vb = np.array([0,0,0,0,1,2])
    q_dot_throw = inv_jac.dot(Vb)


    x = np.array(q_dot_throw)[0]
    mx = np.max(np.abs(x))
    norm = x/mx*.95
    plt.plot(range(7),norm,'m-o')
    plt.show(block=False)



q_throw = np.array([.73, -1.11, .75, 2.22, .624, -1.2, -2.38])



