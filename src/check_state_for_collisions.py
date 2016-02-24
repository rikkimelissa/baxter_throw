#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetStateValidityRequest
from moveit_msgs.srv import GetStateValidityResponse
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float32MultiArray, Int16
from rospy.numpy_msg import numpy_msg

class CheckCollisionState( object ):
    def __init__(self):
        # create subscribers, timers, clients, etc.
        try:
            rospy.wait_for_service("/check_state_validity", timeout=5)
        except ROSException:
            rospy.logwarn("[check_collisions_node] Done waiting for /check_state_validity service... service not found")
            rospy.logwarn("shutting down...")
            rospy.signal_shutdown("service unavailable")
        except ROSInterruptException:
            pass
        self.coll_client = rospy.ServiceProxy("check_state_validity", GetStateValidity)
        self.js_sub = rospy.Subscriber("joint_state_check", numpy_msg(Float32MultiArray), self.js_cb)
        self.js_pub = rospy.Publisher("collision_state", Int16, queue_size = 10)
        return


    def js_cb(self, a):
        js = JointState()
        js.name = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        jList = a.data
        jMatrix = np.reshape(jList,(jList.shape[0]/15,15))
        for pos in jMatrix:
            js.position = pos
            gsvr = GetStateValidityRequest()
            rs = RobotState()
            rs.joint_state = js
            gsvr.robot_state = rs
            gsvr.group_name = "both_arms"
            resp = self.coll_client(gsvr)
            if (resp.valid == False):
                rospy.loginfo('false')
                self.js_pub.publish(0)
                return
        self.js_pub.publish(1)
        rospy.loginfo('true')
        return   

def main():
    rospy.init_node('check_collisions_node', log_level=rospy.INFO)
    rospy.loginfo("Starting up collision checking demo node")
    try:
        coll_checker = CheckCollisionState()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

if __name__=='__main__':
    main()

