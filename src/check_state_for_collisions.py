#!/usr/bin/env python
import rospy
# from std_srvs.srv import Empty
# from std_srvs.srv import EmptyRequest
# from std_srvs.srv import EmptyResponse
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetStateValidityRequest
from moveit_msgs.srv import GetStateValidityResponse
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

# import baxter_interface

import copy

import kbhit

def checkCollision(js):
        coll_client = rospy.ServiceProxy("check_state_validity", GetStateValidity)
        gsvr = GetStateValidityRequest()
        rs = RobotState()
        rs.joint_state = ts
        # rospy.loginfo(rs)
        gsvr.robot_state = rs
        gsvr.group_name = "both_arms"
        resp = coll_client(gsvr)
        rospy.loginfo(resp.valid)



class CheckCollisionState( object ):
    def __init__(self):
        # create kbhit instance
        self.kb = kbhit.KBHit()

        # create local variables:
        self.js_latest = None

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
        self.kb_timer = rospy.Timer(rospy.Duration(0.05), self.timercb)
        self.js_sub = rospy.Subscriber("joint_states", JointState, self.js_cb)

        # create instances of Baxter limbs:
        # self.right_arm = baxter_interface.limb.Limb("right")
        # self.left_arm = baxter_interface.limb.Limb("left")
        return


    def timercb(self, tdat):
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 'p':
                rospy.loginfo("You pressed 'p'.... querying current state")
                self.query_current_state()
            self.kb.flush()
        return


    def js_cb(self, js):
        self.js_latest = js
        # rospy.loginfo(js)
        return


    def query_current_state(self):

        # js.header = 
        # if js is None or (js.header.stamp - rospy.Time.now()).to_sec() > 0.5:
        #     rospy.logwarn("Joint state information unavailable!")
        #     return

        # coll_client(gsvr)

        # rospy.loginfo("STATE VALID?"  %s",resp.valid")
        return
        
    

def main():
    rospy.init_node('check_collisions_node', log_level=rospy.INFO)
    rospy.loginfo("Starting up collision checking demo node")
    try:
        coll_checker = CheckCollisionState()
    except rospy.ROSInterruptException:
        pass
    if coll_checker:
        rospy.loginfo('here')
        ts = JointState()
        # js = copy.deepcopy(self.js_latest)
        ts.name = ['head_pan',
 'right_s0',
 'right_s1',
 'right_e0',
 'right_e1',
 'right_w0',
 'right_w1',
 'right_w2',
 'left_s0',
 'left_s1',
 'left_e0',
 'left_e1',
 'left_w0',
 'left_w1',
 'left_w2']
  #       ts.position = [-0.12292413412121322,
  # -0.5102427709463296,
  # 0.24618322580899588,
  # 1.7910693208863315,
  # -0.7719102566800585,
  # -1.033812230696972,
  # -1.2285815669617817,0,0,0,0,0,0,0,0]
        ts.position = [0,.24,0,.04,2.62,0,0,0,0,0,0]
        gsvr = GetStateValidityRequest()
        rs = RobotState()
        rs.joint_state = ts
        # rospy.loginfo(rs)
        gsvr.robot_state = rs
        gsvr.group_name = "both_arms"
        resp = coll_checker.coll_client(gsvr)
        rospy.loginfo(resp.valid)

 #        try:
 #            rospy.loginfo(gsvr)
 #            resp = coll_checker.coll_client(gsvr)
 #        except rospy.ServiceException, e:
 #            rospy.logwarn("Service did not process request: %s"%str(e))
 #            return
 #        rospy.loginfo("STATE VALID?"  %s",resp.valid")



    rospy.spin()



if __name__=='__main__':
    main()

