#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int16
from rospy.numpy_msg import numpy_msg
from rrt_joint import find_path
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from functions import RpToTrans
import matplotlib.pyplot as plt
from path_smooth import shortcut, path2traj
from random import random

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class Checker(object):
    def __init__(self):
        self._coll_checked = False
        self._pub_joints = rospy.Publisher('joint_state_check', numpy_msg(Float32MultiArray), queue_size = 10, latch=True)
        self._sub = rospy.Subscriber('collision_check', Int16, self.sub_cb)
        # self._pub_path = rospy.Publisher('joint_path',numpy_msg(Float32MultiArray), queue_size = 10)
        self._pub_traj = rospy.Publisher('joint_traj',numpy_msg(Float32MultiArray), queue_size = 10)
        self._iter = 0
        self._executing = True
        self._sub_demo_state = rospy.Subscriber('demo_state', Int16, self.sub_demo)
        self._sub_pos_state = rospy.Subscriber('pos_state', Int16, self.sub_pos) 
        self._plot_demo = get_param('plot_demo', False)
        rospy.loginfo("Here's my demo state: ")
        rospy.loginfo(self._plot_demo)
        self._plotIndex = -1
        
    def sub_pos(self,a):
        rospy.loginfo('received pos')
        rospy.loginfo(a)
        self._pos_state = a.data

    def sub_demo(self,a):
        rospy.loginfo(a)
        rospy.loginfo('received start command')
        self._iter = 0
        self._start = rospy.get_time()
        self._path = find_path(self._plot_demo, self._pos_state)
        rospy.loginfo('path found')
        self.publish_joints()

    def sub_cb(self,a):
        # rospy.loginfo(a)
        if self._iter == 0:
            if (a.data):
                rospy.loginfo('RRT path checked')
                self._iter = 1;
                self._plotIndex = -1
                self._traj, path_orig = path2traj(self._path)
                self.smooth_path()
            else:
                rospy.loginfo('Calculating new path')
                self.find_new_path()
        else:
            if (a.data):
                self.replace_segment()
            else:
                if self._iter < 30:
                    if (self._iter == 1 or self._iter == 2):
                        pass
                    else:
                        self._iter += 1
                    self.smooth_path()
                else:
                    # print "sent to publish"
                    self.publish_traj()
                    self._executing = False

    def publish_traj(self):

        rospy.loginfo('publishing traj')
        if (self._traj[:,6] > -1.9).all():
            a = Float32MultiArray()
            N = self._traj.shape[0]
            data = np.reshape(self._traj,(N*22,1))
            a.data = np.array(data, dtype = np.float32)
            rospy.loginfo(rospy.get_time() - self._start)
            self._pub_traj.publish(a)   
        else:
            rospy.loginfo("trying again")
            self._path = find_path(self._plot_demo, self._pos_state)
            self._iter = 0
            self.publish_joints()

    def find_new_path(self):
        self._path = find_path(self._plot_demo, self._pos_state)
        self.publish_joints()

    def publish_joints(self):
        a = Float32MultiArray()
        N = self._path.shape[0]
        thList_arms = np.hstack((self._path[:,0:7],np.zeros((N,7))))
        thList_all = np.hstack((np.zeros((N,1)),thList_arms))
        data = np.reshape(thList_all,(N*15,1))
        a.data = np.array(data, dtype = np.float32)
        self._pub_joints.publish(a)   
        rospy.loginfo ('published RRT')

    def publish_segment(self):
        a = Float32MultiArray()
        thList_arms = np.hstack((self._segment[1:8], np.zeros((1,7))[0]))
        thList_all = np.hstack((np.array([0]),thList_arms))
        data = np.reshape(thList_all,(15,1))
        a.data = np.array(data, dtype = np.float32)
        self._pub_joints.publish(a)  

    def replace_segment(self):
        # print "Segment ", str(self._iter), " smoothed"
        old_dur = self._vertex2[0] - self._vertex1[0]
        new_dur = self._s[-1,0] - self._s[0,0]
        if new_dur < old_dur:
            if (self._plot_demo):
                if (self._plotIndex == -1):
                    # plt.close('all')
                    self._plotIndex = 0
                    # plt.figure()
                    f, self._axarr = plt.subplots(2,2)
                    self._axarr[0,0].plot(self._traj[:,0],self._traj[:,1:8])
                    self._axarr[0,0].set_title('Iterations = 0')
            # print "replacing segment"
            self._traj = np.delete(self._traj,range(int(self._ind1),int(self._ind2+1)),0)
            self._traj[self._ind1:,0] += new_dur - old_dur
            self._traj = np.insert(self._traj,self._ind1,self._s,0)
            if (self._plot_demo):
                if self._plotIndex == 0:
                    self._axarr[0,1].plot(self._traj[:,0],self._traj[:,1:8])
                    title = "Iterations = " + str(self._plotIndex+1)
                    self._axarr[0,1].set_title(title)
                    self._plotIndex = 1
                    # plt.show(block=False)
                    # raw_input('Press enter to continue...')
                elif self._plotIndex == 1:
                    print('check 1')
                    self._axarr[1,0].plot(self._traj[:,0],self._traj[:,1:8])
                    title = "Iterations = " + str(self._plotIndex+1)
                    self._axarr[1,0].set_title(title)
                    self._plotIndex = 2
                    print ('check 2')
                     # plt.show(block=False)
                elif self._plotIndex == 2:
                    print ('check 3')
                    self._axarr[1,1].plot(self._traj[:,0],self._traj[:,1:8])
                    title = "Iterations = " + str(self._plotIndex+1)
                    self._axarr[1,1].set_title(title)
                    self._plotIndex = 3
                    plt.show(block=False)
                    wm = plt.get_current_fig_manager()
                    wm.window.wm_geometry("800x500+1000+0")
                    raw_input('Press enter to continue...')
        else:
            self._iter -=1
        if self._iter < 30:
            print "smoothing next segment"
            self._iter += 1
            self.smooth_path()
        else:
            print "sent to publish"
            self.publish_traj()
            self._executing = False

    def smooth_path(self):
        path_length = self._traj.shape[0]
        if (path_length == 0):
            self.publish_traj()
        else:
            if self._iter == 1:
                # print "chose the first"
                self._ind1 = 0
                self._ind2 = round(random()*(path_length-20)) + 19
                # print self._ind1, self._ind2
            elif self._iter == 2:
                # print "chose the last"
                self._ind1 = round(random()*(path_length-20))
                self._ind2 = path_length - 1
                # print self._ind1, self._ind2
            else:
                self._ind1 = round(random()*(path_length-1))
                self._ind2 = round(random()*(path_length-1))
            if (self._ind1 > self._ind2):
                temp = self._ind1;
                self._ind1 = self._ind2;
                self._ind2 = temp
            if (self._ind1 != self._ind2):
                print "path legth is " + str(path_length)
                print "ind1 is " + str(self._ind1)
                print "list is " + str(self._traj)
                self._vertex1 = self._traj[self._ind1,:]
                self._vertex2 = self._traj[self._ind2,:]
                if np.isfinite(self._vertex1).all() and np.isfinite(self._vertex2).all() and (self._vertex1 < 10).all() and (self._vertex2 < 10).all():
                    self._s = shortcut(self._vertex1,self._vertex2)
                    # print(self._s)
                    # x+=1
                    midpoint_s = self._s[self._s.shape[0]/2,:]
                    self._segment = midpoint_s
                    self.publish_segment()  
                else:
                    # print ("here")
                    # x+=1
                    if self._iter < 30:
                        self._iter += 1
                        self.smooth_path()
                    else:
                        # print "sent to publish"
                        self.publish_traj()
                        self._executing = False                      
            else:
                if self._iter < 30:
                    self.smooth_path()
                else:
                    # print "sent to publish"
                    self.publish_traj()
                    self._executing = False    

def main():

    rospy.init_node('jsc_publisher')
    rate = rospy.Rate(60)    
    check = Checker()

    rospy.spin()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

