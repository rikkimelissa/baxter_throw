#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def main():

    rospy.init_node('get_state')
    pub_pos_state = rospy.Publisher('pos_state',Int16, queue_size = 10)

    while (1):
        n = input('Enter desired state: ')
        pub_pos_state.publish(n)

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass