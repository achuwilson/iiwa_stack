#!/usr/bin/env python3

import rospy
from iiwa_msgs.msg import JointPosition

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    #print(data.position)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/iiwa/state/JointPosition", JointPosition, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()