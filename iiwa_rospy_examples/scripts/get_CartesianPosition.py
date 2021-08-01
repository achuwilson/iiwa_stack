#!/usr/bin/env python3

import rospy
from iiwa_msgs.msg import CartesianPose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    #print(data.position)
    position = [data.pose.position.x,data.pose.position.y,data.pose.position.z]
    print("POSITION ", position)
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/iiwa/state/CartesianPose", CartesianPose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()