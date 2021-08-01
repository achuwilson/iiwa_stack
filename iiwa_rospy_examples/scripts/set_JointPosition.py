#!/usr/bin/env python3

import rospy
from iiwa_msgs.msg import JointPosition
import time
rospy.init_node('listener2', anonymous=True)

#get the initial joint position of the robot
msg = rospy.wait_for_message("/iiwa/state/JointPosition", JointPosition, timeout=None)


pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)

#wait a little for the publisher to be initialized and connected to ROS Master
time.sleep(1)

#create a Joint position message and increment only J7 ( for safe demonstration)
j_msg = JointPosition()
j_msg.header.stamp = rospy.Time.now()
j_msg.position = msg.position
j_msg.position.a7 =msg.position.a7 + 0.25

pub.publish(j_msg)
