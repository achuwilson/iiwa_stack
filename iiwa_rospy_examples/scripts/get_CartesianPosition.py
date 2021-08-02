#!/usr/bin/env python3

import rospy
from iiwa_msgs.msg import CartesianPose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('get_CartesianPose', anonymous=False)

#read a single message
msg = rospy.wait_for_message("/iiwa/state/CartesianPose", CartesianPose)
position = [msg.poseStamped.pose.position.x, msg.poseStamped.pose.position.y, msg.poseStamped.pose.position.z]
orn_quaternion = [msg.poseStamped.pose.orientation.x, msg.poseStamped.pose.orientation.y, msg.poseStamped.pose.orientation.z, msg.poseStamped.pose.orientation.w]
orn_euler = euler_from_quaternion(orn_quaternion)

print("POSITION : ", position)
print("ORIENTATION QUATERNION : ", orn_quaternion)
print("ORIENTATION EULER : ", orn_euler)
