#!/usr/bin/env python3
import rospy
from iiwa_msgs.msg import CartesianPose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from iiwa_msgs.srv import SetSmartServoLinSpeedLimits, SetSmartServoLinSpeedLimitsRequest, SetSmartServoLinSpeedLimitsResponse
from iiwa_msgs.srv import TimeToDestination, TimeToDestinationRequest, TimeToDestinationResponse

import time

rospy.init_node('set_CartesianPose', anonymous=False)

# The service to set velocity
#   - if we give 0, it runs with default velocity
#   - values should be in range 0 - 1
#   - by default it seems to run at velocity of 0.5
# NOTE that the service names for normal move velocity control 
#    and linear move velocity control  are different
# In linear velocity control, we can control the speeds in each of the cartesian coordinates and euler angles.
# it is defined a a ROS Twist message type
vel = Twist()
vel.linear.x = 0.02
vel.linear.y = 0.02
vel.linear.z = 0.02
vel.angular.x = 0.02
vel.angular.y = 0.02
vel.angular.z = 0.02
rospy.wait_for_service("/iiwa/configuration/setSmartServoLinLimits")
try:
    set_vel = rospy.ServiceProxy("/iiwa/configuration/setSmartServoLinLimits", SetSmartServoLinSpeedLimits)
    resp = set_vel(vel) 
    #("RR", resp.success)
except (rospy.ServiceException,e):
    print ("Service call failed: ", e) 

# The service to get the time to commanded position
rospy.wait_for_service("/iiwa/state/timeToDestination")
try:
    hasReached = rospy.ServiceProxy("/iiwa/state/timeToDestination",TimeToDestination)
    resp = hasReached() 
    #print("RT", resp.remaining_time)
except (rospy.ServiceException):
    print ("Service call failed") 

#get the initial position
#read a single message
msg = rospy.wait_for_message("/iiwa/state/CartesianPose", CartesianPose)
init_position = [msg.poseStamped.pose.position.x, msg.poseStamped.pose.position.y, msg.poseStamped.pose.position.z]
init_orn = [msg.poseStamped.pose.orientation.x, msg.poseStamped.pose.orientation.y, msg.poseStamped.pose.orientation.z, msg.poseStamped.pose.orientation.w]

#define a publisher
pub = rospy.Publisher("/iiwa/command/CartesianPoseLin",
                      PoseStamped, queue_size=10)
time.sleep(1)

#set a low velocity
#(since we are moving only in x axis, else we have to set all axes)
#vel.linear.x = 0.02
#set_vel(vel)

#define a PoseStamped message
pose0 = PoseStamped()
pose0.header.stamp = rospy.Time.now()
# the coordinates are relative to the link0 or base of the iiwa
pose0.header.frame_id = "iiwa_link_0" 
#come back to the initial pos
pose0.pose.position = Point(x=init_position[0]+0.1, y=init_position[1], z=init_position[2])
pose0.pose.orientation = Quaternion(*init_orn)
#publish the message
pub.publish(pose0)

# a small delay required to avoid erratic behavior
# ( so as we start checking hasReached only after robot starts moving)
time.sleep(0.2)
#wait until we have reached the position
while(hasReached().remaining_time>=0.05):    
    print("Moving. Remaning time : ", hasReached().remaining_time)
    continue

#set a high velocity
#(since we are moving only in x axis, else we have to set all axes)
#vel.linear.x = 0.5
#set_vel(vel)

#define a PoseStamped message
pose1 = PoseStamped()
pose1.header.stamp = rospy.Time.now()
# the coordinates are relative to the link0 or base of the iiwa
pose1.header.frame_id = "iiwa_link_0" 
#come back to the initial pos
pose1.pose.position = Point(x=init_position[0], y=init_position[1], z=init_position[2])
pose1.pose.orientation = Quaternion(*init_orn)
#publish the message
pub.publish(pose1)

# a small delay required to avoid erratic behavior
# ( so as we start checking hasReached only after robot starts moving)
time.sleep(0.2)
#wait until we have reached the position
while(hasReached().remaining_time>=0.05):    
    #print("Moving. Remaning time : ", hasReached().remaining_time)
    continue


