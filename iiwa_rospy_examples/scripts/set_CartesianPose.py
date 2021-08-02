#!/usr/bin/env python3
############################ CartesianPose (no interpolation) ############################

import rospy
from iiwa_msgs.msg import CartesianPose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from iiwa_msgs.srv import TimeToDestination, TimeToDestinationRequest, TimeToDestinationResponse

import time

rospy.init_node('set_CartesianPose', anonymous=False)

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
pub = rospy.Publisher("/iiwa/command/CartesianPose",
                      PoseStamped, queue_size=10)

time.sleep(1)

#define a PoseStamped message
pose0 = PoseStamped()
pose0.header.stamp = rospy.Time.now()
# the coordinates are relative to the link0 or base of the iiwa
pose0.header.frame_id = "iiwa_link_0" 
#increment a little in x axis
pose0.pose.position = Point(x=init_position[0]+0.1, y=init_position[1], z=init_position[2])
pose0.pose.orientation = Quaternion(*init_orn)
#publish the message
pub.publish(pose0)

# a small delay required to avoid erratic behavior
# ( so as we start checking hasReached only after robot starts moving)
time.sleep(0.2)
#wait until we have reached the position
while(hasReached().remaining_time>=0):    
    #print("Moving. Remaning time : ", hasReached().remaining_time)
    continue
    
# publish a message again to move back to starting position
#define a PoseStamped message
pose1 = PoseStamped()
pose1.header.stamp = rospy.Time.now()
# the coordinates are relative to the link0 or base of the iiwa
pose1.header.frame_id = "iiwa_link_0" 
#increment a little in x axis
pose1.pose.position = Point(x=init_position[0], y=init_position[1], z=init_position[2])
pose1.pose.orientation = Quaternion(*init_orn)
#publish the message
pub.publish(pose1)


# a small delay required to avoid erratic behavior
# ( so as we start checking hasReached only after robot starts moving)
time.sleep(0.2)
#wait until we have reached the position
while(hasReached().remaining_time>=0):    
    #print("Moving. Remaning time : ", hasReached().remaining_time)
    continue