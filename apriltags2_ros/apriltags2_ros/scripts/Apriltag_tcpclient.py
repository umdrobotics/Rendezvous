#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection

import socket
import re

TCP_IP = '192.168.1.17'
TCP_PORT = 9999
BUFFER_SIZE = 1024


# Setup ROS
rospy.init_node('Apriltag_server', anonymous=True)
pub1 = rospy.Publisher("/usb_cam/tag_detections", AprilTagDetectionArray, queue_size=1)
print("Initialized ROS, try to connect......")


# Setup Socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.connect((TCP_IP, TCP_PORT))
except socket.error:
    raise ValueError("TIMEOUT, check the ip addr and connection! ")
print("Connected to server! Start receiving data.")


# Start processing data and publish msg
while 1:

    data = s.recv(BUFFER_SIZE)


    # Deal with exceptions
    if not data:    continue
    if not ("###" in data and "!!!" in data):     continue


    TagDetection = AprilTagDetection()
    vecTagDetections = AprilTagDetectionArray()

    # Try to split data
    indicesMsgHead = [m.start() for m in re.finditer("###", data)]
    indicesMsgEnd = [m.start() for m in re.finditer("!!!", data)]

    # Process the first msg
    firstMsg = data[indicesMsgHead[0]+3 : indicesMsgEnd[0]]
    indicesComma = [m.start() for m in re.finditer(",", firstMsg)]

    if len(indicesComma) != 6: 	continue

    TagDetection.pose.pose.pose.position.x    = float(firstMsg[0 : indicesComma[0]])
    TagDetection.pose.pose.pose.position.y    = float(firstMsg[indicesComma[0]+1 : indicesComma[1]])
    TagDetection.pose.pose.pose.position.z    = float(firstMsg[indicesComma[1]+1 : indicesComma[2]])
    TagDetection.pose.pose.pose.orientation.x = float(firstMsg[indicesComma[2]+1 : indicesComma[3]])
    TagDetection.pose.pose.pose.orientation.y = float(firstMsg[indicesComma[3]+1 : indicesComma[4]])
    TagDetection.pose.pose.pose.orientation.z = float(firstMsg[indicesComma[4]+1 : indicesComma[5]])
    TagDetection.pose.pose.pose.orientation.w = float(firstMsg[indicesComma[5]+1 : -1])
    vecTagDetections.detections.append(TagDetection)
    pub1.publish(vecTagDetections)
    rospy.loginfo(vecTagDetections)



rospy.spin()
conn.close()


