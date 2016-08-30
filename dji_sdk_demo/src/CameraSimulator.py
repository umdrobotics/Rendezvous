#!/usr/bin/env python

import ros

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 

#Following 2 lines import the apriltags message format. May need to play with these
from AprilTagDetectionArray.msg import *
from AprilTagDetection.msg import * 



#import ros.geometry_msgs
#from geomemsg.Point
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

import rospy
import std_msgs
import  std_msgs.msg


h = std_msgs.msg.Header()#std_msgs.msg.
h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
p = Point()#geometry_msgs.

p.x=0
p.y=0
p.z=150 

#ps =PointStamped(h,p)  #geometry_msgs.msg.PointStamped(h,p)
q = Quaternion()
q.x=0 ;q.y=0; q.z=0; q.w=0;

poseU = Pose(p,q)
poseS = PoseStamped(h, poseU)

tag = AprilTagDetection()
tag.size = 0.163513 #default from the launch file
tad.id = 1 #the tad number I believe
tag.pose = poseS 

atda = AprilTagDetectionArray()
atda.detections = [tag] 

pub = rospy.Publisher('/dji_sdk/cameraSimulator', AprilTagDetectionArray, queue_size=1)
pub.publish(ps)

def maketag(x ,y ,z ):
			h = std_msgs.msg.Header()#std_msgs.msg.
			h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
			p = Point()#geometry_msgs.

			p.x=xIn
			p.y=yIn
			p.z=zIn   
			q = Quaternion()
			q.x=0 ;q.y=0; q.z=0; q.w=0;			
			
			poseU = Pose(p,q)
			poseS = PoseStamped(h, poseU)

			tag = AprilTagDetection()
			tag.size = 0.163513 #default from the launch file
			tad.id = 1 #the tad number I believe
			tag.pose = poseS 
			atda = AprilTagDetectionArray()
			atda.detections = [tag] 
			
			return atda;


			
			
def main():
	pub = rospy.Publisher('/dji_sdk/cameraSimulator', AprilTagDetectionArray, queue_size=1)
    rospy.init_node('tagPublisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mes = maketag(10,10,20);
        pub.publish(mes);
        rate.sleep()

main()
