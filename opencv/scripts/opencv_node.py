#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

# import built-in module and opencv
import cv2
import time
import math
# import our own module: OpencvLibrary
import OpencvLib
# import ROS related library
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from opencv.msg import AprilTagDetectionArray, AprilTagDetection


# Enable to record raw video without detection 
ENABLE_VIDEO_RECORD = False

"""
Function: sigint_handler
Mainly deal with the keyboard interupt
Clean up everything. 
"""
def sigint_handler():
    print("shutdown time!")

    if ENABLE_VIDEO_RECORD:
        rawImOut.release()
    # del imageObj
    # del opencvInfoPub
    # del imagePub

    
"""
Function: try_connect_camera
Mainly initial camera.
Usually try 5 times to connect, in case port index changes.
"""
def try_connect_camera():

    try:
        imageObj.connect_camera(0)
    finally:
        print

    for i in range(1,20):
        if imageObj.capture.isOpened():
            break
        try: 
            imageObj.connect_camera(i)
        finally:
            print



"""
Function: calculate_opencv_data
Mainly calculate the distance & pose frome a capture image. 
Output: distance_NED which is saved in imageObj attributes. 
"""
def detect_tags():
    
    # capture image and record cam shot time 
    # camShotTime is for Kalman Filter 
    # camShotTimeHMS is for displaying on video
    ret, originalImage = imageObj.capture.read()
    imageObj.camShotTime = time.time()
    imageObj.camShotTimeHMS = time.strftime("%H:%M:%S")

    # record raw video
    if ENABLE_VIDEO_RECORD:
        rawImOut.write(originalImage)

    # start calculate 
    imageObj.prepare_image(originalImage)
    imageObj.detect_markers()
    imageObj.identify_markerID()
    imageObj.estimate_position_pose()



"""
Function: publish_opencv_data
Mainly translate the opencv_data into messages
So that other pkg can receive via ROS.
"""
def publish_data():

    # # opencv info message
    # # The position part is about distance_NED
    # # The orientation part is about gimbal angle, camshot time and target lost or not.
    # # msgOpencvInfo.pose.orientation.w = -1 means target lost, 1 means target found
    # msgOpencvInfo = PoseStamped()
    # msgOpencvInfo.header.frame_id = str(imageObj.currentMarkerFlag)
    # msgOpencvInfo.header.stamp = rospy.Time.now()
    # msgOpencvInfo.pose.position.x = imageObj.distance_north
    # msgOpencvInfo.pose.position.y = imageObj.distance_east 
    # msgOpencvInfo.pose.position.z = imageObj.distance_down 
    # msgOpencvInfo.pose.orientation.x = imageObj.gimbal_pitch_current
    # msgOpencvInfo.pose.orientation.y = imageObj.gimbal_pitch_desired
    # msgOpencvInfo.pose.orientation.z = imageObj.camShotTime
    # msgOpencvInfo.pose.orientation.w = -1 if imageObj.isTargetLost else 1
    # opencvInfoPub.publish(msgOpencvInfo)
    # # Print debug data
    # # rospy.loginfo(msgOpencvInfo)
    # # print(time.time(), imageObj.distance_north, imageObj.distance_east, imageObj.currentMarkerFlag)


    msgTagPose = PoseStamped()
    msgTagPose.header.stamp = rospy.Time.now()
    msgTagPose.pose.position.x = imageObj.distanceInCameraFrame[0]
    msgTagPose.pose.position.y = imageObj.distanceInCameraFrame[1]
    msgTagPose.pose.position.z = imageObj.distanceInCameraFrame[2]
    msgTagPose.pose.orientation.x = 0
    msgTagPose.pose.orientation.y = 0
    msgTagPose.pose.orientation.z = 0
    msgTagPose.pose.orientation.w = 0
    # msgTagPose.pose.orientation.x = imageObj.quaternion[1]
    # msgTagPose.pose.orientation.y = imageObj.quaternion[2]
    # msgTagPose.pose.orientation.z = imageObj.quaternion[3]
    # msgTagPose.pose.orientation.w = imageObj.quaternion[0]
    # Print debug data
    # rospy.loginfo(msgTagPose)
    # print(time.time(), imageObj.distance_north, imageObj.distance_east, imageObj.currentMarkerFlag)

    msgTagDetection = AprilTagDetection()
    msgTagDetection.id = 0
    msgTagDetection.size = 0
    msgTagDetection.pose = msgTagPose
    
    msgTagDetectionArray = AprilTagDetectionArray()
    msgTagDetectionArray.detections.append(msgTagDetection)
    tagDetectionPub.publish(msgTagDetectionArray)




    # draw and publish image message
    # Make sure image pub at 4 Hz
    # current_time = time.time()
    # if current_time - imageObj.lastImageMsgTime < 0.23:   return
    # imageObj.draw_contours()
    # bridge = CvBridge()
    # msgGrayImageWithMarker = bridge.cv2_to_imgmsg(
    #     imageObj.grayImageWithMarker, encoding="passthrough")   # mono8
    # imagePub.publish(msgGrayImageWithMarker)
    # imageObj.lastImageMsgTime = time.time()


    # out.write(imageObj.grayImageWithMarker)


"""
Function: timer_callback
Mainly a scheduler to decide how fast to calculate and publish opencv data
"""
def timer_callback(event):

    detect_tags()
    publish_data()


"""
Function: marker_attitude_callback
Mainly get truck yaw data to calculate distance_NED
"""
# def marker_attitude_callback(data):
#     imageObj.marker_attitude_yaw = data.point.z




if __name__ == '__main__':

    # setup objects for global variables
    # cameraLensDecider is to decide which camera we are using, 
    # since different camera has different camera matrix
    imageObj = OpencvLib.OpencvLib()


    # Initial the node opencv_node
    # set the excutive rate to 25Hz
    # rate = rospy.Rate(25) # 25hz
    rospy.init_node('opencv_node', anonymous=True)


    # setup camera
    try_connect_camera()

    # setup video saver
    if ENABLE_VIDEO_RECORD:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = imageObj.capture.get(cv2.CAP_PROP_FPS)  
        size = (int(imageObj.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),   
                int(imageObj.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))  
        rawFilename = 'rawImage'+time.strftime("%H-%M-%S")+'.avi' 
        rawImOut = cv2.VideoWriter(rawFilename, fourcc, fps, size) 



    # setup publishers
    # opencvInfoPub = rospy.Publisher("/opencv/info", PoseStamped, queue_size=10)
    tagDetectionPub = rospy.Publisher("/usb_cam/tag_detections", AprilTagDetectionArray, queue_size=10)
    imagePub = rospy.Publisher("/usb_cam/grayImage_marker", Image, queue_size=1)

    # setup subscribers
    # rospy.Subscriber("/truck/attitude", PointStamped, marker_attitude_callback)
    

    # setup timer
    # Use timer interupt to make sure publish at 20.8Hz
    dTimeStep = 0.033
    rospy.Timer(rospy.Duration(dTimeStep), timer_callback)


    # deal with some left-behinds
    # After spin() no code will be excuted.
    rospy.on_shutdown(sigint_handler)
    rospy.spin()
