#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import numpy as np
import cv2
import math
import time
import marker
from pyquaternion import Quaternion
#import sys


"""
class: OpencvLib
Mainly collect the useful opencv functions to detect markers,
recognize the id, calculate the pose and position. 
"""
class OpencvLib(object):
    
    # DEBUG: Time counting gear
    # e1 = cv2.getTickCount()

    # e2 = cv2.getTickCount()
    # gap = (e2-e1)/cv2.getTickFrequency()
    # print(time.time(), gap)


    # Constructor.
    def __init__(self):

        # camera parameters
        horizontalResolution = 640
        verticalResolution = 480

        # Algorithm parameters
        self.minContourPerimeter = verticalResolution/5   
        self.minContourLengthAllowed = 100
        self.distanceToSwitchMarker = 0.7 
        # self.minContourPerimeter = verticalResolution/4   
        # self.minContourLengthAllowed = 120
        self.camMat = np.float32([   [463.6725, 0.0, 317.3370], 
                                     [0.0, 521.3451, 240.6362], 
                                     [0.0, 0.0, 1.0]             ])
        self.distortionVec = np.float32([-0.3575, 0.1281, -0.000010532, -0.0006737, 0.0])


        """
        Marker transfer flags
        """
        self.marker213 = marker.marker()
        self.marker456 = marker.marker()
        self.isMarker213Detected = False
        self.isMarker456Detected = False
        self.currentMarkerID = 213


        """
        Decide what kind of camera len and marker size we used.
        See decide_obj_points & decide_cam_lens functions for more info.
        """
        self.markerSizeDecider = 'big'


        """
        Debug data.
        """
        self.camShotTime = 0
        self.camShotTimeHMS = time.strftime("%H:%M:%S")
        self.lastImageMsgTime = 0

        
        """
        Control flags
        """
        # target lost flag
        self.targetLostCount = -1
        self.isTargetLost = True

        self.numOfMarkerDetected = 0
        self.isAnyTargetDetected = False


        """
        Estimated position & attitude data
        """
        self.distanceInCameraFrame = []
        self.eulerAngle_rads = []




        # distance_NED correction
        # if 213 is used, deducted the center difference 
        self.center_difference = 0.162


    # Use opencv built-in function to capture image.
    def connect_camera(self, indexOfCamera):
        print("trying to connect the index=%d usb port for camera." % indexOfCamera)
        self.capture = cv2.VideoCapture(indexOfCamera)


    # Since we use two markers, here is to decide which marker size we are currently using. 
    # Object points means the real marker size in real world.
    def decide_obj_points(self, decider='big'):

        if decider == 'big':
            objPoints = np.float32([    [-0.121,-0.121,0.0], 
                                        [+0.121,-0.121,0.0], 
                                        [+0.121,+0.121,0.0], 
                                        [-0.121,+0.121,0.0]     ])
            return objPoints
        elif decider == 'small':
            objPoints = np.float32([    [-0.030,-0.030,0.0], 
                                        [+0.030,-0.030,0.0], 
                                        [+0.030,+0.030,0.0], 
                                        [-0.030,+0.030,0.0]     ])
            return objPoints
        else:
            raise NameError("NO THIS SIZE OF MARKER!!")


    # Since we use two markers, here is to decide which marker size we are currently using. 
    # Image points means the marker size in the image from camera.     
    def decide_img_points(self, decider='big'):

        if decider == 'big':
            imgPoints = np.float32([    self.marker213.markerPoints[0][0], 
                                        self.marker213.markerPoints[1][0], 
                                        self.marker213.markerPoints[2][0], 
                                        self.marker213.markerPoints[3][0]   ])
            return imgPoints

        elif decider == 'small':
            imgPoints = np.float32([    self.marker456.markerPoints[0][0], 
                                        self.marker456.markerPoints[1][0], 
                                        self.marker456.markerPoints[2][0], 
                                        self.marker456.markerPoints[3][0]   ])
            return imgPoints

        else:
            raise NameError("NO THIS SIZE OF MARKER!!")



    ##################################################################################
    # Main algorithm functions starts
    ##################################################################################

    """
    Member Function: prepare_image
    Mainly save the raw image from camera as original image,
    and convert the raw image into gray scale image and binary image. 
    """
    def prepare_image(self, image):

        originalImage = image

        self.grayImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)

        # Binarization
        # Adaptive method can find the contour
        self.binImage = cv2.adaptiveThreshold(
            self.grayImage,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,7,7)
        # ret, binImage = cv2.threshold(grayImage,127,255,cv2.THRESH_BINARY_INV)



    """
    Member Function: detect_markers
    To find all the contours and delete some impossible ones from these requirments: too small, not rectangle. 
    Also checked the point order, we hope the order is anti-clockwise. 
    """
    def detect_markers(self):

        # Find all the contours
        _, allContours, _ = cv2.findContours(self.binImage,cv2.RETR_LIST, 
                                                cv2.CHAIN_APPROX_NONE)

        # Skip the small impossible contours
        indexOfPassedTestMarker = []
        possibleContours = []

        for i in range(len(allContours)):  
            real_perimeter = cv2.arcLength(allContours[i],True)  
            if real_perimeter > self.minContourPerimeter:
                indexOfPassedTestMarker.append(i)
        
        # print(indexOfPassedTestMarker)
        for i in range(len(indexOfPassedTestMarker)):
            possibleContours.append(allContours[indexOfPassedTestMarker[i]])

            

        # Find candidate marker
        # Only the ones can pass the tests
        self.detectedMarkers = []

        for i in range(len(possibleContours)): 
            epsilon = 0.05*cv2.arcLength(possibleContours[i],True)
            approxCurve = cv2.approxPolyDP(possibleContours[i],epsilon,True)
            
            if len(approxCurve) != 4:
                continue

            if cv2.isContourConvex(approxCurve) == False:
                continue

            # Ensure that the distance between consecutive points is large enough
            minDistance = 9999
            for i in range(4): 
                side = approxCurve[i][0] - approxCurve[(i+1)%4][0]
                squaredSideLength = side[0]*side[0]+side[1]*side[1]
                # print(squaredSideLength)
                minDistance = min(minDistance, squaredSideLength)

            if minDistance < self.minContourLengthAllowed:
                continue


            # Sort the points in anti-clockwise order
            # Trace a line between the first and second point.
            # If the third point is at the right side, 
            # then the points are anti-clockwise
            tempApproxCurve = np.array([[[0, 0]], [[0, 0]], [[0, 0]], [[0, 0]]])
            v1 = approxCurve[1][0] - approxCurve[0][0]
            v2 = approxCurve[2][0] - approxCurve[0][0]
            o = (v1[0]*v2[1]) - (v1[1]*v2[0])
            if o < 0:
                tempApproxCurve[0][0][0] = approxCurve[0][0][0]
                tempApproxCurve[0][0][1] = approxCurve[0][0][1]
                tempApproxCurve[1][0][0] = approxCurve[3][0][0]
                tempApproxCurve[1][0][1] = approxCurve[3][0][1]        
                tempApproxCurve[2][0][0] = approxCurve[2][0][0]
                tempApproxCurve[2][0][1] = approxCurve[2][0][1]
                tempApproxCurve[3][0][0] = approxCurve[1][0][0]
                tempApproxCurve[3][0][1] = approxCurve[1][0][1]
                self.detectedMarkers.append(tempApproxCurve)
                continue

            self.detectedMarkers.append(approxCurve)



    """
    Member Function: identify_markerID
    Mainly we use this function to identify the marker ID. Now we got 213 and 456. 
    getPerspectiveTransform and warpPerspective function are used to transform the image to front view. 
    After filter out those marker whose id = -1, we save the marker contours info into marker class. 
    """
    def identify_markerID(self):
        
        self.goodMarkers = []
        markers = []

        for i in range(len(self.detectedMarkers)):

            if self.detectedMarkers[i][1][0][0] == self.detectedMarkers[i][3][0][0] \
            and self.detectedMarkers[i][1][0][1] == self.detectedMarkers[i][3][0][1]:
                continue

            markerPoints = np.float32([     self.detectedMarkers[i][3][0], 
                                            self.detectedMarkers[i][0][0], 
                                            self.detectedMarkers[i][1][0], 
                                            self.detectedMarkers[i][2][0]   ])
            markerSize = (105, 105)
            markerCorners2d = np.float32([  [0,0], 
                                            [markerSize[0]-1,0], 
                                            [markerSize[0]-1,markerSize[1]-1], 
                                            [0,markerSize[1]-1]     ])

            markerTransform = cv2.getPerspectiveTransform(markerPoints,markerCorners2d)
            tempCanonicalMarkerImage = cv2.warpPerspective( self.grayImage, 
                                                            markerTransform, 
                                                            markerSize)

            tempMarkerID, tempRotations = self.get_markerID(tempCanonicalMarkerImage)

            if tempMarkerID == -1: 
                continue 

            # rotate the marker to the oriented direction
            markers = self.detectedMarkers[i]
            markers = self.rotate_marker(markers, tempRotations)

            # save the real marker and its info
            if tempMarkerID == 213:
                self.isMarker213Detected = True
                self.marker213.saveMarker(markers, tempMarkerID)
                self.goodMarkers.append(markers)
            elif tempMarkerID == 456:
                self.isMarker456Detected = True
                self.marker456.saveMarker(markers, tempMarkerID)
                self.goodMarkers.append(markers)
            else: 
                continue


    """
    Member Function: estimate_position_pose
    To calculate the position and attitude of marker, first we get relative distance in camera frame 
    and then rotate into target frame and then NED frame based on relative heading angle between target and camera. 
    Then we calculate the makrer local location in planner.py. 
    """
    def estimate_position_pose(self):

        # If detect any markers, then clear all the properties and re-give values
        # Otherwise just stay the old ones
        self.numOfMarkerDetected = len(self.goodMarkers)
        self.isAnyTargetDetected = self.numOfMarkerDetected > 0
        if not self.isAnyTargetDetected:

            if self.targetLostCount >= 0:
                self.targetLostCount = self.targetLostCount + 1

                if self.targetLostCount == 3:
                    self.isTargetLost = True
                    self.targetLostCount = -1

            return


        # initialize the failure handler parameters
        self.targetLostCount = 0
        self.isTargetLost = False


        # Switch marker decision part
        if self.distance_camera_z == 0:
            if self.isMarker456Detected:
                self.currentMarkerID = 456
            else:
                self.currentMarkerID = 213
        elif self.distance_camera_z > self.distanceToSwitchMarker:
            if self.isMarker213Detected:
                self.currentMarkerID = 213
            else:
                self.currentMarkerID = 456
        else:
            if self.isMarker456Detected:
                self.currentMarkerID = 456
            else:
                self.currentMarkerID = 213

        # If deicde, initialize obj/img points
        if self.currentMarkerID == 213:
            self.markerSizeDecider = 'big'
        elif self.currentMarkerID == 456:
            self.markerSizeDecider = 'small'
        else:
            raise NameError("SO WE DECIDE ANOTHER MARKER???")




        for i in range(self.numOfMarkerDetected):
            
            # prepare objPoints & imgpoints
            objPoints = self.decide_obj_points(self.markerSizeDecider)
            imgPoints = self.decide_img_points(self.markerSizeDecider)

            # prepare some more about imgPoints
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            imgPoints = cv2.cornerSubPix(
                self.grayImage, imgPoints,(5,5), (-1,-1), criteria)


            # solvePnP function
            retval, temprvecs, temptvecs = cv2.solvePnP(
                objPoints, imgPoints, self.camMat, self.distortionVec)



            # if tvecs and rvecs are valid, then save and calculate Eular angles
            isCalculationReliable = temptvecs[2][0] > 0
            if not isCalculationReliable:
                continue

            # # save rvec and tvec 
            # self.rvecs.append(temprvecs)
            # self.tvecs.append(temptvecs)


            # save distance in camera frame
            # camera's right: x axis
            # camera's down: y axis
            # camera's out: z axis
            self.distanceInCameraFrame = [temptvecs[0][0], temptvecs[1][0], temptvecs[2][0]]

            rotMat = cv2.Rodrigues(temprvecs)
            self.quaternion = Quaternion(matrix=rotMat)


            # # get rotation values for calculation
            # r11 = rotMat[0][0][0]
            # r12 = rotMat[0][0][1]
            # r13 = rotMat[0][0][2]
            # r21 = rotMat[0][1][0]
            # r22 = rotMat[0][1][1]
            # r23 = rotMat[0][1][2]
            # r31 = rotMat[0][2][0]
            # r32 = rotMat[0][2][1]
            # r33 = rotMat[0][2][2]


            # # # Calculate Euler angles. Order is z-y-x. Intrinsic rotation
            # yaw_rads = math.atan2(r21, r11)
            # pitch_rads = math.atan2(-1*r31, math.sqrt(r32*r32+r33*r33))
            # roll_rads = math.atan2(r32, r33)

            # self.eulerAngle_rads = [roll_rads, pitch_rads, yaw_rads]


            self.isMarker213Detected = False
            self.isMarker456Detected = False


    ##################################################################################
    # Main algorithm functions ends
    ##################################################################################


    # Method to calculate marker ID. 
    # First we divide the marker part into 7*7 cube. Calculate the number of zeros(black), and calculate 
    # the distance. Rotate the marker until we find the min distance. Then we calculate the ID. 
    def get_markerID(self, tempCanonicalMarkerImage):
        
        grayImage = tempCanonicalMarkerImage

        ret, binImage = cv2.threshold(grayImage,125,255,
            cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        cellSize = 15

        for y in range(7):
            inc = 6
            if y == 0 or y == 6: 
                inc = 1

            for x in range(0,7,inc):
                cellX = x * cellSize
                cellY = y * cellSize
                cell = binImage[cellX:cellX+cellSize, cellY:cellY+cellSize]

                nZ = cv2.countNonZero(cell)
                if nZ > cellSize*cellSize/2:
                    return -1, -1

        bitMatrix = [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]

        for x in range(5):
            for y in range(5):
                cellX = (x+1) * cellSize
                cellY = (y+1) * cellSize
                cell = binImage[cellX:cellX+cellSize, cellY:cellY+cellSize]
                
                nZ = cv2.countNonZero(cell)
                if nZ > cellSize*cellSize/2:
                    bitMatrix[x][y] = 1



        rotations = [[[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
                    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
                    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
                    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]]
        distances = [0,0,0,0]

        rotations[0] = bitMatrix  
        distances[0] = self.hamm_dist_marker(rotations[0])

        minDist = [distances[0], 0]

        for i in range(1,4):
            rotations[i] = self.rotate(rotations[i-1])
            distances[i] = self.hamm_dist_marker(rotations[i])

            if distances[i] < minDist[0]:
                minDist[0] = distances[i]
                minDist[1] = i

        nRotations = minDist[1]
        if minDist[0] == 0:
            return self.mat2id(rotations[minDist[1]]), nRotations

        return -1, -1


    # Method to rotate matrix. The effect equals to transposition.
    def rotate(self, inMatrix):
        
        outMatrix = [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
        for i in range(5):
            for j in range(5):
                outMatrix[i][j] = inMatrix[5-j-1][i]

        return outMatrix


    # Method to rotate marker to align with target frame. Basically it's the same idea with the rotate matrix method.
    # For example, nRotations = 2, rotate the marker points anti-clockwisely 90 degree for 2 time.
    def rotate_marker(self, inMarker, nRotations):

        outMarker = np.array([[[0, 0]], [[0, 0]], [[0, 0]], [[0, 0]]])
        
        if nRotations == 0:
            outMarker[0][0][0] = inMarker[0][0][0]
            outMarker[0][0][1] = inMarker[0][0][1]
            outMarker[3][0][0] = inMarker[3][0][0]
            outMarker[3][0][1] = inMarker[3][0][1]        
            outMarker[2][0][0] = inMarker[2][0][0]
            outMarker[2][0][1] = inMarker[2][0][1]
            outMarker[1][0][0] = inMarker[1][0][0]
            outMarker[1][0][1] = inMarker[1][0][1]
            return outMarker

        if nRotations == 1:
            outMarker[0][0][0] = inMarker[3][0][0]
            outMarker[3][0][0] = inMarker[2][0][0]
            outMarker[2][0][0] = inMarker[1][0][0]
            outMarker[1][0][0] = inMarker[0][0][0]
            outMarker[0][0][1] = inMarker[3][0][1]
            outMarker[3][0][1] = inMarker[2][0][1]
            outMarker[2][0][1] = inMarker[1][0][1]
            outMarker[1][0][1] = inMarker[0][0][1]

            return outMarker

        if nRotations == 2:
            outMarker[0][0][0] = inMarker[2][0][0]
            outMarker[1][0][0] = inMarker[3][0][0]
            outMarker[2][0][0] = inMarker[0][0][0]
            outMarker[3][0][0] = inMarker[1][0][0]
            outMarker[0][0][1] = inMarker[2][0][1]
            outMarker[1][0][1] = inMarker[3][0][1]
            outMarker[2][0][1] = inMarker[0][0][1]
            outMarker[3][0][1] = inMarker[1][0][1]

            return outMarker

        if nRotations == 3:
            outMarker[0][0][0] = inMarker[1][0][0]
            outMarker[1][0][0] = inMarker[2][0][0]
            outMarker[2][0][0] = inMarker[3][0][0]
            outMarker[3][0][0] = inMarker[0][0][0]
            outMarker[0][0][1] = inMarker[1][0][1]
            outMarker[1][0][1] = inMarker[2][0][1]
            outMarker[2][0][1] = inMarker[3][0][1]
            outMarker[3][0][1] = inMarker[0][0][1]

            return outMarker

  
    # Calculate the hamming distance of markers.
    def hamm_dist_marker(self, inMatrix):
        
        ids = [[1,0,0,0,0],[1,0,1,1,1],[0,1,0,0,1],[0,1,1,1,0]]
        dist = 0

        for y in range(5):
            minSum = 100000
            for p in range(4):
                Sum = 0
                for x in range(5):
                    if inMatrix[y][x] == ids[p][x]:
                        Sum = Sum + 0
                    else:
                        Sum = Sum + 1

                if minSum > Sum:
                    minSum = Sum

            dist = dist + minSum

        return dist


    # Calculate the id of the code inside marker. 
    def mat2id(self, inMatrix):
        
        val = 0
        for i in range(5):
            val = val << 1
            if inMatrix[i][1] == 1:
                val = val | 1
            val = val << 1
            if inMatrix[i][3] == 1:
                val = val | 1    
                
        return val


    # Draw contours alone marker, and put timestamp on each frame.
    def draw_contours(self):
  
        self.grayImageWithMarker = cv2.drawContours(self.grayImage, 
                                                    self.goodMarkers, 
                                                    -1, (0,255,0), 10)

        # draw time stamp on image
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = self.camShotTimeHMS
        cv2.putText(self.grayImageWithMarker,text,(400,50), font, 1,(255,255,255),3)








    




