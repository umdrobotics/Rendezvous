#!/usr/bin/env python

import math

def degsToRads(degrees):
    return math.pi*degrees/180.0

def radsToDegs(radians):
    return 180.0*radians/math.pi


def getCameraPlane(altitude, FOV_deg=94.0 ):
    thetaDeg = FOV_deg/2.0
    theta = degsToRads(thetaDeg) #this is the angle that defines the "cone" of the camera
    #the camera x and y boundaries can each be defined using a cone, and then the image itself is a rectangle defined by those bounds
             # (theta)
            # /|
           # / |
          # /  |
         # /   | object distance
        # /    |  
       # /     | 
      # /      |
     # /       |
    # /-----------
    # field of vision (1 side, actual field is twice this much)

    # so the boundary on this is objectDistance*tan(theta), and the boundary is from -objectDistance*tan(theta) to objectDistance*tan(theta)


    yBoundsCamera = [altitude*-1.0*math.tan(theta),altitude*math.tan(theta) ] 
    xBoundsCamera = [altitude*-1.0*math.tan(theta),altitude*math.tan(theta) ]

    return[xBoundsCamera , yBoundsCamera, "1st element = x bounds, 2nd = y bounds, meters"]
    

def getCameraBoundsInertial(roll_rads, pitch_rads, yaw_rads, quadNorth, quadEast, altitude):
     roll=roll_rads
     pitch=pitch_rads
     yaw=yaw_rads
     
     #so now we need to convert the camera FOV to boundaries in an inertial plane.
     #To do so, we'll need to know the current camera position and rotation, and the transform matrix of the camera
     transformMatrix =  [ [0,0,1], [1,0,0], [0,-1,0] ]
     #now, assuming roll is 0:
     # The camera boundaries will change depending on the camera pitch and yaw
     # Example: If camera is pointing straight ahead (pitch 0, yaw 0), y boundaries on camera are z boundaries  on inertial frame, x bounds are y bounds on inertial frame
     # If camera is pointing straight down (pitch -90, yaw 0) then y boundaries on camera are x boundaries on inertial frame, x bounds are y bounds on inertial frame
     # If camera is pointing to the right (pitch 0, yaw 90)  then y boundaries are z boundaries on inertial frame, x bounds are x bounds on inertial frame
     # Note that if camera is pointing straight down, yaw still matters
     # Ex: if pitch is -90 and yaw is 90, then y boundaries are y boundaries on inertial frame, x boundaries are x on inertial frame
     # So it appears we can use the rotation matrix again

     cameraRotationMatrix = [
            [   math.cos(yaw)*math.cos(pitch),
                 math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll), 
                math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)
            ],
            [  
                math.sin(yaw)*math.cos(pitch),
                math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll),
                math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll) 
            ],
            [
             -1.0*math.sin(pitch),
            math.cos(pitch)*math.sin(roll),
            math.cos(pitch)*math.cos(roll)
            ],
        ];
     
     boundsCameraFrame = getCameraPlane(altitude)
     xBoundsCameraFrame = boundsCameraFrame[0]
     yBoundsCameraFrame = boundsCameraFrame[1]
     
     #There's not a z boundary yet but there may be when converting, so need to create a field for that
     lowerBoundsXYZ = [[xBoundsCameraFrame[0]], [yBoundsCameraFrame[0]], [0]]
     upperBoundsXYZ = [[xBoundsCameraFrame[1]], [yBoundsCameraFrame[1]], [0]]
     
     lowerBoundsXYZinertial = [[0],[0],[0]]
     upperBoundsXYZinertial = [[0],[0],[0]]
     
     for row in range(0, 3):
        sumLower = 0.0;
        sumUpper = 0.0
        for  column in range(0 ,3):
            sumLower += transformMatrix[row][column] * lowerBoundsXYZ [column][0]; 
            sumUpper += transformMatrix[row][column] * upperBoundsXYZ [column][0]; 

        
        lowerBoundsXYZinertial[row][0] = sumLower;
        upperBoundsXYZinertial[row][0] = sumUpper
    
     
     #If we have a negative bounds, and a positive bounds, and apply the rotation matrix to it, that will convert the offset to inertial frame
     #But we also need to know where the center of the bounds is in inertial frame (ie, what point does 0,0 on the camera correspond to on inertial frame?)
     #Assuming target is roughly ground level, that can be solved with simple trig
     # 
     #  Copter: x_______________
      #         |\ (pitch)
    #           | \
    #           |  \
    #   Altitude|   \
    #           |    \
    #           |     \
    #           |      \
    #           |       \
     #          __________
      #          targetDistance ( = displacement2D)
    #  Ok, so displacement2D = altitude*tan(theta), where theta=90-pitch 
    #  And then from that, 
     
    #              _____ Y
    #             |   /
    #           X |  / displacement2D
    #             | /  
    #             |/_____________________(90 - yaw)
    #     Copter: x 
    #  clearly, deltaX = displacement2D * cos(yaw), deltaY = displacement2D*sin(yaw)
    
    #prevent getting an infinity in here
     if ( (degsToRads(-0.1) < pitch) & (pitch < degsToRads(0.1))  ):
        if(pitch > 0):
            pitch = degsToRads(0.1)
        else:
            pitch = degsToRads(-0.1)

     displacement2D = math.tan(-1.0 * pitch) * altitude
    
     deltaX = displacement2D * math.cos(yaw)
     deltaY = displacement2D * math.sin(yaw)
     centerX = quadNorth + deltaX
     centerY = quadEast + deltaY
    
     yBoundsInertial = [centerY +lowerBoundsXYZinertial[1][0], centerY +upperBoundsXYZinertial[1][0] ]
     xBoundsInertial = [centerX+lowerBoundsXYZinertial[0][0], centerX +upperBoundsXYZinertial[0][0] ]
    
     cameraBoundsInertial = [xBoundsInertial, yBoundsInertial, "1st element: x bounds, 2nd element: y bounds, both in meters on inertial frame"]
     return cameraBoundsInertial
    
#This assumes that the target is actually in the camera's field of view. You'll need to calculated whether it is or not elsehwere
def calculateAprilTagDetection(roll_rads, pitch_rads, yaw_rads, quadNorth, quadEast, altitude, targetNorth, targetEast):
        yaw = yaw_rads; roll=roll_rads; pitch=pitch_rads;
    #knowing camera rotation matrix and transform matrix, camera position, and target position, need to figure out what the displacement is in the camera frame 
    #Assuming target is roughly at ground level, Z displacement(camera frame) is just  the total camera distance to ground
        deltaNorth = targetNorth - quadNorth;
        deltaEast = targetEast - quadEast;
        displacement2D = math.sqrt( deltaEast * deltaEast  +  deltaNorth * deltaNorth)
        cameraZ = math.sqrt( altitude * altitude   +   displacement2D * displacement2D) # this serves as a sanity check on the camera Z calculation done later

    #disp = rotation*transform*cameraDisp
    #transform^-1*rotation^-1*disp = cameraDisp  
    #transform^-1 is [ [0,1,0] [0,0,-1] [1,0,0] ]
    #so let's just convert the camera plane into inertial coords, and then we can figure out the camera displacement from there
        
        a = math.cos(yaw)
        b = math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll)
        c = math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)
        d = math.sin(yaw)*math.cos(pitch)
        e = math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll)
        f = math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll) 
        g = -1.0*math.sin(pitch)
        h = math.cos(pitch)*math.sin(roll)
        i = math.cos(pitch)*math.cos(roll)
        cameraRotationMatrix = [
            [   a,
                 b, 
                c
            ],
            [  
                d,
                e,
                f 
            ],
            [
             g,
            h,
            i
            ],
        ];
        
    # so if we do, in matlab:
    #  syms a b c d e f g h id
    #  m = [a,b,c; d,e,f; g,h,i]
    #  v = m^-1
    # result is 
        """
     v =
         [  (e*i - f*h)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), -(b*i - c*h)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g),  (b*f - c*e)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g)]
         [ -(d*i - f*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g),  (a*i - c*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), -(a*f - c*d)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g)]
         [  (d*h - e*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), -(a*h - b*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g),  (a*e - b*d)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g)]

        """
    #so there's our inverse 
        invRow1 = [(e*i - f*h)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), -(b*i - c*h)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), (b*f - c*e)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g)   ]
        invRow2 = [-(d*i - f*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g),(a*i - c*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), -(a*f - c*d)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g)   ]
        invRow3 = [ (d*h - e*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g), -(a*h - b*g)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g),  (a*e - b*d)/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) ]
        inverseRotationMatrix = [invRow1, invRow2, invRow3] 
    
        inverseTransformMatrix =  [0,1,0] [0,0,-1] [1,0,0] # converts from inertial frame to camera frame, neglecting the rotation matrix
    
    
    #remember, transform^-1*rotation^-1*disp = cameraDisp
        dispInertial = [[deltaNorth], [deltaEast], [altitude] ]
        tempMatrix = [[0] [0] [0]] #handle multiplication of dispInertial with rotation^-1
        for row in range(0, 3):
            sum = 0.0
            for  column in range(0 ,3):
                sum += inverseRotationMatrix[row][column] * dispInertial [column][0]; 

        
            tempMatrix[row][0] = sum
    
    # now finish the multiplication through multiplying by inverse transform matrix
        cameraDisplacement = [[0] [0] [0]]
        for row in range(0, 3):
            sum = 0.0
            for  column in range(0 ,3):
                sum += inverseTransformMatrix[row][column] * tempMatrix [column][0]; 

            cameraDisplacement[row][0] = sum    
    
    #convert back to a row vector instead of a column vector before returning, for simplicity 
        cameraDisplacementSimplified = [cameraDisplacement[0][0], cameraDisplacement[1][0],cameraDisplacement[2][0] ]
        return cameraDisplacementSimplified 

    
def getApriltagDetection(roll_rads, pitch_rads, yaw_rads, quadNorth, quadEast, altitude, targetNorth, targetEast):
    cameraBoundsInertial = getCameraBoundsInertial(roll_rads, pitch_rads, yaw_rads, quadNorth, quadEast, altitude)
    xBounds = cameraBoundsInertial[0]
    yBounds = cameraBoundsInertial[1]
    
    
    inXBounds = ( (xBounds[0] < targetNorth) & (targetNorth < xBounds[1]) )
    inYBounds = ( (yBounds[0] < targetEast) & (targetEast < yBounds[1]) )
    if( inXBounds & inYBounds ):
        #need to figure out what the x y and z coordinates should be, IN THE CAMERA FRAME
        detectionCoords = calculateAprilTagDetection(roll_rads, pitch_rads, yaw_rads, quadNorth, quadEast, altitude, targetNorth, targetEast)
        return detectionCoords 
    else:
        blankDetection = []
        return blankDetection
    
    






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

def maketag(x ,y ,z ): #to use, do something like tagCoords = getAprilTagDetection(arguments here); myTag = maketag(tagCoords[0], tagCoords[1], tagCoords[2])
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
