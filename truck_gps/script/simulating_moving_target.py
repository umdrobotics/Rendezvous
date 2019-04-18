#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Create by Cong Zhang in 06/05/2018
import time
import math 
import numpy as np
import sys

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped


class Truck:

    LAT_PER_NORTH = 0.0000089354
    LON_PER_EAST = 0.0000121249
    
    def __init__(self,isEnableNoise = True, timeStep = 0.1, timeStepTag = 0.1, gpsNoiseVariance = 0.3, velocityNoiseVariance = 0.18, tagNoiseVariance = 0.1):
        # inital the class

        self.truckGpsPub =            rospy.Publisher("/truck/location_GPS", PoseStamped, queue_size=10)
        self.truckVelocityPub =       rospy.Publisher("/truck/velocity", PointStamped, queue_size=10)
        self.truckRealPositionPub =   rospy.Publisher("/truck/real_location_GPS", PointStamped, queue_size=10)
        self.StartSimulationPub =     rospy.Publisher("/truck/start_simulation", PointStamped, queue_size=10)
        self.aprilTagPub =            rospy.Publisher("/truck/tag_detections", PoseStamped, queue_size=10)
        
        self.isEnableNoise = isEnableNoise 
        self.timeStep = timeStep
        self.timeStepTag = timeStepTag
        self.turning_case = 1  # for circle path: 1. move to x first, 2. move to y first
       
        self.gpsNoiseVariance = gpsNoiseVariance
        self.velocityNoiseVariance = velocityNoiseVariance
        self.tagNoiseVariance = tagNoiseVariance

        # self.isPublishStart = False


    def SetSimulationPath(self, path_number, simLatitudeStart, simLongitudeStart, variable_x = 0, variable_y = 0):

        # simPathName is for the future use.
        self.simPathNumber = path_number
        # self.simLatitudeStart = simLatitudeStart
        # self.simLongitudeStart = simLongitudeStart
        self.variable_x = variable_x
        self.variable_y = variable_y
        self.time = 0
        self.timeTag = 0
        self.turning_time = variable_y
        self.simLatitudeStart = simLatitudeStart
        self.simLongitudeStart = simLongitudeStart
		
        self.simStart = PointStamped()
        self.simStart.point.x = 22
       
        self.simGlobalLocation = PoseStamped()
        self.simGlobalLocation.pose.position.x = simLatitudeStart
        self.simGlobalLocation.pose.position.y = simLongitudeStart
        self.simGlobalLocation.pose.orientation.x = simLatitudeStart   # latitude
        self.simGlobalLocation.pose.orientation.y = simLongitudeStart  # longitude
     
        self.simVehicleVelocity = PointStamped()
        self.simVehicleVelocity.point.x = 0                 # north speed
        self.simVehicleVelocity.point.y = 0                 # east speed
        self.simVehicleVelocity.point.z = 0                 # down speed

        self.simTagLocation = PoseStamped()
        self.simTagLocation.pose.position.x = 0
        self.simTagLocation.pose.position.y = 0
        self.simTagLocation.pose.orientation.x = 0   # latitude
        self.simTagLocation.pose.orientation.y = 0
        
        # speed = math.sqrt((speed_north*speed_north)+(speed_east*speed_east))


    def PublishSimulationMsg(self):

        # Inital ros messages and publish it
        self.simGlobalLocation.header.stamp = rospy.Time.now()   
        self.simVehicleVelocity.header.stamp = rospy.Time.now()
        self.simTagLocation.header.stamp = rospy.Time.now() 

        self.truckGpsPub.publish(self.simGlobalLocation)
        self.truckVelocityPub.publish(self.simVehicleVelocity)
        # self.aprilTagPub.publish(self.simTagLocation)

        #~ self.StartSimulationPub.publish(self.simStart)
        if self.simPathNumber == 1:


                self.simGlobalLocation.pose.orientation.x += (self.variable_x * self.timeStep) * Truck.LAT_PER_NORTH 
                self.simGlobalLocation.pose.orientation.y += (self.variable_y * self.timeStep) * Truck.LON_PER_EAST 

                self.simGlobalLocation.pose.position.x = self.simGlobalLocation.pose.orientation.x
                self.simGlobalLocation.pose.position.y = self.simGlobalLocation.pose.orientation.y


                self.simVehicleVelocity.point.x = self.variable_x        # north speed
                self.simVehicleVelocity.point.y = self.variable_y        # east speed

       
                self.time += self.timeStep
                # self.logger.debug("Simulation: send the truck data")            

        elif self.simPathNumber == 2: 
            if self.time > 180:
                self.time = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * (1.0 - math.cos(omega*self.time))
                distance_y = self.variable_y * math.sin(omega*self.time)
                last_x = self.simGlobalLocation.pose.position.x
                last_y = self.simGlobalLocation.pose.position.y

                self.simGlobalLocation.pose.position.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.pose.position.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart

                self.simGlobalLocation.pose.orientation.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.pose.orientation.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart

                angle = math.atan2((self.simGlobalLocation.pose.position.y - last_y), (self.simGlobalLocation.pose.position.x - last_x))
                self.simVehicleVelocity.point.x = self.variable_x * math.cos(angle)       # north speed
                self.simVehicleVelocity.point.y = self.variable_x * math.sin(angle)       # east speed
                # self.logger.debug("Simulation: send the truck data")            
                self.time += self.timeStep

        elif self.simPathNumber == 3:
            if self.time > 180:
                self.time = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                if self.turning_case == 1:
                    distance_x = self.variable_x * self.timeStep
                    distance_y = 0.0
                    self.simVehicleVelocity.point.x = self.variable_x        # north speed
                    self.simVehicleVelocity.point.y = 0                      # east speed
                    if self.turning_time <= self.time:
                        self.turning_time = self.time + self.variable_y
                        self.turning_case = 2
                        # self.count1 += 1

                elif self.turning_case == 2:
                    distance_x = 0.0
                    distance_y = self.variable_x * self.timeStep
                    self.simVehicleVelocity.point.x = 0                     # north speed
                    self.simVehicleVelocity.point.y = self.variable_x       # east speed
                    if self.turning_time <= self.time:
                        self.turning_time = self.time + self.variable_y
                        self.turning_case = 1
                        # self.count2 += 1

                self.simGlobalLocation.pose.orientation.x += distance_x * Truck.LAT_PER_NORTH 
                self.simGlobalLocation.pose.orientation.y += distance_y * Truck.LON_PER_EAST
                self.simGlobalLocation.pose.x = self.simGlobalLocation.pose.orientation.x
                self.simGlobalLocation.pose.y = self.simGlobalLocation.pose.orientation.y

                # self.logger.debug("Simulation: send the truck data")            
                self.time += self.timeStep

        elif self.simPathNumber == 4:
            if self.time > 180:
                self.time = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * (1.0 - math.cos(omega*self.time))
                distance_y = self.variable_y * math.sin(2*omega*self.time)
                last_x = self.simGlobalLocation.pose.x
                last_y = self.simGlobalLocation.pose.y
                
                self.simGlobalLocation.pose.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.pose.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
                self.simGlobalLocation.pose.orientation.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.pose.orientation.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
               
                angle = math.atan2((self.simGlobalLocation.point.y - last_y), (self.simGlobalLocation.point.x - last_x))
                self.simVehicleVelocity.point.x = self.variable_x * math.cos(angle)       # north speed
                self.simVehicleVelocity.point.y = self.variable_x * math.sin(angle)       # east speed
                # self.logger.debug("Simulation: send the truck data")            
                self.time += self.timeStep

        elif self.simPathNumber == 5:
            if self.time > 180:
                self.time = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * math.sin(omega*self.time + math.sin(omega*self.time)) 
                distance_y = self.variable_y * (math.cos(omega*self.time + math.cos(omega*self.time)) - math.cos(1))
                last_x = self.simGlobalLocation.pose.x
                last_y = self.simGlobalLocation.pose.y
               
                self.simGlobalLocation.pose.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.pose.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
               
                self.simGlobalLocation.pose.orientation.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.pose.orientation.x = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
                              
                angle = math.atan2((self.simGlobalLocation.point.y - last_y), (self.simGlobalLocation.point.x - last_x))
                self.simVehicleVelocity.point.x = self.variable_x * math.cos(angle)       # north speed
                self.simVehicleVelocity.point.y = self.variable_x * math.sin(angle)       # east speed
                
                # self.logger.debug("Simulation: send the truck data")            
                self.time += self.timeStep
        else:
            print("Wrong path!!! Can not update the simulation path!!")


        if self.isEnableNoise:
            self.simGlobalLocation.pose.position.x += (np.random.normal(0, self.gpsNoiseVariance))*Truck.LAT_PER_NORTH
            self.simGlobalLocation.pose.position.y += (np.random.normal(0, self.gpsNoiseVariance))*Truck.LON_PER_EAST
            
            self.simVehicleVelocity.point.x += np.random.normal(0, self.velocityNoiseVariance)      # north speed
            self.simVehicleVelocity.point.y += np.random.normal(0, self.velocityNoiseVariance)       # east speed



    def PublishSimulationTagMsg(self):

        # Inital ros messages and publish it
        self.simTagLocation.header.stamp = rospy.Time.now() 
        self.aprilTagPub.publish(self.simTagLocation)

        #~ self.StartSimulationPub.publish(self.simStart)
        if self.simPathNumber == 1:

                self.simTagLocation.pose.orientation.x += (self.variable_x * self.timeStepTag) 
                self.simTagLocation.pose.orientation.y += (self.variable_y * self.timeStepTag) 

                self.simTagLocation.pose.position.x = self.simTagLocation.pose.orientation.x
                self.simTagLocation.pose.position.y = self.simTagLocation.pose.orientation.y


                self.timeTag += self.timeStepTag
                # self.logger.debug("Simulation: send the truck data")            

        elif self.simPathNumber == 2: 
            if self.timeTag > 180:
                self.timeTag = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * (1.0 - math.cos(omega*self.timeTag))
                distance_y = self.variable_y * math.sin(omega*self.timeTag)
 
                self.simTagLocation.pose.orientation.x = distance_x
                self.simTagLocation.pose.orientation.y = distance_y

                self.simTagLocation.pose.position.x = self.simTagLocation.pose.orientation.x
                self.simTagLocation.pose.position.y = self.simTagLocation.pose.orientation.y

       # east speed
                # self.logger.debug("Simulation: send the truck data")            
                self.timeTag += self.timeStepTag

        elif self.simPathNumber == 3:
            if self.timeTag > 180:
                self.timeTag = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                if self.turning_case == 1:
                    distance_x = self.variable_x * self.timeStepTag
                    distance_y = 0.0
                    self.simVehicleVelocity.point.x = self.variable_x        # north speed
                    self.simVehicleVelocity.point.y = 0                      # east speed
                    if self.turning_time <= self.timeTag:
                        self.turning_time = self.timeTag + self.variable_y
                        self.turning_case = 2
                        # self.count1 += 1

                elif self.turning_case == 2:
                    distance_x = 0.0
                    distance_y = self.variable_x * self.timeStepTag
                    self.simVehicleVelocity.point.x = 0                     # north speed
                    self.simVehicleVelocity.point.y = self.variable_x       # east speed
                    if self.turning_time <= self.timeTag:
                        self.turning_time = self.timeTag + self.variable_y
                        self.turning_case = 1
                        # self.count2 += 1


                self.simTagLocation.pose.orientation.x += distance_x
                self.simTagLocation.pose.orientation.y += distance_y

                self.simTagLocation.pose.position.x = self.simTagLocation.pose.orientation.x
                self.simTagLocation.pose.position.y = self.simTagLocation.pose.orientation.y


                # self.logger.debug("Simulation: send the truck data")            
                self.timeTag += self.timeStepTag

        elif self.simPathNumber == 4:
            if self.timeTag > 180:
                self.timeTag = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * (1.0 - math.cos(omega*self.timeTag))
                distance_y = self.variable_y * math.sin(2*omega*self.timeTag)

                self.simTagLocation.pose.orientation.x = distance_x
                self.simTagLocation.pose.orientation.y = distance_y

                self.simTagLocation.pose.position.x = self.simTagLocation.pose.orientation.x
                self.simTagLocation.pose.position.y = self.simTagLocation.pose.orientation.y
                
                # self.logger.debug("Simulation: send the truck data")            
                self.timeTag += self.timeStepTag

        elif self.simPathNumber == 5:
            if self.timeTag > 180:
                self.timeTag = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * math.sin(omega*self.timeTag + math.sin(omega*self.timeTag)) 
                distance_y = self.variable_y * (math.cos(omega*self.timeTag + math.cos(omega*self.timeTag)) - math.cos(1))

                self.simTagLocation.pose.orientation.x = distance_x
                self.simTagLocation.pose.orientation.y = distance_y

                self.simTagLocation.pose.position.x = self.simTagLocation.pose.orientation.x
                self.simTagLocation.pose.position.y = self.simTagLocation.pose.orientation.y                
                            
                # self.logger.debug("Simulation: send the truck data")            
                self.timeTag += self.timeStepTag
        else:
            print("Wrong path!!! Can not update the simulation path!!")


        if self.isEnableNoise:
            self.simTagLocation.pose.position.x += (np.random.normal(0, self.tagNoiseVariance))
            self.simTagLocation.pose.position.y += (np.random.normal(0, self.tagNoiseVariance))
            
    def TimerCallback(self, event):     
        self.PublishSimulationMsg()

    def TimerCallbackTag(self, event):     
        self.PublishSimulationTagMsg()


def main():
    if len(sys.argv) > 1:        
        truck = Truck(timeStep = float(sys.argv[1]),timeStepTag = float(sys.argv[2]), gpsNoiseVariance = float(sys.argv[3]), velocityNoiseVariance = float(sys.argv[4]), tagNoiseVariance = float(sys.argv[5]))
        timeStep = float(sys.argv[1])
        timeStepTag = float(sys.argv[2])
    else:
        truck = Truck()
        timeStep = 0.1
        timeStepTag = 0.05

    # # GPS location of Simulahstion 
    simLatitudeStart = 42.323396612189850
    simLongitudeStart = -83.222952844799720 
    # Set normal distribution variance of noise and the msg update rate


    # SetSimulationPath has no effect if isSimulation is False
    path = raw_input("Please enter the truck path:\n 1. Straight_line \n 2. Circle \n 3. Zigzag \n 4. Figure_eight \n 5. Arrow_heading \n")

    if path == "1":
        print("Straight_line path!!!")
        variable_x = raw_input("Please enter north velocity:\n")
        variable_y  = raw_input("Please enter east velocity:\n")
    elif path == "2":
        print("Circle path!!!")
        variable_x = raw_input("Please enter the line velocity of the circle:\n")
        variable_y = raw_input("Please enter the radius of the circle:\n")
    elif path == "3":
        print("Zigzag path!!")
        variable_x = raw_input("Please enter the line velocity of the Zigzag path: \n")
        variable_y  = raw_input("Please enter the turning time: \n")
    elif path == "4":
        print("Figure_eight path!!")
        variable_x = raw_input("Please enter the line velocity of the Figure_eight path: \n")
        variable_y  = raw_input("Please enter the radius of Figure_eight path: \n")
    elif path == "5":
        print("Arrow_heading path!!")
        variable_x = raw_input("Please enter the line velocity of the Arrow_heading path: \n")
        variable_y  = raw_input("Please enter the radius of Arrow_heading path: \n")

    else:
        print("Wrong input, please try again!!")
        pause()

    truck.SetSimulationPath(float(path), simLatitudeStart, simLongitudeStart, variable_x = float(variable_x), variable_y = float(variable_y))

    rospy.init_node('truck', anonymous=True)


    rospy.Timer(rospy.Duration(timeStep), truck.TimerCallback)
    rospy.Timer(rospy.Duration(timeStepTag), truck.TimerCallbackTag)
    # rospy.on_shutdown(truck.sigint_handler)
    rospy.spin()

 
if __name__ == '__main__':
    
    main()

