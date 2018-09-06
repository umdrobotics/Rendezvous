#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math 
import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped

# import logging
# import sys
# from io import FileIO, BufferedWriter



class Truck:

    LAT_PER_NORTH = 0.0000089354
    LON_PER_EAST = 0.0000121249
    
    TimeStep = 0.1
    isEnableNoise = True

    def __init__(self, bIsSimulationMode = True):
        # inital the class

        self.truckGpsPub =            rospy.Publisher("/truck/location_GPS", PoseStamped, queue_size=10)
        self.truckVelocityPub =       rospy.Publisher("/truck/velocity", PointStamped, queue_size=10)
        self.truckRealPositionPub=    rospy.Publisher("/truck/real_location_GPS", PointStamped, queue_size=10)
        self.StartSimulationPub=    rospy.Publisher("/truck/start_simulation", PointStamped, queue_size=10)
        self.isSimulationMode = bIsSimulationMode 
        self.timeStep = 0.1
        self.turning_case = 1 
        self.noiseRange = 0.3 
        self.count1 = 0
        self.count2 = 0
        self.isPublishStart = False
        # self.LoggerWarningLevel = logging.DEBUG
        # # self.LoggerWarningLevel = logging.INFO
        # #self.LoggerWarningLevel = logging.WARNING
        # #self.LoggerWarningLevel = logging.ERROR

        # self.logger = logging.getLogger('Truck_message')
        # self.fileHandler_message = logging.StreamHandler(BufferedWriter(FileIO("truck_message" + time.strftime("%Y%m%d-%H%M%S") + ".log", "w")))
        # self.streamHandler_message = logging.StreamHandler(sys.stdout)
        # self.logger.addHandler(self.fileHandler_message)
        # self.logger.addHandler(self.streamHandler_message)

        # # record the data in log file
        # self.formatter_message = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        # # only display the data message in screen
        # self.formattter = logging.Formatter('%(message)s')

        # self.fileHandler_message.setFormatter(self.formatter_message)
        # self.streamHandler_message.setFormatter(self.formattter)
        # self.logger.setLevel(self.LoggerWarningLevel)


    def SetSimulationPath(self, path_number, simLatitudeStart, simLongitudeStart, variable_x = 0, variable_y = 0):

        # simPathName is for the future use.
        self.simPathNumber = path_number
        # self.simLatitudeStart = simLatitudeStart
        # self.simLongitudeStart = simLongitudeStart
        self.variable_x = variable_x
        self.variable_y = variable_y
        self.time = 0
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
        
        # speed = math.sqrt((speed_north*speed_north)+(speed_east*speed_east))
        # self.logger.debug("Truck simulation has started.")


    def PublishVehicleMsg(self):
        
        msgVehicleGps = self.truck.GetGlobalPosition()

        msgVehicleVelocity = self.truck.GetVelocity()

        self.truckGpsPub.publish(msgVehicleGps)
        self.truckVelocityPub.publish(msgVehicleVelocity)
        #~ if self.isPublishStart:
       	#~ self.StartSimulationPub.publish(self.simStart)	
            #~ self.isPublishStart = True
		    
        # self.logger.debug("send the truck data")

    def PublishSimulationMsg(self):

        self.simGlobalLocation.header.stamp = rospy.Time.now()   
        self.simVehicleVelocity.header.stamp = rospy.Time.now()
        #~ self.simRealLocation.header.stamp = rospy.Time.now()

        self.truckGpsPub.publish(self.simGlobalLocation)
        self.truckVelocityPub.publish(self.simVehicleVelocity)
        #~ self.truckRealPositionPub.publish(self.simRealLocation)
        self.StartSimulationPub.publish(self.simStart)
        if self.simPathNumber == 1:


                self.simGlobalLocation.pose.orientation.x += (self.variable_x * self.timeStep) * Truck.LAT_PER_NORTH 
                self.simGlobalLocation.pose.orientation.y += (self.variable_y * self.timeStep) * Truck.LON_PER_EAST 

                self.simGlobalLocation.pose.position.x = self.simGlobalLocation.pose.orientation.x
                self.simGlobalLocation.pose.position.y = self.simGlobalLocation.pose.orientation.y



                self.simVehicleVelocity.point.x = self.variable_x        # north speed
                self.simVehicleVelocity.point.y = self.variable_y        # east speed
                self.time += Truck.TimeStep
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
                self.time += Truck.TimeStep

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

                self.simRealLocation.point.x += distance_x * Truck.LAT_PER_NORTH 
                self.simRealLocation.point.y += distance_y * Truck.LON_PER_EAST
                self.simGlobalLocation.point.x = self.simRealLocation.point.x
                self.simGlobalLocation.point.y = self.simRealLocation.point.y


                # self.logger.debug("Simulation: send the truck data")            
                self.time += Truck.TimeStep

        elif self.simPathNumber == 4:
            if self.time > 180:
                self.time = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * (1.0 - math.cos(omega*self.time))
                distance_y = self.variable_y * math.sin(2*omega*self.time)
                last_x = self.simGlobalLocation.point.x
                last_y = self.simGlobalLocation.point.y
                
                self.simGlobalLocation.point.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.point.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart

                self.simRealLocation.point.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simRealLocation.point.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
                
                angle = math.atan2((self.simGlobalLocation.point.y - last_y), (self.simGlobalLocation.point.x - last_x))
                self.simVehicleVelocity.point.x = self.variable_x * math.cos(angle)       # north speed
                self.simVehicleVelocity.point.y = self.variable_x * math.sin(angle)       # east speed
                # self.logger.debug("Simulation: send the truck data")            
                self.time += Truck.TimeStep

        elif self.simPathNumber == 5:
            if self.time > 180:
                self.time = 0
                print("Out of time!! Timer reset as 0!!")
            else:
                omega = self.variable_x/self.variable_y
                distance_x = self.variable_y * math.sin(omega*self.time + math.sin(omega*self.time)) 
                distance_y = self.variable_y * (math.cos(omega*self.time + math.cos(omega*self.time)) - math.cos(1))
                last_x = self.simGlobalLocation.point.x
                last_y = self.simGlobalLocation.point.y
               
                self.simGlobalLocation.point.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simGlobalLocation.point.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
               
                self.simRealLocation.point.x = distance_x * Truck.LAT_PER_NORTH + self.simLatitudeStart
                self.simRealLocation.point.y = distance_y * Truck.LON_PER_EAST + self.simLongitudeStart
                
                angle = math.atan2((self.simGlobalLocation.point.y - last_y), (self.simGlobalLocation.point.x - last_x))
                self.simVehicleVelocity.point.x = self.variable_x * math.cos(angle)       # north speed
                self.simVehicleVelocity.point.y = self.variable_x * math.sin(angle)       # east speed
                
                # self.logger.debug("Simulation: send the truck data")            
                self.time += Truck.TimeStep
        else:
            print("Wrong path!!! Can not update the simulation path!!")


        if Truck.isEnableNoise:
            self.simGlobalLocation.pose.position.x += (np.random.normal(0, 0.3))*Truck.LAT_PER_NORTH
            self.simGlobalLocation.pose.position.y += (np.random.normal(0, 0.3))*Truck.LON_PER_EAST
            
            self.simVehicleVelocity.point.x += np.random.normal(0, 0.18)      # north speed
            self.simVehicleVelocity.point.y += np.random.normal(0, 0.18)       # east speed
                
            
    def TimerCallback(self, event):     
		
   
        self.PublishSimulationMsg()


    # def sigint_handler(self):
        # self.logger.debug("shutdown!")


def main():
	
    global LON_PER_EAST
    truck = Truck()

    # # GPS location of Simulahstion 
    simLatitudeStart = 42.323396612189850
    simLongitudeStart = -83.222952844799720 - 0*truck.LON_PER_EAST

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

    dTimeStep = 0.1
    rospy.Timer(rospy.Duration(dTimeStep), truck.TimerCallback)
    # rospy.on_shutdown(truck.sigint_handler)
    rospy.spin()

 
if __name__ == '__main__':
    
    main()

