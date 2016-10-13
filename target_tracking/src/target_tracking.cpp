#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <signal.h>
#include <sstream>
#include <iostream>


typedef std::tuple<double, double, std::string> UTMobject ;
#define EASTING 0
#define NORTHING 1
#define DESIGNATOR 2

using namespace std;


DJIDrone* _ptrDrone;

void SigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate gimbal_control...");
    // delete(_ptrDrone);
      
    ROS_INFO("Shutting down gimbal_control...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}



// this version also places the quadcopter height above the target in an output variable
UTMobject targetDistanceMetersToUTM_WithHeightDifference (geometry_msgs::Point distanceM,        
                                                        double cameraRollToGround_radians, 
                                                        double cameraPitchToGround_radians, 
                                                        double cameraYawToGround_radians, 
                                                        double currentQuadcopterLatitude, 
                                                        double currentQuadcopterLongitude, 
                                                        double currentQuadcopterAltitude_meters,
                                                        // output variable
                                                        double & copterHeightAboveTarget_meters)
{
    
    UTMobject quadcopterLocation2D_UTM;
    
    // we have the magnitude of the offset in each direction
    // and we know the camera's roll, pitc, yaw,
    // relative to the ground (NED frame just like UTM)
    // and we need to convert this to (x,y) coordinates on the ground (don't care about Z, we'll handle altitude in a separate algorithm)
    quadcopterLocation2D_UTM = GPStoUTM(currentQuadcopterLatitude, currentQuadcopterLongitude); //need to test this function
    
    // printf("quad loc. UTM object easting northing zone %f %f %s \n", 
    //              std::get<EASTING>(quadcopterLocation2D_UTM), 
    //              std::get<NORTHING>(quadcopterLocation2D_UTM), 
    //              std::get<DESIGNATOR>(quadcopterLocation2D_UTM).c_str() );
    // so now we have the camera offset from ground (meters), the camera rotation, and the target offset from camera. 
    // We need to convert this to camera offset from ground (ie, from center of UTM coordinates)
    // to do so: we say that targetPosition = cameraOffset + cameraRotationMatrix *** targetOffsetFromCamera, where *** denotes matrix multiplication
    // camera_offset is just our known UTM coordinates since the camera is a point mass with the drone in this model.
      
    double targetOffsetFromUAV[3][1];
    getTargetOffsetFromUAV(distanceM, 
                           cameraRollToGround_radians, 
                           cameraPitchToGround_radians, 
                           cameraYawToGround_radians,
                           targetOffsetFromUAV);
                           
    //cout <<"target offset from UAV (x y z intertial)" << targetOffsetFromUAV[0][0] <<" "<< targetOffsetFromUAV[1][0] <<" "<< targetOffsetFromUAV[2][0] <<" ";
      
    UTMobject targetLocation2D_UTM;

    //it's reasonable to assume we don't cross zones
    std::get<DESIGNATOR>(targetLocation2D_UTM) = std::get<DESIGNATOR>(quadcopterLocation2D_UTM) ; 
    
    //can still use the UTMobject since we don't care about the Z offset because we'll handle altitude separately
    std::get<EASTING>(targetLocation2D_UTM) = std::get<EASTING>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[1][0];
    //targetLocation2D_UTM.first = quadcopterLocation2D_UTM.first + targetOffsetFromUAV[0][0];
    //targetLocation2D_UTM.second = quadcopterLocation2D_UTM.second + targetOffsetFromUAV[1][0];
    
    std::get<NORTHING>(targetLocation2D_UTM) = std::get<NORTHING>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[0][0]; 

    //now convert back to GPS coordinates and we can generate a proper waypoint

    // cout <<"\nOffset from UAV x y z inertial " << targetOffsetFromUAV[0][0] <<" "<< targetOffsetFromUAV[1][0] <<" "<< targetOffsetFromUAV[2][0] <<" quadcopter location east north zone " << std::get<EASTING>(quadcopterLocation2D_UTM) << " "<< std::get<NORTHING>(quadcopterLocation2D_UTM) <<" "<< std::get<DESIGNATOR>(quadcopterLocation2D_UTM)  ;
    // cout <<"\nresult e,n,zone "<< std::get<EASTING>(targetLocation2D_UTM) << " "<< std::get<NORTHING>(targetLocation2D_UTM) <<" "<< std::get<DESIGNATOR>(targetLocation2D_UTM) <<" ";
  
    // copterHeightAboveTarget_meters = targetOffsetFromUAV[2][0];
    // cout << "\n\n Height above target " << copterHeightAboveTarget_meters <<" meters \n"; 
  
    return targetLocation2D_UTM;
   


} ///end function

void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray tag_detection_array)
{

    DJIDrone& drone = *_ptrDrone;
	
    UTMobject latestTargetLocation; 
    //must declare before the if statements so it can be used for further estimates 
    // (by recording the UTM designator zone) if the target is lost	
	
    std_msgs::Header latestHeader; 
    //declare outside the if statements so it can be used at the end	


    apriltags_ros::AprilTagDetectionArray aprilTagsMessage;
    aprilTagsMessage = tag_detection_array;

    // std::vector<apriltags_ros::AprilTagDetection> found;
    // found = aprilTagsMessage.detections;

    // int numTags = found.size();
  
    // cv::Mat targetLocPrediction; 
    // must declare outside the if statements so we can keep estimating if the target is lost
    // If we use a local variable and then assign a global variable to it, 
    // this is less likely to cause memory overflow that just using the global variable directly

    // let's assume that if there are multiple tags, we only want to deal with the first one.
    if (aprilTagsMessage.detections.size() ) //TODO : correct flaw in logic here, such that if we lose the target we still perform he calculations based on estimates 
    {
	    bool firstDetection = true; //assume it's the first detection (and thus that we may need to disregard the information) until proven otherwise
	    // FRAMES_WITHOUT_TARGET = 0; //we've found the target again
  
        apriltags_ros::AprilTagDetection current = aprilTagsMessage.detections.at(0);
               
        // since we want camera frame y to be above the camera, but it treats this as negative y
        current.pose.pose.position.y *= -1;  
                
        //since we want to ensure up, relative to the camera, is treated as positive y in the camera frame  
  
        latestHeader = current.pose.header; 

        //update the global variables containing the coordinates (in the camera frame remember) 
        // LATEST_TARGET_X_CAMERA = current.pose.pose.position.x;
        // LATEST_TARGET_Y_CAMERA =  current.pose.pose.position.y; 
        // LATEST_TARGET_Z_CAMERA = current.pose.pose.position.z;

        //Need to use "pose.pose" since the AprilTagDetection contains a "PoseStamped" which then contains a stamp and a pose
        // TODO make sure that your variables can handle this calculation
        //ros::time tStamp = current.pose.header.stamp;//this gives an error for some reason
   
        double currentTime = current.pose.header.stamp.nsec/1000000000.0 + current.pose.header.stamp.sec;
    
        //figure out elapsed time between detections, necessary for kalman filtering
        //LATEST_DT = currentTime - LATEST_TIMESTAMP ;
 
        //be sure to keep track of time
        //LATEST_TIMESTAMP = currentTime;
        
	 
	    //need to keep track of the altitude and height above target for when we land
        //ALTITUDE_AT_LAST_TARGET_SIGHTING = drone.global_position.altitude;
        
        double dLastRecordedHeightAboveTargetM = 0.0;
        
        // Note that gimbal result is in degrees, but gimbal control is in tenths of a dgree
        latestTargetLocation = targetDistanceMetersToUTM_WithHeightDifference ( current.pose.pose.position,
                                                                                degreesToRadians(drone.gimbal.roll), 
			                                                                    degreesToRadians(drone.gimbal.pitch), 
			                                                                    degreesToRadians(drone.gimbal.yaw/*bodyFrameToInertial_yaw(drone.gimbal.yaw,drone)*/),
			                                                                    // since yaw is relative to body, not inertial frame, 
			                                                                    //we need to convertdegreesToRadians(drone.gimbal.yaw)
			                                                                    drone.global_position.latitude,
			                                                                    drone.global_position.longitude,
			                                                                    drone.global_position.altitude, //TODO decide if we should use .height instead
			                                                                    //LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS // THIS IS AN OUTPUT VARIABLE
                                                                                dLastRecordedHeightAboveTargetM
			                                                                  );
			                                                                  
        // cout <<" verify result e,n,zone "<< std::get<EASTING>(latestTargetLocation) << " "
        //      << std::get<NORTHING>(latestTargetLocation) <<" "<< std::get<DESIGNATOR>(latestTargetLocation)  <<"\n";

        //recall that x is north, y is east, and we need these values to pass the Kalman filter
        double targetX = std::get<NORTHING>(latestTargetLocation); //LATEST_TARGET_X_CAMERA; //std::get<NORTHING>(latestTargetLocation);
        double targetY = std::get<EASTING>(latestTargetLocation); //LATEST_TARGET_Y_CAMERA; //std::get<EASTING>(latestTargetLocation);


	 
    } //closing brace to if(numTags>0)
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_tracking");
    ROS_INFO("target tracking");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::spin();

    _ptrDrone = new DJIDrone(nh);
    
    numMessagesToBuffer = 5;
    ros::Subscriber sub = n.subscribe("dji_sdk/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    
    signal(SIGINT, SigintHandler);    
    ros::spin();
    
    return 0;    

}

