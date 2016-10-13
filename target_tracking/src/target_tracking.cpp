#include <ros/ros.h>
#include "std_msgs/String.h"

#include <target_tracking/conversion.h>
#include <target_tracking/UasMath.h>
#include <dji_sdk/dji_drone.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/NavSatFix.h>

#include <sstream>
#include <iostream> // std::cout, std::end, ...
#include <iomanip> // std::setprecision
#include <signal.h>


//#include <tuple>


//typedef std::tuple<double, double, std::string> UTMobject;

//#define EASTING 0
//#define NORTHING 1
//#define DESIGNATOR 2


DJIDrone* _ptrDrone;

void SigintHandler(int sig)
{
    ROS_INFO("It is requested to terminate gimbal_control...");
    
    delete(_ptrDrone);
      
    ROS_INFO("Shutting down gimbal_control...");
    
    ros::shutdown();
}


 
 
void getTargetOffsetFromUAV(geometry_msgs::Point& distance_m,        
                            dji_sdk::Gimbal& gimbal,
                            double outputDistance[3][1]) //THIS IS AN OUTPUT VARIABLE! 
{
    // rotation matrix will be calculated based on instructions 
    // here: http://planning.cs.uiuc.edu/node102.html
    // first, rename some variables to make calculations more readable

    double yaw = UasMath::ConvertDeg2Rad(gimbal.yaw);
    double pitch = UasMath::ConvertDeg2Rad(gimbal.pitch);
    double roll = UasMath::ConvertDeg2Rad(gimbal.roll);
    
    //printf("\n confirm that roll pitch yaw is respectively %f %f %f \n", roll, pitch, yaw); 
    double cameraRotationMatrix[3][3] = 
        {   {   
                cos(yaw)*cos(pitch),
                cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), 
                cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)
            },
            {  
                sin(yaw)*cos(pitch),
                sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),
                sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll) 
            },
            {
                -1.0*sin(pitch),
                cos(pitch)*sin(roll),
                cos(pitch)*cos(roll)
            },
        };
        
    // WARNING: These x, y, and z values are relative to the camera frame, 
    // NOT THE GROUND FRAME! 
    // This is what we want and why we'll multiply by the rotation matrix
    
    double targetOffsetFromCamera[3][1] = {{distance_m.x},{distance_m.y},{distance_m.z}};
    
    // perform matrix multiplication 
    // (recall that you take the dot product of the 1st matrix rows with the 2nd matrix colums)
    // process is very simple since 2nd matrix is a vertical vector
    
    // first, convert from image plane to real-world coordinates
    double transformMatrix[3][3] = { {0,0,1}, {1,0,0}, {0,-1,0} };

    //now we can determine the distance in the inertial frame
    double distanceInRealWorld[3][1];
  

    for (int row = 0; row < 3; row++)
    {
        double sum = 0;
        for (int column = 0; column < 3; column++)
        {
            sum += transformMatrix[row][column] * targetOffsetFromCamera [column][0]; 
        }
        distanceInRealWorld[row][0] = sum;
    } 
    //end matrix multiplication

    // now we can determine the actual distance from the UAV, by accounting for the camera's orientation
    double targetOffsetFromUAV[3][1];
    
    for (int row = 0; row < 3; row++)
    {
        double sum = 0;
        for (int column = 0; column < 3; column++)
        {
            sum += cameraRotationMatrix[row][column] * distanceInRealWorld [column][0]; 
        }
        targetOffsetFromUAV[row][0] = sum;
    } 
    
    outputDistance[0][0] = targetOffsetFromUAV[0][0];
    outputDistance[1][0] = targetOffsetFromUAV[1][0];
    outputDistance[2][0] = targetOffsetFromUAV[2][0];
   
} ///end getTargetOffsetFromUAV()


// this version also places the quadcopter height above the target in an output variable
// UTMobject 
geometry_msgs::Point targetDistanceMetersToUTM_WithHeightDifference (geometry_msgs::Point& distanceM,   
                                                        dji_sdk::Gimbal& gimbal,
                                                        geometry_msgs::Point& droneUtmPosition,
                                                        // output variable
                                                        double & copterHeightAboveTarget_meters)
{
    
    // we have the magnitude of the offset in each direction
    // and we know the camera's roll, pitc, yaw,
    // relative to the ground (NED frame just like UTM)
    // and we need to convert this to (x,y) coordinates on the ground (don't care about Z, we'll handle altitude in a separate algorithm)
    
    //UTMobject quadcopterLocation2D_UTM = GPStoUTM(dronePosition.latitude, dronePosition.longitude); //need to test this function
    
    // printf("quad loc. UTM object easting northing zone %f %f %s \n", 
    //              std::get<EASTING>(quadcopterLocation2D_UTM), 
    //              std::get<NORTHING>(quadcopterLocation2D_UTM), 
    //              std::get<DESIGNATOR>(quadcopterLocation2D_UTM).c_str() );
    // so now we have the camera offset from ground (meters), the camera rotation, and the target offset from camera. 
    // We need to convert this to camera offset from ground (ie, from center of UTM coordinates)
    // to do so: we say that targetPosition = cameraOffset + cameraRotationMatrix *** targetOffsetFromCamera, where *** denotes matrix multiplication
    // camera_offset is just our known UTM coordinates since the camera is a point mass with the drone in this model.
      
    double targetOffsetFromUAV[3][1];
    getTargetOffsetFromUAV(distanceM, gimbal, targetOffsetFromUAV);
                           
                          
    // UTMobject targetLocation2D_UTM;

    geometry_msgs::Point targetUtmPosition;

    //it's reasonable to assume we don't cross zones
    //std::get<DESIGNATOR>(targetLocation2D_UTM) = std::get<DESIGNATOR>(quadcopterLocation2D_UTM) ; 
    
    targetUtmPosition.x = droneUtmPosition.x + targetOffsetFromUAV[1][0];
    targetUtmPosition.y = droneUtmPosition.y + targetOffsetFromUAV[0][0]; 
    targetUtmPosition.z = 0.0;
    
    // can still use the UTMobject since we don't care about the Z offset because we'll handle altitude separately
    // std::get<EASTING>(targetLocation2D_UTM) = std::get<EASTING>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[1][0];
    // targetLocation2D_UTM.first = quadcopterLocation2D_UTM.first + targetOffsetFromUAV[0][0];
    // targetLocation2D_UTM.second = quadcopterLocation2D_UTM.second + targetOffsetFromUAV[1][0];
    
    // std::get<NORTHING>(targetLocation2D_UTM) = std::get<NORTHING>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[0][0]; 

    // now convert back to GPS coordinates and we can generate a proper waypoint

    // cout <<"\nOffset from UAV x y z inertial " << targetOffsetFromUAV[0][0] <<" "<< targetOffsetFromUAV[1][0] <<" "<< targetOffsetFromUAV[2][0] <<" quadcopter location east north zone " << std::get<EASTING>(quadcopterLocation2D_UTM) << " "<< std::get<NORTHING>(quadcopterLocation2D_UTM) <<" "<< std::get<DESIGNATOR>(quadcopterLocation2D_UTM)  ;
    // cout <<"\nresult e,n,zone "<< std::get<EASTING>(targetLocation2D_UTM) << " "<< std::get<NORTHING>(targetLocation2D_UTM) <<" "<< std::get<DESIGNATOR>(targetLocation2D_UTM) <<" ";
  
    // copterHeightAboveTarget_meters = targetOffsetFromUAV[2][0];
    // cout << "\n\n Height above target " << copterHeightAboveTarget_meters <<" meters \n"; 
  
    // return targetLocation2D_UTM;
    return targetUtmPosition;


} ///end function

void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray tag_detection_array)
{

    DJIDrone& drone = *_ptrDrone;

    //must declare before the if statements so it can be used for further estimates 
    // (by recording the UTM designator zone) if the target is lost	
	
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
    if (aprilTagsMessage.detections.size() ) 
    //TODO : correct flaw in logic here, such that if we lose the target we still perform he calculations based on estimates 
    {
    
        //assume it's the first detection (and thus that we may need to disregard the information) until proven otherwise
	    bool firstDetection = true; 
	    
	    // FRAMES_WITHOUT_TARGET = 0; //we've found the target again
  
        apriltags_ros::AprilTagDetection current = aprilTagsMessage.detections.at(0);
               
        // since we want camera frame y to be above the camera, but it treats this as negative y
        current.pose.pose.position.y *= -1;  
                
        //since we want to ensure up, relative to the camera, is treated as positive y in the camera frame  
  
        //update the global variables containing the coordinates (in the camera frame remember) 

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
        
        	
        //        double northing, easting;
        char zone;

        geometry_msgs::Point droneUtmPosition;    

        gps_common::LLtoUTM(drone.global_position.latitude, 
                            drone.global_position.longitude, 
                            droneUtmPosition.x, 
                            droneUtmPosition.y, 
                            &zone);
    
        
        
        //droneUtmPosition.x = easting;
        //droneUtmPosition.y = northing;
        droneUtmPosition.z = drone.global_position.height;
        	
        	
        // UTMobject latestTargetLocation; 
        geometry_msgs::Point targetUtmLocation
             = targetDistanceMetersToUTM_WithHeightDifference ( current.pose.pose.position,
                                                                drone.gimbal,
                                                                droneUtmPosition,
                                                                //TODO decide if we should use .height instead
			                                                    //LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS 
                                                                // THIS IS AN OUTPUT VARIABLE
                                                                dLastRecordedHeightAboveTargetM);

        std::stringstream ss ;
        
        ss  << std::fixed << std::setprecision(7) << std::endl
            << "Drone Pos(lat,lon,alt): "   << drone.global_position.latitude << "," 
                                            << drone.global_position.longitude << ","
                                            << drone.global_position.height << std::endl
            << "Drone Pos(x,y,z): " << droneUtmPosition.x << "," 
                                    << droneUtmPosition.y << ","
                                    << droneUtmPosition.z << "," << std::endl
            << "Gimbal Angle(y,p,r): "  << drone.gimbal.yaw << ","
                                        << drone.gimbal.pitch << ","
                                        << drone.gimbal.roll << "," << std::endl
            << "Tag Distance(x,y,z): "  << current.pose.pose.position.x << ","
                                        << current.pose.pose.position.y << ","
                                        << current.pose.pose.position.z << "," << std::endl
            << "Distance from UAV (x y z inertial): "   << targetUtmLocation.x << ","
                                                        << targetUtmLocation.y << ","
                                                        << targetUtmLocation.z << std::endl;
                                                        
        ROS_INFO("%s", ss.str().c_str());



	 
    } //closing brace to if(numTags>0)
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_tracking");
    ROS_INFO("target tracking");
    ros::NodeHandle nh;

    _ptrDrone = new DJIDrone(nh);

    DJIDrone& drone = *_ptrDrone;

    // set the gimbal pitch  to -45 for tests.
    drone.gimbal_angle_control(0.0, -250.0, 0.0, 10.0);    
    
    int numMessagesToBuffer = 5;
    ros::Subscriber sub = nh.subscribe("dji_sdk/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    
    signal(SIGINT, SigintHandler);    
    ros::spin();
    
    return 0;    

}

