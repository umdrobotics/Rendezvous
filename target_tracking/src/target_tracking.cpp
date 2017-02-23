#include <ros/ros.h>
#include "target_tracking/KalmanFilter.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <target_tracking/conversion.h>
#include <target_tracking/UasMath.h>
#include <dji_sdk/dji_drone.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/NavSatFix.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#include <sstream>
#include <iostream> // std::cout, std::end, ...
#include <iomanip> // std::setprecision
#include <signal.h>


DJIDrone* _ptrDrone;
ros::Publisher _GimbalAnglePub;
ros::Publisher _TargetLocalPosition;

// ros::Publisher _DroneUTMPub;
// ros::Publisher _TargetGPSPub;
// ros::Publisher _TargetUTMPub;
ros::Publisher _ToTargetDistancePub;




void SigintHandler(int sig)
{
    ROS_INFO("It is requested to terminate gimbal_control...");
    
    delete(_ptrDrone);
      
    ROS_INFO("Shutting down gimbal_control...");
    
    ros::shutdown();
}


//I think this is how you get the yaw of the quadcopter body
 //this method is based off the documentation for onboard-sdk  on Github https://github.com/dji-sdk/Onboard-SDK/blob/3.1/doc/en/ProgrammingGuide.md
 void quaternionToRPY(dji_sdk::AttitudeQuaternion q, double & roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{ 
     roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
     pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
     yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}



//also need to be able to convert from the body frame to the coordinate frame, for gimbal yaw
//so calculate the angle you want relative to inertal frame, then use this function to get the command you need
double inertialFrameToBody_yaw(double angleToInertial_rad, DJIDrone& drone)
{

    //first need to convert from quaternion format to RPY to get drone's yaw 
    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    quaternionToRPY( drone.attitude_quaternion, 
                    roll_rad, 
                    pitch_rad, 
                    yaw_rad); 
    //roll_body pitch_body and yaw_body are output variables that will be altered by this function
    //rotation to inertial frame = rotation to body + body's rotation to inertial frame
    //rotation to body = rotation to inertial frame - body's rotation to inertial frame
 
    double angleToBody = angleToInertial_rad - yaw_rad;
      
    //now keep the angle between -180 and 180 degrees (ie -pi and pi)

    while(angleToBody <= -1.0*M_PI) {angleToBody += 2.0*M_PI;}
    while(angleToBody > M_PI) {angleToBody -= 2.0*M_PI;}
    
    return angleToBody;

}



//if the quadcopter can't catch up to the target, we still want the camera to point at it. This calculates how the camera will need to be oriented, relative to the inertial frame. Assume no roll is used, only pitch and yaw. 
//It will assign the values to the PASS-BY-REFERENCE variables yaw, pitch, and roll (roll will end up 0)
void getGimbalAngleToPointAtTarget_rads(geometry_msgs::Point& droneUtmPosition, //UTMobject quadcopterLocation_UTM, 
                                        double heightAboveTarget_Meters, 
                                        geometry_msgs::Point targetUtmLocation, //UTMobject targetLocation_UTM,
                                        double &yaw_rads, //This is an output variable
                                        double &pitch_rads, //This is an output variable
                                        double &roll_rads )    //This is an output variable
{

    // compute displacements in northing and easting (signed)
    double deltaNorth = targetUtmLocation.y  - droneUtmPosition.y;
    double deltaEast = targetUtmLocation.x  - droneUtmPosition.x;

    double distance_Meters = sqrt( (deltaEast * deltaEast) + (deltaNorth * deltaNorth ) );
    
    //rotation matrix will be calculated based on instructions here: http://planning.cs.uiuc.edu/node102.html  
    //this applies roll, then pitch, then yaw (we use 0 roll always). 
    // thus, pitch can be computed from the altitude and the unsigned distance, and then the gimbal can be yawed according to signed distance
    roll_rads = 0;
 
    // for exaplanation of calculations, see diagram in Aaron Ward's July 20 report
    // pitch_rads = -1.0 * atan2( heightAboveTarget_Meters,  distance_Meters ); 
    //this is done correctly since we want to limit it to between 0 and -90 degrees (in fact could just use regular tangent)
    pitch_rads = -1.0 * atan2( droneUtmPosition.z, distance_Meters );

    // yaw_rads = acos( deltaNorth / distance_Meters );
    // turns out acos can't be used, since it doesn't do enough to specify the quadrant. Use
 
    yaw_rads = atan2( deltaEast, deltaNorth); 
    //remember north is the x axis, east is the y axis
  
  
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
geometry_msgs::Point targetDistanceMetersToUTM_WithHeightDifference(geometry_msgs::Point& distanceM,   
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

void PredictDesiredGimbalAngle(const apriltags_ros::AprilTagDetectionArray tag_detection_array)
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
        droneUtmPosition.z = drone.global_position.altitude;
        

        	
        // UTMobject latestTargetLocation; 
        geometry_msgs::Point targetUtmPosition
             = targetDistanceMetersToUTM_WithHeightDifference ( current.pose.pose.position,
                                                                drone.gimbal,
                                                                droneUtmPosition,
                                                                //TODO decide if we should use .height instead
			                                                    //LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS 
                                                                // THIS IS AN OUTPUT VARIABLE
                                                                dLastRecordedHeightAboveTargetM);
		

		
        
        // then need to modify the gimbal angle to have it point appropriately. 
        // This will be done with multiple variables that will be modified by a function, 
        // rather than an explicit return
       
        double yaw_rads;
        double pitch_rads;
        double roll_rads;
        
        getGimbalAngleToPointAtTarget_rads (droneUtmPosition, 
                                            drone.global_position.height, //heightAboveTarget, 
                                            targetUtmPosition,
                                            yaw_rads, //This is an output variable
                                            pitch_rads, //This is an output variable
                                            roll_rads);   //This is an output variable
                                               
        // Yaw is not relative to body
        yaw_rads = inertialFrameToBody_yaw(yaw_rads, drone);
     
        //then  we're done with this step
         
        // if we want to use a separate nod for PID calculations, need to publish them here
        // the following link provides a good guide: http://answers.ros.org/question/48727/publisher-and-subscriber-in-the-same-node/
        // for simplicity, I'm going to send the desired angle in the form of a pointStamped message (point with timestamp)
        // I will translate the indices according to their standard order, ie, since x comes before y and roll comes before pitch.
        // roll will be the x element and pitch will be the y element

        geometry_msgs::PointStamped msgDesiredAngleDeg;	
        msgDesiredAngleDeg.point.x = UasMath::ConvertRad2Deg(roll_rads);
        msgDesiredAngleDeg.point.y = UasMath::ConvertRad2Deg(pitch_rads);
        msgDesiredAngleDeg.point.z = UasMath::ConvertRad2Deg(yaw_rads);
        
        msgDesiredAngleDeg.header = current.pose.header; //send the same time stamp information that was on the apriltags message to the PID node
        
        _GimbalAnglePub.publish(msgDesiredAngleDeg);



                
        std::stringstream ss ;
        
        ss  << std::fixed << std::setprecision(7) << std::endl
            << "Time: " << ros::Time::now().toSec() << std::endl
            << "Drone Pos(lat,lon,alt,heigt): "   << drone.global_position.latitude << "," 
                                            << drone.global_position.longitude << ","
                                            << drone.global_position.altitude << "," 
                                            << drone.global_position.height << std::endl
            << "Drone Pos(x,y,z): " << droneUtmPosition.x << "," 
                                    << droneUtmPosition.y << ","
                                    << droneUtmPosition.z << "," << std::endl
            << "Gimbal AngleDeg(y,p,r): "  << drone.gimbal.yaw << ","
                                        << drone.gimbal.pitch << ","
                                        << drone.gimbal.roll << "," << std::endl
            << "Tag Distance(x,y,z): "  << current.pose.pose.position.x << ","
                                        << current.pose.pose.position.y << ","
                                        << current.pose.pose.position.z << "," << std::endl
            << "Target Pos(x y z): "    << targetUtmPosition.x << ","
                                        << targetUtmPosition.y << ","
                                        << targetUtmPosition.z << std::endl
            << "Desired AngleDeg(y,p,r): "     << msgDesiredAngleDeg.point.z << ","
                                                << msgDesiredAngleDeg.point.y << ","
                                                << msgDesiredAngleDeg.point.x << std::endl;
                                                        
        ROS_INFO("%s", ss.str().c_str());
    } //closing brace to if(numTags>0)
}


void FindDesiredGimbalAngle(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{

    DJIDrone& drone = *_ptrDrone;
	char zone;
    double dLastRecordedHeightAboveTargetM = 0.0;

    if (vecTagDetections.detections.empty())
    {
        // There is nothing we can do
        return;
    }

    // We assume there is only one tag detection.
    // Even if there are multiple detections, we still take the first tag.
    
    apriltags_ros::AprilTagDetection tag = vecTagDetections.detections.at(0);
               
    double x = tag.pose.pose.position.x;
    // since we want camera frame y to be above the camera, but it treats this as negative y
    double y = -tag.pose.pose.position.y;
    double z = tag.pose.pose.position.z;
    
    // double currentTime = tag.pose.header.stamp.nsec/1000000000.0 + tag.pose.header.stamp.sec;
    
    // for exaplanation of calculations, see diagram in Aaron Ward's July 20 report
    // pitch_rads = -1.0 * atan2( heightAboveTarget_Meters,  distance_Meters ); 
    // this is done correctly since we want to limit it to between 0 and -90 degrees (in fact could just use regular tangent)
    double pitchDeg = UasMath::ConvertRad2Deg(atan2(y, z));
 
    double yawDeg = UasMath::ConvertRad2Deg(atan2(x, z)); 
    //remember north is the x axis, east is the y axis
   
        
    geometry_msgs::PointStamped msgDesiredAngleDeg;	
    msgDesiredAngleDeg.point.x = 0;
    msgDesiredAngleDeg.point.y = drone.gimbal.pitch + pitchDeg;
    msgDesiredAngleDeg.point.z = drone.gimbal.yaw + yawDeg;
  
    _GimbalAnglePub.publish(msgDesiredAngleDeg);


	//Test
	tag.pose.pose.position.y *= -1;

    double targetOffsetFromUAV[3][1];
    getTargetOffsetFromUAV(tag.pose.pose.position, drone.gimbal, targetOffsetFromUAV);

    geometry_msgs::PointStamped msgTargetLocalPosition;
    msgTargetLocalPosition.point.x = drone.local_position.x + targetOffsetFromUAV[1][0];
    msgTargetLocalPosition.point.y = drone.local_position.y + targetOffsetFromUAV[0][0];	
    msgTargetLocalPosition.point.z = 0;
	
    _TargetLocalPosition.publish(msgTargetLocalPosition);

    //Create message
    geometry_msgs::PointStamped msgToTargetDistance;
    msgToTargetDistance.point.x = targetOffsetFromUAV[1][0];
    msgToTargetDistance.point.y = targetOffsetFromUAV[0][0];
    msgToTargetDistance.point.z = drone.global_position.height;

    _ToTargetDistancePub.publish(msgToTargetDistance);   




      
    std::stringstream ss ;
    
    ss  << std::fixed << std::setprecision(7) << std::endl
        << "Time: " << ros::Time::now().toSec() << std::endl
        << "Tag Distance(x,y,z): "  << x << ","
                                    << y << ","
                                    << z << "," << std::endl
        << "Real Distance(Easting,Northing,Height): "   << targetOffsetFromUAV[1][0] << ","
                                                        << targetOffsetFromUAV[0][0] << ","
                                                        << drone.global_position.height << "," << std::endl                                
        << "Gimbal Angle Deg(y,p,r): "  << drone.gimbal.yaw << ","
										<< drone.gimbal.pitch << ","
										<< drone.gimbal.roll << "," << std::endl
        << "Desired Angle Deg(y,p,r): " << msgDesiredAngleDeg.point.z << ","
                                        << msgDesiredAngleDeg.point.y << ","
                                        << msgDesiredAngleDeg.point.x << std::endl;
              
    ROS_INFO("%s", ss.str().c_str());
	
}

/*
void FindTargetLocation(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{

    DJIDrone& drone = *_ptrDrone;
    char zone;
    double dLastRecordedHeightAboveTargetM = 0.0;

    if (vecTagDetections.detections.empty())
    {
        // There is nothing we can do
        return;
    }

    apriltags_ros::AprilTagDetection tag = vecTagDetections.detections.at(0);
    tag.pose.pose.position.y *= -1;


    ////////////////////////////////////////////////////////////////////////////////////////
    //Get drone UTM position
    geometry_msgs::Point droneUtmPosition;    
    gps_common::LLtoUTM(drone.global_position.latitude, 
                        drone.global_position.longitude, 
                        droneUtmPosition.x, 
                        droneUtmPosition.y, 
                        &zone);
    //droneUtmPosition.x = easting;
    //droneUtmPosition.y = northing;
    droneUtmPosition.z = drone.global_position.altitude;

    //Create message
    geometry_msgs::PointStamped msgDroneUtmPosition;
    msgDroneUtmPosition.header = tag.pose.header;
    msgDroneUtmPosition.point.x = droneUtmPosition.x;
    msgDroneUtmPosition.point.y = droneUtmPosition.y;
    msgDroneUtmPosition.point.z = droneUtmPosition.z;
    _DroneUTMPub.publish(msgDroneUtmPosition);


    ////////////////////////////////////////////////////////////////////////////////////////
    // Get target UTM position
    // UTMobject latestTargetLocation; 
    geometry_msgs::Point targetUtmPosition
        = targetDistanceMetersToUTM_WithHeightDifference (  tag.pose.pose.position,
                                                            drone.gimbal,
                                                            droneUtmPosition,
                                                            //TODO decide if we should use .height instead
                                                            //LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS 
                                                            // THIS IS AN OUTPUT VARIABLE
                                                            dLastRecordedHeightAboveTargetM);

    //Create message
    geometry_msgs::PointStamped msgTargetUtmPosition;
    msgTargetUtmPosition.header = tag.pose.header;
    msgTargetUtmPosition.point.x = targetUtmPosition.x;
    msgTargetUtmPosition.point.y = targetUtmPosition.y;
    msgTargetUtmPosition.point.z = targetUtmPosition.z;
    _TargetUTMPub.publish(msgTargetUtmPosition);


    ////////////////////////////////////////////////////////////////////////////////////////
    //Get target GPS position
    geometry_msgs::Point targetGpsPosition;
    gps_common::UTMtoLL(targetUtmPosition.x,
                        targetUtmPosition.y,
                        &zone,
                        targetGpsPosition.x,
                        targetGpsPosition.y);

    //Create message    
    geometry_msgs::PointStamped msgTargetGpsPosition;
    msgTargetGpsPosition.header = tag.pose.header;
    msgTargetGpsPosition.point.x = targetGpsPosition.x;
    msgTargetGpsPosition.point.y = targetGpsPosition.y;
    msgTargetGpsPosition.point.z = targetUtmPosition.z;
    _TargetGPSPub.publish(msgTargetGpsPosition);


    std::stringstream ss ;
        
    ss  << std::fixed << std::setprecision(7) << std::endl
        << "Time: " << ros::Time::now().toSec() << std::endl
        << "Drone Pos(lat,lon,alt,heigt): "   << drone.global_position.latitude << "," 
                                              << drone.global_position.longitude << ","
                                              << drone.global_position.altitude << "," 
                                              << drone.global_position.height << std::endl
        << "Drone Pos(x,y,z): " << droneUtmPosition.x << "," 
                                << droneUtmPosition.y << ","
                                << droneUtmPosition.z << "," << std::endl
        << "Tag Distance(x,y,z): "  << tag.pose.pose.position.x << ","
                                    << tag.pose.pose.position.y << ","
                                    << tag.pose.pose.position.z << "," << std::endl
        << "Target Pos(x y z): "    << targetUtmPosition.x << ","
                                    << targetUtmPosition.y << ","
                                    << targetUtmPosition.z << std::endl
        << "Target Pos(lat,lon,alt): "  << targetGpsPosition.x << "," 
                                        << targetGpsPosition.y << ","
                                        << targetUtmPosition.z << std::endl;
                                              
    ROS_INFO("%s", ss.str().c_str());
}
*/


void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{
    // PredictDesiredGimbalAngle(vecTagDetections);
   
    FindDesiredGimbalAngle(vecTagDetections);
    //FindTargetLocation(vecTagDetections);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_tracking");
    ROS_INFO("target tracking");
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);    

    _ptrDrone = new DJIDrone(nh);
    DJIDrone& drone = *_ptrDrone;

    // set the gimbal pitch  to -25 for tests.
    drone.gimbal_angle_control(0.0, -250.0, 0.0, 10.0);    

    _GimbalAnglePub = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 2); 
    _TargetLocalPosition = nh.advertise<geometry_msgs::PointStamped>("/target_tracking/target_local_position", 2); 
    _ToTargetDistancePub = nh.advertise<geometry_msgs::PointStamped>("/target_tracking/to_target_distance", 2);


	// _DroneUTMPub = nh.advertise<geometry_msgs::PointStamped>("/target_tracking/drone_utm_position", 2); 
	// _TargetGPSPub = nh.advertise<geometry_msgs::PointStamped>("/target_tracking/target_gps_position", 2); 
	// _TargetUTMPub = nh.advertise<geometry_msgs::PointStamped>("/target_tracking/target_utm_position", 2); 
    
    // queue size of 2 seems reasonable
    int numMessagesToBuffer = 2;
    ros::Subscriber sub = nh.subscribe("/usb_cam/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    

    ros::spin();
    
    return 0;    

}

