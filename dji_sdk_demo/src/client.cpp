/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>



#include <dji_sdk_demo/kalmanPhysics.cpp>  //need to figure out how to put this in the main folder instead of include
               volatile int degs =0; // for debugging gimbal control
             volatile int GIMBAL_TEST_SIGN = 1; //for debugging gimbal control

#include <dji_sdk_demo/PIDcontrol.cpp> //need to figure out how to put this in the main folder instead of include


#define YAW_RELATIVE_TO_BODY false // if gimbal yaw command is relative to the body, this is true, if it's relative to the inertial frame, it's false
///Begin global tracking variables
//These variables need to be global so tha Apriltag Detection Callback can access them

double LATEST_TIMESTAMP;
double LATEST_DT; //the time interval

//keep in mind these will be in the camera frame
double LATEST_TARGET_X_CAMERA;
double LATEST_TARGET_Y_CAMERA;
double LATEST_TARGET_Z_CAMERA;

const double CAMERA_Y_MULTIPLIER = -1.0 ; // since we want camera frame y to be above the camera, but it treats this as negative y

bool IS_TRACKING; //have we found that the target yet?
const int FRAMES_UNTIL_TARGET_LOST = 5; //can go 5 frames before the target is lost and the kalman filter resets
int FRAMES_WITHOUT_TARGET = 0; 


cv::KalmanFilter GLOBAL_KALMAN_FILTER; 
cv::KalmanFilter GLOBAL_KALMAN_FILTER_DIST; //for debugging, let's just try to track the apriltag distance 


//Note: The angle control system needs to have access to the DJIDrone* object to send the gimbal commands
// or else we'll need to simultaneously publish to and subscribe from the node that does the PID calculations.
//Because of that, I'm placing the PID control within this node for now. 
PIDController* GLOBAL_ROLL_CONTROLLER = new PIDController();
PIDController* GLOBAL_PITCH_CONTROLLER = new PIDController();
PIDController* GLOBAL_YAW_CONTROLLER = new PIDController();
double GLOBAL_ROLL_DJI_UNITS =0.0;
double GLOBAL_PITCH_DJI_UNITS =0.0;
double GLOBAL_YAW_DJI_UNITS =0.0;


//if we want to publish a new message whenever a new apriltag is detected, it might be best to publish the angles from within the apriltag subscriber callback
//that means we need a global publisher variable
ros::Publisher GLOBAL_ANGLE_PUBLISHER;
//doesn't seem to let me set it as a null pointer for some reason
//*GLOBAL_ANGLE_PUBLISHER = NULL ; //initialize to prevent errors = NULL ; //initialize to prevent errors 

///End global tracking variables



//following includes are to allow integration with AprilTags:
//instructions on how to get Catkin to see them are here:
// http://answers.ros.org/question/206257/catkin-use-ros-message-from-another-package/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/image_encodings.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include<vector>
#include <image_transport/image_transport.h>


//begin includes and definitions for geolocalization
#include <math.h> //used for sine and cosine for rotation matrix when geolocalizing target
#include <dji_sdk_demo/conversions.h> //provides GPS-UTM conversions
//end includes for integration with AprilTags
#include <tuple>

typedef std::tuple<double, double, std::string> UTMobject ;
#define eastingIndex 0
#define northingIndex 1
#define designatorIndex 2
#define latitudeIndex first //for use with std::pair objects
#define longitudeIndex second //for use with std::pair objects


//end includes and definitions for geolocalizations



//begin functions for integration with AprilTags and geolocalization

//DJI's gimbal angles are in tenths of a degree, but the drone->gimbal.yaw, etc, returns it in degrees. We need to be able to convert forward and back
double radiansToDjiUnits(double angle_rads)
 {
  return (10*180.0/M_PI)* angle_rads;
 }
double djiUnitsToRadians(double angle_dji)
 {
  return ( (M_PI*angle_dji)/ (10.0*180.0) );
 }

double radiansToDegrees(double angle_rads)
 {
  return (180.0/M_PI)* angle_rads;
 }
double degreesToRadians(double angle_degrees)
 {
       return ( (M_PI*angle_degrees)/ (1.0*180.0) );
 }
 
double degreesToDjiUnits(double angle_degrees)
 {
       return (10.0*angle_degrees); 
 }
double djiUnitsToDegrees(double angle_dji)
 {
       return ( (angle_dji)/ (10.0) );
 }
 
//I think this is how you get the yaw of the quadcopter body
 //this method is based off the documentation for onboard-sdk  on Github https://github.com/dji-sdk/Onboard-SDK/blob/3.1/doc/en/ProgrammingGuide.md
 void quaternionToRPY(   dji_sdk::AttitudeQuaternion q, double & roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{ 
     roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
     pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
     yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}


//also need to be able to convert from the body frame to the coordinate frame, for gimbal yaw
//so calculate the angle you want relative to inertal frame, then use this function to get the command you need
double inertialFrameToBody_yaw(double angleToInertial_rads, DJIDrone* dronePointer)
   {

     //first need to convert from quaternion format to RPY to get drone's yaw 
        double roll_body;
  	double pitch_body;
 	double yaw_body;
        quaternionToRPY(dronePointer->attitude_quaternion, roll_body, pitch_body, yaw_body); //roll_body pitch_body and yaw_body are output variables that will be altered by this function
        //rotation to inertial frame = rotation to body + body's rotation to inertial frame
			//rotation to body = rotation to inertial frame - body's rotation to inertial frame
        double angleToBody = angleToInertial_rads - yaw_body;
      
         //now keep the angle between -180 and 180 degrees (ie -pi and pi)
        while(angleToBody < -1.0*M_PI) {angleToBody += 2.0*M_PI;}
        while(angleToBody > M_PI) {angleToBody -= 2.0*M_PI;}
    // cout << " desired angle inertial " << angleToInertial_rads<< "body yaw " << yaw_body << "resulting angle for command " <<  angleToBody <<"\n";

       return angleToBody;
    }


double bodyFrameToInertial_yaw(double angleToBody_rads, DJIDrone* dronePointer)
   {

     //first need to convert from quaternion format to RPY to get drone's yaw 
        double roll_body;
  	double pitch_body;
 	double yaw_body;
        quaternionToRPY(dronePointer->attitude_quaternion, roll_body, pitch_body, yaw_body); //roll_body pitch_body and yaw_body are output variables that will be altered by this function
        //rotation to inertial frame = rotation to body + body's rotation to inertial frame
        double angleToInertial = angleToBody_rads + yaw_body;
      
         //now keep the angle between -180 and 180 degrees (ie -pi and pi)
        while(angleToInertial < -1.0*M_PI) {angleToInertial += 2.0*M_PI;}
        while(angleToInertial > M_PI) {angleToInertial -= 2.0*M_PI;}


       return angleToInertial;
    }





void waypointBasedOnApriltags(int id, DJIDrone* drone)
{
           dji_sdk::WaypointList newWaypointList;
	   dji_sdk::Waypoint waypoint0, waypoint1, waypoint2, waypoint3;
		

                /* Waypoint List Navi Test */
                
                    waypoint0.latitude = 22.535;
                    waypoint0.longitude = 113.95;
                    waypoint0.altitude = 100;
                    waypoint0.staytime = 5;
                    waypoint0.heading = 0;
    

                
                    waypoint1.latitude = 22.535;
                    waypoint1.longitude = 113.96;
                    waypoint1.altitude = 100;
                    waypoint1.staytime = 0;
                    waypoint1.heading = 90;
    
                
                    waypoint2.latitude = 22.545;
                    waypoint2.longitude = 113.96;
                    waypoint2.altitude = 100;
                    waypoint2.staytime = 4;
                    waypoint2.heading = -90;

                    waypoint3.latitude = 22.545;
                    waypoint3.longitude = 113.96;
                    waypoint3.altitude = 10;
                    waypoint3.staytime = 2;
                    waypoint3.heading = 180;
    
                    /*waypoint4.latitude = 22.525;
                    waypoint4.longitude = 113.93;
                    waypoint4.altitude = 50;
                    waypoint4.staytime = 0;
                    waypoint4.heading = -180;*/
 if(0 < id && id < 5)               
   {
     if(id>=1)
       {
                 newWaypointList.waypoint_list.push_back(waypoint0);
        }
     if(id>=2)
       {
                 newWaypointList.waypoint_list.push_back(waypoint1);
        }
     if(id>=3)
       {
                 newWaypointList.waypoint_list.push_back(waypoint2);
        }
     if(id>=4)
       {
                 newWaypointList.waypoint_list.push_back(waypoint3);
        }

     printf("\n Since first id spotted was %d drone will fly to %d waypoints", id, id);

                drone->waypoint_navigation_send_request(newWaypointList);
 }
 else{printf("first id seen was %d, please show a tag id between 1 and 4 (inclusive) for waypoint demo", id);}


}


UTMobject GPStoUTM(double latitude, double longitude)
{
 //printf("WARNING: still need to test GPS to UTM conversion!"); 
 //variables which will be modified to store the eastings and northings
 double northing;
 double easting; 
//string to hold zone designator
 std::string zone; 
 //find the eastings and northings from lat and long
 gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);


 //now create an object ( a tuple) that holds these
 UTMobject UTMcoords;
 std::get<eastingIndex>(UTMcoords) = easting;
 std::get<northingIndex>(UTMcoords) = northing;
 std::get<designatorIndex>(UTMcoords) = zone;

 return UTMcoords;
}


std::pair<double, double> UTMtoGPS(double northing, double easting, std::string zone)
{
 //printf("WARNING: still need to test UTM back to GPS conversion!"); 
 //create storage variables to hold lat and long results
 double latResult = 0;
 double longResult = 0;

 gps_common::UTMtoLL(northing, easting, zone, latResult, longResult) ;
 std::pair<double, double> GPScoords;
 GPScoords.latitudeIndex = latResult;
 GPScoords.longitudeIndex = longResult;
 return GPScoords;
}


//if the quadcopter can't catch up to the target, we still want the camera to point at it. This calculates how the camera will need to be oriented, relative to the inertial frame. Assume no roll is used, only pitch and yaw. 
//It will assign the values to the PASS-BY-REFERENCE variables yaw, pitch, and roll (roll will end up 0)
void getGimbalAngleToPointAtTarget_rads
(
 	UTMobject quadcopterLocation_UTM, 
	double quadcopterAltitude_Meters, 
	UTMobject targetLocation_UTM
 	,double &yaw_rads //This is an output variable
	,double &pitch_rads //This is an output variable
	,double &roll_rads    //This is an output variable
)
{
	//printf("\nCAUTION: Gimbal angle calculation assumes target and quadcopter are in same UTM zones. \n Also, assumes that positive pitch is up, and positive yaw is from north to east \n");
  //compute displacements in northing and easting (signed)
	double deltaNorth = std::get<northingIndex>(targetLocation_UTM) - std::get<northingIndex>(quadcopterLocation_UTM) ;
 	double deltaEast = std::get<eastingIndex>(targetLocation_UTM) - std::get<eastingIndex>(quadcopterLocation_UTM); 

//compute distance using pythagorean theorem
	double squareOfDistance_Meters = (deltaEast * deltaEast) + (deltaNorth * deltaNorth ) ;
        double distance_Meters = sqrt(squareOfDistance_Meters);
	
 //rotation matrix will be calculated based on instructions here: http://planning.cs.uiuc.edu/node102.html  
 //this applies roll, then pitch, then yaw (we use 0 roll always). 
// thus, pitch can be computed from the altitude and the unsigned distance, and then the gimbal can be yawed according to signed distance
 roll_rads = 0;
 
//for exaplanation of calculations, see diagram in Aaron Ward's July 20 report
 pitch_rads = -1.0 * atan2( quadcopterAltitude_Meters,  distance_Meters ); //this is done correctly since we want to limit it to between 0 and -90 degrees (in fact could just use regular tangent)
 //pitch_rads = -1.0 * atan( quadcopterAltitude_Meters / distance_Meters );
 //printf("\n\nTODO: Check if the negative sign is done correctly in the atan or atan2 function in the \"getGimbalAngleToPointAtTarget_rads\" function \n\n");
 //yaw_rads = acos( deltaNorth / distance_Meters );
 //turns out acos can't be used, since it doesn't do enough to specify the quadrant. Use
 yaw_rads = atan2( deltaEast, deltaNorth); //remember north is the x axis, east is the y axis
  cout <<"\n atan2 of y x " << deltaEast <<" " <<deltaNorth << " quadcopter east north " << std::get<eastingIndex>(quadcopterLocation_UTM) << " "<< std::get<northingIndex>(quadcopterLocation_UTM) <<  "    target east north " <<std::get<eastingIndex>(targetLocation_UTM) <<" " << std::get<northingIndex>(targetLocation_UTM)<<  "\n"; 
  
  
}



void getTargetOffsetFromUAV 
     (
	geometry_msgs::Point targetDistanceFromCamera_meters, 	     
	float cameraRollToGround_radians, 
	float cameraPitchToGround_radians, 
	float cameraYawToGround_radians,
	double outputDistance[3][1] //THIS IS AN OUTPUT VARIABLE!
     )
{
	

      
//////yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
      //rotation matrix will be calculated based on instructions here: http://planning.cs.uiuc.edu/node102.html
//first, rename some variables to make calculations more readable
	double yaw = cameraYawToGround_radians;
	double pitch = cameraPitchToGround_radians;
	double roll = cameraRollToGround_radians;
       //printf("\n confirm that roll pitch yaw is respectively %f %f %f \n", roll, pitch, yaw); 
      double cameraRotationMatrix[3][3] = 
		{
			{	cos(yaw)*cos(pitch),
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
  //WARNING: These x, y, and z values are relative to the camera frame, NOT THE GROUND FRAME! This is what we want and why we'll multiply by the rotation matrix
    double targetOffsetFromCamera[3][1] =
             {
		{targetDistanceFromCamera_meters.x},	  			{targetDistanceFromCamera_meters.y},	 			{targetDistanceFromCamera_meters.z}
	     };
    
  //perform matrix multiplication 
  //(recall that you take the dot product of the 1st matrix rows with the 2nd matrix colums)
 //process is very simple since 2nd matrix is a vertical vector

//first, convert from image plane to real-world coordinates
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


dji_sdk::Waypoint targetDistanceMetersToLongitude
     (
	geometry_msgs::Point targetDistanceFromCamera_meters, 	      float cameraRollToGround_radians, 
	float cameraPitchToGround_radians, 
	float cameraYawToGround_radians 
	,double currentQuadcopterLatitude 
	,double currentQuadcopterLongitude 
	,float currentQuadcopterAltitude_meters
     )
{
	
        UTMobject quadcopterLocation2D_UTM;
       //we have the magnitude of the offset in each direction
       // and we know the camera's roll, pitc, yaw,
       // relative to the ground (NED frame just like UTM)
       //and we need to convert this to (x,y) coordinates on the ground (don't care about Z, we'll handle altitude in a separate algorithm)
        quadcopterLocation2D_UTM = GPStoUTM(currentQuadcopterLatitude, currentQuadcopterLongitude); //need to test this function
        //printf("quad loc. UTM object easting northing zone %f %f %s \n", std::get<eastingIndex>(quadcopterLocation2D_UTM), std::get<northingIndex>(quadcopterLocation2D_UTM), std::get<designatorIndex>(quadcopterLocation2D_UTM).c_str() );

		
		double  targetOffsetFromUAV[3][1];
	  getTargetOffsetFromUAV(
	                           targetDistanceFromCamera_meters 
							   ,cameraRollToGround_radians 
							   ,cameraPitchToGround_radians 
							   ,cameraYawToGround_radians
							   ,targetOffsetFromUAV   ///THIS IS AN OUTPUT VARIABlE THAT WILL BE MODIFIED
							);		
		
		
    UTMobject targetLocation2D_UTM;
//it's reasonable to assume we don't cross zones
    std::get<designatorIndex>(targetLocation2D_UTM) = std::get<designatorIndex>(quadcopterLocation2D_UTM) ; 
    printf("CAUTION: target assumed to be in same UTM zone as quadcopter, zone: %s", std::get<designatorIndex>(quadcopterLocation2D_UTM).c_str());
    //can still use the UTMobject since we don't care about the Z offset because we'll handle altitude separately
      std::get<eastingIndex>(targetLocation2D_UTM) = std::get<eastingIndex>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[1][0];//targetLocation2D_UTM.first = quadcopterLocation2D_UTM.first + targetOffsetFromUAV[0][0];
      //targetLocation2D_UTM.second = quadcopterLocation2D_UTM.second + targetOffsetFromUAV[1][0];
      std::get<northingIndex>(targetLocation2D_UTM) = std::get<northingIndex>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[0][0]; 



//now convert back to GPS coordinates and we can generate a proper waypoint
      std::pair<double, double> targetLocation2D_GPS = UTMtoGPS(std::get<northingIndex>(targetLocation2D_UTM), std::get<eastingIndex>(targetLocation2D_UTM), std::get<designatorIndex>(targetLocation2D_UTM)); 
printf("target GPS location is lat %f long %f ", targetLocation2D_GPS.latitudeIndex, targetLocation2D_GPS.longitudeIndex); 

    	dji_sdk::Waypoint targetLocationWithSameAltitude;
      targetLocationWithSameAltitude.latitude = targetLocation2D_GPS.latitudeIndex;
      targetLocationWithSameAltitude.longitude = targetLocation2D_GPS.longitudeIndex;
      targetLocationWithSameAltitude.altitude = currentQuadcopterAltitude_meters; 
      targetLocationWithSameAltitude.staytime = 0;
      targetLocationWithSameAltitude.heading = 0 ;

	return targetLocationWithSameAltitude;
   
} 





UTMobject targetDistanceMetersToUTM
     (
	geometry_msgs::Point targetDistanceFromCamera_meters, 	      float cameraRollToGround_radians, 
	float cameraPitchToGround_radians, 
	float cameraYawToGround_radians 
	,double currentQuadcopterLatitude 
	,double currentQuadcopterLongitude 
	,float currentQuadcopterAltitude_meters
     )
{
	
        UTMobject quadcopterLocation2D_UTM;
       //we have the magnitude of the offset in each direction
       // and we know the camera's roll, pitc, yaw,
       // relative to the ground (NED frame just like UTM)
       //and we need to convert this to (x,y) coordinates on the ground (don't care about Z, we'll handle altitude in a separate algorithm)
        quadcopterLocation2D_UTM = GPStoUTM(currentQuadcopterLatitude, currentQuadcopterLongitude); //need to test this function
        //printf("quad loc. UTM object easting northing zone %f %f %s \n", std::get<eastingIndex>(quadcopterLocation2D_UTM), std::get<northingIndex>(quadcopterLocation2D_UTM), std::get<designatorIndex>(quadcopterLocation2D_UTM).c_str() );
      //so now we have the camera offset from ground (meters), the camera rotation, and the target offset from camera. We need to convert this to camera offset from ground (ie, from center of UTM coordinates)
     //to do so: we say that targetPosition = cameraOffset + cameraRotationMatrix *** targetOffsetFromCamera, where *** denotes matrix multiplication
    //camera_offset is just our known UTM coordinates since the camera is a point mass with the drone in this model.
      
       double targetOffsetFromUAV[3][1];
	  getTargetOffsetFromUAV(
	                           targetDistanceFromCamera_meters 
							   ,cameraRollToGround_radians 
							   ,cameraPitchToGround_radians 
							   ,cameraYawToGround_radians
							   ,targetOffsetFromUAV   ///THIS IS AN OUTPUT VARIABlE THAT WILL BE MODIFIED
							);
        //cout <<"target offset from UAV (x y z intertial)" << targetOffsetFromUAV[0][0] <<" "<< targetOffsetFromUAV[1][0] <<" "<< targetOffsetFromUAV[2][0] <<" ";
	  
    UTMobject targetLocation2D_UTM;
//it's reasonable to assume we don't cross zones
    std::get<designatorIndex>(targetLocation2D_UTM) = std::get<designatorIndex>(quadcopterLocation2D_UTM) ; 
    //printf("CAUTION: target assumed to be in same UTM zone as quadcopter, zone: %s", std::get<designatorIndex>(quadcopterLocation2D_UTM).c_str());
    //can still use the UTMobject since we don't care about the Z offset because we'll handle altitude separately
      std::get<eastingIndex>(targetLocation2D_UTM) = std::get<eastingIndex>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[1][0];//targetLocation2D_UTM.first = quadcopterLocation2D_UTM.first + targetOffsetFromUAV[0][0];
      //targetLocation2D_UTM.second = quadcopterLocation2D_UTM.second + targetOffsetFromUAV[1][0];
      std::get<northingIndex>(targetLocation2D_UTM) = std::get<northingIndex>(quadcopterLocation2D_UTM) + targetOffsetFromUAV[0][0]; 
/*printf("\n target offset from camera is %f x %f y %f z", targetOffsetFromCamera[0][0], targetOffsetFromCamera[1][0], targetOffsetFromCamera[2][0]);
printf("\n rotation matrix is is %f %f %f \n %f %f %f \n %f %f %f \n", cameraRotationMatrix[0][0], cameraRotationMatrix[0][1], cameraRotationMatrix[0][2], cameraRotationMatrix[1][0], cameraRotationMatrix[1][1], cameraRotationMatrix[1][2], cameraRotationMatrix[2][0], cameraRotationMatrix[2][1], cameraRotationMatrix[2][2] );
printf("\n target offset from camera in inertial frame coords is %f x %f y %f z", distanceInRealWorld[0][0], distanceInRealWorld[1][0], distanceInRealWorld[2][0]);
printf("\n target offset from UAV is %f x %f y %f z", targetOffsetFromUAV[0][0], targetOffsetFromUAV[1][0], targetOffsetFromUAV[2][0]);
printf("\n target location UTM is easting %f northing %f zone  %s \n", std::get<eastingIndex>(targetLocation2D_UTM), std::get<northingIndex>(targetLocation2D_UTM), std::get<designatorIndex>(targetLocation2D_UTM).c_str());*/
//now convert back to GPS coordinates and we can generate a proper waypoint

cout <<"\nOffset from UAV x y z inertial " << targetOffsetFromUAV[0][0] <<" "<< targetOffsetFromUAV[1][0] <<" "<< targetOffsetFromUAV[2][0] <<" quadcopter location east north zone " << std::get<eastingIndex>(quadcopterLocation2D_UTM) << " "<< std::get<northingIndex>(quadcopterLocation2D_UTM) <<" "<< std::get<designatorIndex>(quadcopterLocation2D_UTM)  ;
cout <<"\nresult e,n,zone "<< std::get<eastingIndex>(targetLocation2D_UTM) << " "<< std::get<northingIndex>(targetLocation2D_UTM) <<" "<< std::get<designatorIndex>(targetLocation2D_UTM) <<" ";
  return targetLocation2D_UTM;
   


} ///end ffuncition


void goToTargetEstimate(DJIDrone* drone, float latitude, float longitude, float altitude)
{ 
dji_sdk::Waypoint targetEstimate; 
targetEstimate.latitude = latitude;
targetEstimate.longitude = longitude;
targetEstimate.altitude = altitude;
targetEstimate.heading = 0; //I think this means the aircraft flies straight at it
targetEstimate.staytime = 0; //don't wish to linger there

dji_sdk::WaypointList loneWaypoint; //will only be 1 element in this list
loneWaypoint.waypoint_list.push_back(targetEstimate);

drone->waypoint_navigation_send_request(loneWaypoint);

}
//ros::init(argc, argv, "sdk_client");//must come before declaring NodeHandle
ros::NodeHandle* nh; //had to make global so global DJIDRone declaration can use it
DJIDrone* drone;// = new DJIDrone(nh);//had to make global so callback can see it
#define AprilTagsTopic "usb_cam/tag_detections"
#define AprilTagsTopicTracking "dji_sdk/tag_detections"
void apriltagCheckCallback(const apriltags_ros::AprilTagDetectionArray /*sensor_msgs::ImageConstPtr&*/ tag_detection_array)
{
//sensor_msgs::Image rgb = *tag_detection_array; //have to use this to access the data, can't use the ConstPtr for that
apriltags_ros::AprilTagDetectionArray aprilTagsMessage;
aprilTagsMessage = tag_detection_array;
std::vector<apriltags_ros::AprilTagDetection> found;
found = aprilTagsMessage.detections;
int numTags = found.size();

printf("\n I heard [%d] tags", numTags);
for(int i=0; i<numTags; i++)
 	{
        apriltags_ros::AprilTagDetection current=found.at(i);
         int id = current.id;
         printf("\n found id %d", id);
         printf(" with position x %lf y %lf z%lf ", current.pose.pose.position.x, current.pose.pose.position.y, current.pose.pose.position.z);//lf is long double which I think is 64-bit double. Need to use "pose.pose" since the AprilTagDetection contains a "PoseStamped" which then contains a stamp and a pose
        //if(i==0){waypointBasedOnApriltags(id, drone);}
         }
ROS_INFO("\n That's what I Heard");
//ros::Duration(0.5).sleep(); // sleep for half a second

}


////////Turned this piece into a function so it could also easily be called when there is not a target detection, only a prediction
void handleTargetPrediction(cv::Mat targetLocPrediction ,std::string targetUtmZone ,dji_sdk::GlobalPosition copterState ,std_msgs::Header latestHeader ,DJIDrone* drone ){
cout <<"targetLocPrediction " <<targetLocPrediction <<" "; 

 ///following line is just to test basic gimbal control, has nothing to do with rest of code
     // drone->gimbal_angle_control(0, /*-500.0*/-300.0 -0.0,  8, 1); printf("tested gimbal"); //this line has confirmed that we don't need to call sleep after executing a gimbal command
 double predictedNorth = targetLocPrediction.at<double>(0,0); //access element 0,0 ie x
 double predictedEast = targetLocPrediction.at<double>(0,1); //access element 0,1 ie y
cout <<" east north zone " << predictedEast<<" "<< predictedNorth<<" "<< targetUtmZone;
//getting an error like: targetLocPrediction [-0.47096807; 203677.88; -0.00017216911; 3540.2554]  east north zone 1.41522e+26 1.55878e+40 31N
UTMobject predictedTargetUTM; //will need this for later
std::get<northingIndex>(predictedTargetUTM) = predictedNorth;
std::get<eastingIndex>(predictedTargetUTM) = predictedEast;
std::get<designatorIndex>(predictedTargetUTM) = targetUtmZone;

  std::pair<double, double> targetLocPredictionGPS = UTMtoGPS(predictedNorth, predictedEast, targetUtmZone); 
UTMobject actualCopterUTM = GPStoUTM(copterState.latitude, copterState.longitude);  
//printf("CALLBACK: drone position is lat %f long %f alti %f, in easting  and northing %f %f ", copterState.latitude, copterState.longitude, copterState.altitude, std::get<eastingIndex>(actualCopterUTM), std::get<northingIndex>(actualCopterUTM));   

//printf("\n and camera is roll %f pitch %f yaw %f and target distance from camera is x %f y %f z %f", gimbalState.roll, gimbalState.pitch, gimbalState.yaw,LATEST_TARGET_X_CAMERA, LATEST_TARGET_Y_CAMERA, LATEST_TARGET_Z_CAMERA);

//printf("\n and target location predicted is lat %f long %f in UTM easting %f northing %f actual easting and northing calculated were %f -- %f \n\n", targetLocPredictionGPS.latitudeIndex,   targetLocPredictionGPS.longitudeIndex, predictedEast, predictedNorth,targetY, targetX )  ;
//     cout<<"\ntotal prediction" << targetLocPrediction <<"\n";
//     cout<<"\nverify each element" << " 1: " << targetLocPrediction.at<float>(0,0) << " 2: " << targetLocPrediction.at<float>(0,1)  << " 3: " << targetLocPrediction.at<float>(1,0) << " 4: " << targetLocPrediction.at<float>(1,1)  <<"\n";

  // then need to set quadcopter waypoint accordingly
  goToTargetEstimate(drone ,targetLocPredictionGPS.latitudeIndex ,targetLocPredictionGPS.longitudeIndex  , copterState.altitude); //assume same altitude kept throughout until landing
  // then need to to estimate where the quadcopter will actually be
  dji_sdk::Velocity copterSpeed = drone->velocity; //DJI::onboardSDK::VelocityData copterSpeed = DJI::onboardSDK::Flight::getVelocity(); 
  UTMobject predictedCopterUTM = GPStoUTM(copterState.latitude, copterState.longitude); 
           //do a simple velocity times time prediction
  std::get<northingIndex>(predictedCopterUTM) += LATEST_DT * copterSpeed.vx;
 std::get<eastingIndex>(predictedCopterUTM) += LATEST_DT * copterSpeed.vy;

cout <<"predictions before gimbal " <<"copter e n zone " << std::get<eastingIndex>(predictedCopterUTM)<<" "<< std::get<northingIndex>(predictedCopterUTM)<<" "<< std::get<designatorIndex>(predictedCopterUTM)<<" \n";
cout << "target e n zone " << std::get<eastingIndex>(predictedTargetUTM)<<" "<< std::get<northingIndex>(predictedTargetUTM)<<" "<< std::get<designatorIndex>(predictedTargetUTM)<<" \n";
  //then need to modify the gimbal angle to have it point appropriately. This will be done with multiple variables that will be modified by a function, rather than an explicit return
   double yaw_rads;
   double pitch_rads;
   double roll_rads;
    getGimbalAngleToPointAtTarget_rads
    (
 	predictedCopterUTM, 
	copterState.altitude, 
	predictedTargetUTM
 	,yaw_rads //This is an output variable
	,pitch_rads //This is an output variable
	,roll_rads    //This is an output variable
    );
#define NO_PITCH 1 // this is to avoid pitch during testing on the bench
#ifdef NO_PITCH
pitch_rads = degreesToRadians(-10); //small pitch so calculations still make sense
#endif 
if (YAW_RELATIVE_TO_BODY == true)
  {yaw_rads = inertialFrameToBody_yaw(yaw_rads, drone);}


   unsigned char desiredControlMode = 1; //lets you use DJI go gimbal mode selection, and thus free mode.
   unsigned char desiredDuration = 10; //actually it looks like 10 is the lowest it will go//1; //this is the durtion in tenths of a second. I think 1 is the lowest it can go. TODO verify this 

      //now that calculations are over, need to actually assign the gimbal
        /*DJI::onboardSDK::GimbalAngleData desiredGimbalState; //TODO TODO TODO convert the radians to the 0.1 degrees used by their gimbal angle calculations
        desiredGimbalState.yaw = yaw_rads ; 
        desiredGimbalState.pitch = pitch_rads ; 
        desiredGimbalState.roll = roll_rads ; 
        desiredGimbalState.mode = desiredControlMode;
        desiredGimbalState.duration = desiredDuration ; 
      //now actually send the command 
       //DJI::onboardSDK::Camera::setGimbalAngle(desiredGimbalState); */ //I can't quite figure out how to use this, so let's use this instead
     //cout <<"calculated camera angle (tenths of a degree): " << " Roll : " <<  radiansToDjiUnits(roll_rads) <<  " Pitch : " << radiansToDjiUnits(pitch_rads) <<  " Yaw : " <<   radiansToDjiUnits(yaw_rads);
 
    //drone->gimbal_angle_control(radiansToDjiUnits(roll_rads), radiansToDjiUnits(pitch_rads), radiansToDjiUnits(yaw_rads), desiredDuration, desiredControlMode);
     //skip the angle control and let a PID control (outside this function loop) handle it
	 GLOBAL_ROLL_DJI_UNITS = radiansToDjiUnits(roll_rads); 
	 GLOBAL_PITCH_DJI_UNITS = radiansToDjiUnits(pitch_rads);
	 GLOBAL_YAW_DJI_UNITS = radiansToDjiUnits(yaw_rads);
 //then I think we're done with this step
     
     // if we want to use a separate nod for PID calculations, need to publish them here
	 //the following link provides a good guide: http://answers.ros.org/question/48727/publisher-and-subscriber-in-the-same-node/
	 //for simplicity, I'm going to send the desired angle in the form of a pointStamped message (point with timestamp)
	//I will translate the indices according to their standard order, ie, since x comes before y and roll comes before pitch.
	// roll will be the x element and pitch will be the y element
	#define rollIndex x
	#define pitchIndex  y
	#define yawIndex z
     geometry_msgs::PointStamped desiredAngle ;	
     desiredAngle.point.rollIndex = GLOBAL_ROLL_DJI_UNITS;
	 desiredAngle.point.pitchIndex = GLOBAL_PITCH_DJI_UNITS;
	 desiredAngle.point.yawIndex = GLOBAL_YAW_DJI_UNITS;
	  desiredAngle.header = latestHeader ; //send the same time stamp information that was on the apriltags message to the PID node
	 GLOBAL_ANGLE_PUBLISHER.publish(desiredAngle);
}
//end handleTargetPrediction





void apriltagCheckCallbackForTracking(const apriltags_ros::AprilTagDetectionArray /*sensor_msgs::ImageConstPtr&*/ tag_detection_array)
{

	
UTMobject latestTargetLocation; //must declare before the if statements so it can be used for further estimates (by recording the UTM designator zone) if the target is lost	
	
std_msgs::Header latestHeader ; //declare outside the if statements so it can be used at the end	
//sensor_msgs::Image rgb = *tag_detection_array; //have to use this to access the data, can't use the ConstPtr for that
apriltags_ros::AprilTagDetectionArray aprilTagsMessage;
aprilTagsMessage = tag_detection_array;
std::vector<apriltags_ros::AprilTagDetection> found;
found = aprilTagsMessage.detections;
int numTags = found.size();

//printf("\n I heard [%d] tags", numTags);
/*for(int i=0; i<numTags; i++)
 	{
        apriltags_ros::AprilTagDetection current=found.at(i);
         int id = current.id;
         printf("\n found id %d", id);
         printf(" with position x %lf y %lf z%lf ", current.pose.pose.position.x, current.pose.pose.position.y, current.pose.pose.position.z);//lf is long double which I think is 64-bit double. Need to use "pose.pose" since the AprilTagDetection contains a "PoseStamped" which then contains a stamp and a pose
        //if(i==0){waypointBasedOnApriltags(id, drone);}
         }
ROS_INFO("\n That's what I Heard");
//ros::Duration(0.5).sleep(); // sleep for half a second*/

// let's assume that if there are multiple tags, we only want to deal with the first one.
if (numTags > 0 ) //TODO : correct flaw in logic here, such that if we lose the target we still perform he calculations based on estimates 
  {
  FRAMES_WITHOUT_TARGET = 0; //we've found the target again
  
  
  apriltags_ros::AprilTagDetection current=found.at(0);
  current.pose.pose.position.y = CAMERA_Y_MULTIPLIER * current.pose.pose.position.y;  //since we want to ensure up, relative to the camera, is treated as positive y in the camera frame  
  latestHeader = current.pose.header; 

//update the global variables containing the coordinates (in the camera frame remember) (
  LATEST_TARGET_X_CAMERA = current.pose.pose.position.x;
  LATEST_TARGET_Y_CAMERA =  current.pose.pose.position.y; 
  LATEST_TARGET_Z_CAMERA = current.pose.pose.position.z;
//Need to use "pose.pose" since the AprilTagDetection contains a "PoseStamped" which then contains a stamp and a pose
  // TODO make sure that your variables can handle this calculation
   //ros::time tStamp = current.pose.header.stamp;//this gives an error for some reason
   double currentTime = current.pose.header.stamp.nsec/1000000000.0 + current.pose.header.stamp.sec;
  //figure out elapsed time between detections, necessary for kalman filtering
   LATEST_DT = currentTime - LATEST_TIMESTAMP ;
 //be sure to keep track of time
   LATEST_TIMESTAMP = currentTime;
  
//now begin calculations to figure out target position in northings and eastings
     dji_sdk::Gimbal gimbalState = drone->gimbal;//DJI::onboardSDK::GimbalData gimbalState = (drone->gimbal).getGimbal();
     dji_sdk::GlobalPosition copterState = drone->global_position; //DJI::onboardSDK::PositionData copterState = drone->global_position; //DJI::onboardSDK::Flight::getPosition();
      latestTargetLocation = targetDistanceMetersToUTM( // Note that gimbal result is in degrees, but gimbal control is in tenths of a dgree
			current.pose.pose.position , 	      				degreesToRadians(gimbalState.roll), 
			degreesToRadians(gimbalState.pitch), 
			degreesToRadians(gimbalState.yaw/*bodyFrameToInertial_yaw(gimbalState.yaw,drone)*/)// since yaw is relative to body, not inertial frame, we need to convert//degreesToRadians(gimbalState.yaw) 
			,copterState.latitude 
			,copterState.longitude 
			,copterState.altitude //TODO decide if we should use .height instead
			);
cout <<" verify result e,n,zone "<< std::get<eastingIndex>(latestTargetLocation) << " "<< std::get<northingIndex>(latestTargetLocation) <<" "<< std::get<designatorIndex>(latestTargetLocation)  <<"\n";

double debugAr[3][1]; 
getTargetOffsetFromUAV(current.pose.pose.position, degreesToRadians(gimbalState.roll), degreesToRadians(gimbalState.pitch), degreesToRadians(gimbalState.yaw), debugAr);


//recall that x is north, y is east, and we need these values to pass the Kalman filter
//temporarily, let' just use the camera offset so it has small numbers to work with
    double targetX = std::get<northingIndex>(latestTargetLocation); //LATEST_TARGET_X_CAMERA; //std::get<northingIndex>(latestTargetLocation);
   double targetY = std::get<eastingIndex>(latestTargetLocation); //LATEST_TARGET_Y_CAMERA; //std::get<eastingIndex>(latestTargetLocation);
  cv::Mat targetLocPrediction;
  cv::Mat targetXandZFromCamera ; //for debugging the kalman filter

  if(! ( IS_TRACKING) )
   {

     
     GLOBAL_KALMAN_FILTER = initializeKalmanFilter(LATEST_DT, targetX ,targetY ); 
	 GLOBAL_KALMAN_FILTER_DIST = initializeKalmanFilter(LATEST_DT, debugAr[0][0], debugAr[2][0]); //track side-side distance and distance from camera for debugging



       targetLocPrediction = targetTrackStep(GLOBAL_KALMAN_FILTER, LATEST_DT, targetX, targetY);//GLOBAL_KALMAN_FILTER.predict() ; //TODO TODO make sure this isn't causing a double calculation or something!
		targetXandZFromCamera = 	  targetXandZFromCamera = targetTrackStep(GLOBAL_KALMAN_FILTER_DIST, LATEST_DT, debugAr[0][0], debugAr[2][0]); //GLOBAL_KALMAN_FILTER_DIST.predict();
		 cout <<"real x z: " << debugAr[0][0] << " " << debugAr[2][0] <<" prediction dist from camera initial (x and z) " << targetXandZFromCamera <<"\n";
    // String stall;
    // cout <<"\nCIN pause: press a key then hit enter to contiue\n"; 
    // cin>> stall; 
    }

 else
    {

       
      targetLocPrediction = targetTrackStep(GLOBAL_KALMAN_FILTER, LATEST_DT, targetX, targetY); 
	  targetXandZFromCamera = targetTrackStep(GLOBAL_KALMAN_FILTER_DIST, LATEST_DT, debugAr[0][0], debugAr[2][0]); 
     // cout <<"kalman filter" << "process noise " << GLOBAL_KALMAN_FILTER.processNoiseCov <<"\n measurement Noise " <<  GLOBAL_KALMAN_FILTER.measurementNoiseCov << "\ntransition matrix: "<<GLOBAL_KALMAN_FILTER.transitionMatrix <<"\n";
    // cout<<"dt: "<<LATEST_DT <<"\n";
			 cout <<"real x z: " << debugAr[0][0] << " " << debugAr[2][0] <<" prediction dist from camera (x and z) " << targetXandZFromCamera <<"\n";

     }
  IS_TRACKING = true;

 
                handleTargetPrediction( targetLocPrediction, std::get<designatorIndex>(latestTargetLocation), copterState , latestHeader , drone);
	 
   }

  else // if no detections then we can't track it
    {
	 
    //my concern is that by declaring it false every time there isn't one, we might end up never tracking it
    //IS_TRACKING = false; //couldn't find one so we're obviously not tracking yet. 
	   if(IS_TRACKING == true)
	      {
		     FRAMES_WITHOUT_TARGET ++ ; 
	        if(FRAMES_WITHOUT_TARGET >= FRAMES_UNTIL_TARGET_LOST)
					{
					 IS_TRACKING = false; 
					  FRAMES_WITHOUT_TARGET = 0;
					}
			else
					{
						//
				     
					 cv::Mat targetLocPrediction = targetEstimateWithoutMeasurement(GLOBAL_KALMAN_FILTER, LATEST_DT);
					 dji_sdk::GlobalPosition copterState = drone->global_position;
					 

					handleTargetPrediction(targetLocPrediction, std::get<designatorIndex>(latestTargetLocation), copterState , latestHeader , drone); 
					 cv::Mat targetXandZFromCamera = targetEstimateWithoutMeasurement(GLOBAL_KALMAN_FILTER_DIST, LATEST_DT);
			           cout <<" without measurement  prediction dist from camera (x and z) " << targetXandZFromCamera <<"\n";
					 
					}	
		  }
     }
}

#define numMessagesToBuffer 1 //10
void listenOption(ros::NodeHandle& n)
{
ros::Subscriber sub = n.subscribe(AprilTagsTopic, numMessagesToBuffer, apriltagCheckCallback);
printf("\n After the callback line");
ros::spin(); 
}
void listenOptionForTracking(ros::NodeHandle& n)
{

ros::Subscriber sub = n.subscribe(AprilTagsTopicTracking, numMessagesToBuffer, apriltagCheckCallbackForTracking);
printf("\n After the callback line");
ros::spin(); 
//I believe that spin tests if there are any outstanding messages then stops. So putting the following underneath it
//should result in near-constant PID control between spins
double defaultTimeStep = 0.01; //since near constant control, use a small numbers
dji_sdk::Gimbal currentAngle = drone->gimbal;
double rollSpeedDesired  = getRequiredVelocityPID(GLOBAL_ROLL_DJI_UNITS, degreesToDjiUnits(currentAngle.roll),defaultTimeStep, GLOBAL_ROLL_CONTROLLER);
double pitchSpeedDesired  = getRequiredVelocityPID(GLOBAL_PITCH_DJI_UNITS, degreesToDjiUnits(currentAngle.pitch),defaultTimeStep, GLOBAL_PITCH_CONTROLLER);
double yawSpeedDesired  = getRequiredVelocityPID(GLOBAL_YAW_DJI_UNITS, degreesToDjiUnits(currentAngle.yaw),defaultTimeStep, GLOBAL_YAW_CONTROLLER);
drone->gimbal_speed_control(rollSpeedDesired, pitchSpeedDesired, yawSpeedDesired);

}



//following function is to test for very basic errors in the geolocalization functions. It will be updated as more functions are added
void dummyTest_geolocalization()
{
   geometry_msgs::Point pTest; 
   pTest.x = 100;
   pTest.y = -2;
   pTest.z = -0.04; 
   dji_sdk::Waypoint wpTest = targetDistanceMetersToLongitude(pTest, 0.5, 1.75, 2.3, 101.0, 45.02357, 15);
   
  double testLat = 42.321838 ;
  double testLong =  -83.23293;
   std::pair<double, double> latLong;
   latLong.latitudeIndex = testLat ;
   latLong.longitudeIndex = testLong ;
   printf("\n starting lat and long in degrees: %f and %f ", testLat, testLong); 
   UTMobject utmTest = GPStoUTM(latLong.latitudeIndex, latLong.longitudeIndex); 
      printf("\n UTM translation: northings: %f eastings: %f  zone: %s \n", std::get<northingIndex>(utmTest), std::get<eastingIndex>(utmTest), std::get<designatorIndex>(utmTest).c_str() ); 
  std::pair<double, double> backToGPS = UTMtoGPS( std::get<northingIndex>(utmTest), std::get<eastingIndex>(utmTest), std::get<designatorIndex>(utmTest) );

      printf("\n re-converted lat and long in degrees: %f and %f \n", backToGPS.latitudeIndex, backToGPS.longitudeIndex);
       
printf("\n \n beginning first target localization test case: camera is pointing straight down, 15 meters above a target at location lat %f  long %f \n" , testLat, testLong);
geometry_msgs::Point distanceFromCamera;
//right underneath
distanceFromCamera.x=0; distanceFromCamera.y=0; distanceFromCamera.z=15;
double roll_rad =0; //not rolled, directly underneath
double yaw_rad=0; //assume no yaw
double pitch_rad = -1.0 * M_PI/2.0 ;  //pointing directly down ( I think positive points it down)
dji_sdk::Waypoint testwp = targetDistanceMetersToLongitude
     (
	distanceFromCamera, 
	      roll_rad, 
	pitch_rad, 
	yaw_rad 
	,testLat 
	,testLong
	,15.0
     );

double resultLat = testwp.latitude;
double resultLong = testwp.longitude;

printf("resulting target lat and long %f %f \n", resultLat, resultLong);

double offset = 20.0;
printf("\n \n beginning second target localization test case: camera is pointing straight down, 15 meters above a target at location lat %f  long %f , but this time target location is %f M up in both x and y direction \n" , testLat, testLong, offset);
distanceFromCamera.x = offset;
distanceFromCamera.y = offset;
 testwp = targetDistanceMetersToLongitude
     (
	distanceFromCamera, 
	      roll_rad, 
	pitch_rad, 
	yaw_rad 
	,testLat 
	,testLong,
        15.0
     );

 resultLat = testwp.latitude;
 resultLong = testwp.longitude;

printf("resulting target lat and long %f %f \n", resultLat, resultLong);

printf("\n \n beginning third target localization test case: camera is pointing 45 degrees down, and yawed +45 degrees (ie, 45 degrees to the east IF I UNDERSTAND THAT SYSTEM CORRECTLY), 15 meters away from a target with camera at location lat %f  long %f . UAV is at altitude of 22.0 meters \n" , testLat, testLong);
distanceFromCamera.x=0; distanceFromCamera.y=0; distanceFromCamera.z=15;
 roll_rad =0; //not rolled, 
 yaw_rad= M_PI/4.0; //yawed 45 degrees right
 pitch_rad = -1.0 * M_PI/4.0 ;  //45 degrees down
 testwp = targetDistanceMetersToLongitude
     (
	distanceFromCamera, 
	      roll_rad, 
	pitch_rad, 
	yaw_rad 
	,testLat 
	,testLong,
        22.0
     );

 resultLat = testwp.latitude;
 resultLong = testwp.longitude;

printf("resulting target lat and long %f %f \n", resultLat, resultLong);

   printf("geolocalization dummy test completed");
}


//following function is to test for very basic errors in the gimbal functions. It will be updated as more functions are added

void dummyTest_gimbal()
{
 std::string zone = "17T";

 double copterEasting = 316004.0 ;
 double copterNorthing = 4687925.0;
 double copterAltitude_Meters = 50;
 
 double targetOffsetNorthings = 20;
 double targetOffsetEastings = 10;

 double targetNorthing = copterNorthing + targetOffsetNorthings;
 double targetEasting = copterEasting + targetOffsetEastings; 
 
 UTMobject quadcopterLocation;
 std::get<northingIndex>(quadcopterLocation) = copterNorthing;
 std::get<eastingIndex>(quadcopterLocation) = copterEasting;
 std::get<designatorIndex>(quadcopterLocation) = zone;
 UTMobject targetLocation;
 std::get<northingIndex>(targetLocation) = targetNorthing;
 std::get<eastingIndex>(targetLocation) = targetEasting;
 std::get<designatorIndex>(targetLocation) = zone;

double yaw;
double pitch;
double roll;
 getGimbalAngleToPointAtTarget_rads(quadcopterLocation, copterAltitude_Meters, targetLocation, yaw, pitch, roll) ;

 printf("gimbal test 1. Quadcopter location at northing: %f easting: %f zone %s , altitude %f meters, target offset by %f m North and %f m east. To point at it, we'll need gimbal offset of (radians): roll %f pitch %f yaw %f \n", copterNorthing, copterEasting, zone.c_str(), copterAltitude_Meters, targetOffsetNorthings, targetOffsetEastings, roll, pitch, yaw); 


targetOffsetNorthings = -20;
targetOffsetEastings = -20;
targetNorthing = copterNorthing + targetOffsetNorthings;
targetEasting = copterEasting + targetOffsetEastings;
 std::get<northingIndex>(targetLocation) = targetNorthing;
 std::get<eastingIndex>(targetLocation) = targetEasting;

getGimbalAngleToPointAtTarget_rads(quadcopterLocation, copterAltitude_Meters, targetLocation, yaw, pitch, roll) ;
 printf("gimbal test 2. Target \"behind\" quadcopter. Quadcopter location at northing: %f easting: %f zone %s , altitude %f meters, target offset by %f m North and %f m east. To point at it, we'll need gimbal offset of (radians): roll %f pitch %f yaw %f \n", copterNorthing, copterEasting, zone.c_str(), copterAltitude_Meters, targetOffsetNorthings, targetOffsetEastings, roll, pitch, yaw); 

printf("gimbal dummy tests completed \n"); 

}


//following function is to test for very basic errors in the latest functions. It will be updated as more functions are added

void trackTarget() 
{
//This will consist of  steps
//1: Wait for initial target detection
//2: On first target detection, calculate target location, create kalman filter,  fly to this target's location, and calculate where the quadcopter will be after a timestep, and thus the gimbal angle for pointing at this location
//3: Update target location and quadcopter location
//4: Update kalman filtration
//5: Use that to estimate where target will be in next timestep. Set waypoint. Estimate where quadcopter will be at next timestep
//6: Set gimbal accordingly
//7: Repeat steps 3-6

}

void dummyTest()
{
dummyTest_geolocalization();
printf("geolocalization tests done");
printf("\n beginning gimbal tests \n");
dummyTest_gimbal();
printf("\n gimbal tests done"); 
for (int a =0; a<10; a++){
printf("running kalman filter test, check CSV file");}
kalmanTest();
printf("\n\nall dummy tests done \n\n");

}

//end functions for integration with AprilTags and geolocalization











using namespace DJI::onboardSDK;

//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);


static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23] Hotpoint Mission Upload     |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Take a Picture           | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Start Record Video       | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Stop Record Video        | [32] Mission Hotpoint Set Speed  |\n");	
	printf("| [14] Local Navigation Test    | [33] Mission Hotpoint Set Radius |\n");	
	printf("| [15] Global Navigation Test   | [34] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [16] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [17] Arm the Drone            | [36] Mission Hotpoint Download   |\n");	
	printf("| [18] Disarm the Drone         | [37] Enter Mobile commands mode  |\n");
    printf("| [19] Virtual RC Test           \n");
    printf("| [54] Geolocalization/Gimbal tests and AprilTag recognition)   |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

   
int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle n; //had to make global so global DJIDRone declaration can use it
    nh = &n;
    //DJIDrone* drone = new DJIDrone(nh);

    drone = new DJIDrone(n);
	//virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;
    ros::spinOnce();
    
    //! Setting functions to be called for Mobile App Commands mode 
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

	
    Display_Main_Menu();
    while(1)
    {
        ros::spinOnce();
        std::cout << "Enter Input Val: ";
        std::cin >> temp32;

        if(temp32>0 && temp32<38 || temp32 >= 54) //start ours at 54 to make it distinct from DJI's
        {
            main_operate_code = temp32;         
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
        switch(main_operate_code)
        {
			case 1:
				/* SDK version query*/
				drone->check_version();
				break;
            case 2:
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 3:
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 4:
                /* take off */
                drone->takeoff();
                break;
            case 5:
                /* landing*/
                drone->landing();
                break;
            case 6:
                /* go home*/
                drone->gohome();
                break;
            case 7:
                /*gimbal test*/


                /* drone->gimbal_angle_control(0, -900, 0, 11);

                sleep(1);



                drone->gimbal_angle_control(0, 500, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -300, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 100, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -900, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -900, -1800, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -900, -900, 10);
                sleep(2);
                printf("\n and camera is roll %f pitch %f yaw %f ", drone->gimbal.roll, drone->gimbal.pitch, drone->gimbal.yaw);*/

                /*drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(0.8);
                drone->gimbal_speed_control(0, 0, 200); 
                sleep(2);
                drone->gimbal_speed_control(0, 0, -1800); //input of 400  seems to mvoe it 20 degrees, leading me to conclude that it moves for 0.5 seconds
                sleep(1);
                drone->gimbal_speed_control(0, 0, 1600); //input of 400  seems to mvoe it 20 degrees, leading me to conclude that it moves for 0.5 seconds
                sleep(0.75);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
                drone->gimbal_angle_control(0, 500, 0, 10);
                sleep(3);

                drone->gimbal_speed_control(0, -1800, 0); //1800 is the maximum speed for pitch (-1800 worked, -1801 didnt execute). This is also maximum for yaw(-1800 works, -1801 doesn't)

                sleep(2);
               // drone->gimbal_speed_control(0, -900, 0); */
                
                drone->gimbal_angle_control(0, -450, 450, 10);
                sleep(3);
                GIMBAL_TEST_SIGN = 1;
                while(1)
                {

                    degs = (degs+10)%100;
                    // //oscillate between 10, 20, 30, ... 90 degrees/second
                    if(drone->gimbal.pitch >= 20.0) 
                    {
                        drone->gimbal_speed_control(0, -1*10*degs, 0); GIMBAL_TEST_SIGN=-1;
                    } 
                    else if  (drone->gimbal.pitch < -20) 
                    {
                        GIMBAL_TEST_SIGN=1; drone->gimbal_speed_control(0, 1*10*degs, 0);
                    } //it doesn't seem to be flipping to a negative sign even after I declared it volatile
                        
                    cout<<"degs " <<degs <<"sign " << GIMBAL_TEST_SIGN <<"\n"; 
                }
  
                sleep(2);

                /*drone->gimbal_angle_control(0, 0, -1800, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, -2700, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 900, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 1800, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 2700, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 10);
                sleep(4);

                drone->gimbal_angle_control(0, 0, -1800, 10);
                sleep(2);
                drone->gimbal_angle_control(300, 0, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(-300, 0, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 300, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -300, 0, 10);
                sleep(2);

                drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(0, 0, 200);
                sleep(2);
                drone->gimbal_speed_control(0, 0, -200);
                sleep(2);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
                drone->gimbal_speed_control(0, -200, 0);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 20);/**/
                break;

            case 8:
                /* attitude control sample*/
                drone->takeoff();
                sleep(8);


                for(int i = 0; i < 100; i ++)
                {
                    if(i < 90)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, -2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, -2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, 0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, -0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, 90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, -90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                drone->landing();

                break;

            case 9:
                /*draw circle sample*/
                static float R = 2;
                static float V = 2;
                static float x;
                static float y;
                Phi = 0;
                std::cout<<"Enter the radius of the circle in meteres (10m > x > 4m)\n";
                std::cin>>circleRadius;   

                std::cout<<"Enter height in meteres (Relative to take off point. 15m > x > 5m) \n";
                std::cin>>circleHeight;  

                 if (circleHeight < 5)
                {
                    circleHeight = 5;
                }
                else if (circleHeight > 15)
                {
                    circleHeight = 15;
                }           
                if (circleRadius < 4)
                {
                    circleRadius = 4;
                }
                else if (circleRadius > 10)
                {
                    circleRadius = 10;
                } 
		
                x_center = drone->local_position.x;
                y_center = drone->local_position.y;
                circleRadiusIncrements = 0.01;
		
    		    for(int j = 0; j < 1000; j ++)
                {   
                  if (circleRadiusIncrements < circleRadius)
    			   {
    		        x =  x_center + circleRadiusIncrements;
    		        y =  y_center;
    			    circleRadiusIncrements = circleRadiusIncrements + 0.01;
    		        drone->local_position_control(x ,y ,circleHeight, 0);
    		        usleep(20000);
    			  }
                   else
    			   {
                    break;
                   }
                }

                /* start to draw circle */
                for(int i = 0; i < 1890; i ++)
                {   
                    x =  x_center + circleRadius*cos((Phi/300));
                    y =  y_center + circleRadius*sin((Phi/300));
                    Phi = Phi+1;
                    drone->local_position_control(x ,y ,circleHeight, 0);
                    usleep(20000);
                }
                break;

            case 10:
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, 3, 0, 0 );
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, -3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, -3, 0, 0);
                    usleep(20000);
                }
                break;
            case 11:
                /*take a picture*/
                drone->take_picture();
                break;
            case 12:
                /*start video*/
                drone->start_video();
                break;
            case 13:
                /*stop video*/
                drone->stop_video();
                break;
            case 14:
                /* Local Navi Test */
                drone->local_position_navigation_send_request(-100, -100, 100);
                break;
            case 15:
                /* GPS Navi Test */
                drone->global_position_navigation_send_request(22.5420, 113.9580, 10);
                break;
            case 16:
                /* Waypoint List Navi Test */
                {
                    waypoint0.latitude = 22.535;
                    waypoint0.longitude = 113.95;
                    waypoint0.altitude = 100;
                    waypoint0.staytime = 5;
                    waypoint0.heading = 0;
                }
                newWaypointList.waypoint_list.push_back(waypoint0);

                {
                    waypoint1.latitude = 22.535;
                    waypoint1.longitude = 113.96;
                    waypoint1.altitude = 100;
                    waypoint1.staytime = 0;
                    waypoint1.heading = 90;
                }
                newWaypointList.waypoint_list.push_back(waypoint1);

                {
                    waypoint2.latitude = 22.545;
                    waypoint2.longitude = 113.96;
                    waypoint2.altitude = 100;
                    waypoint2.staytime = 4;
                    waypoint2.heading = -90;
                }
                newWaypointList.waypoint_list.push_back(waypoint2);

                {
                    waypoint3.latitude = 22.545;
                    waypoint3.longitude = 113.96;
                    waypoint3.altitude = 10;
                    waypoint3.staytime = 2;
                    waypoint3.heading = 180;
                }
                newWaypointList.waypoint_list.push_back(waypoint3);

                {
                    waypoint4.latitude = 22.525;
                    waypoint4.longitude = 113.93;
                    waypoint4.altitude = 50;
                    waypoint4.staytime = 0;
                    waypoint4.heading = -180;
                }
                newWaypointList.waypoint_list.push_back(waypoint4);

                drone->waypoint_navigation_send_request(newWaypointList);
                break;
			case 17:
				//drone arm
				drone->drone_arm();
                break;
			case 18:
				//drone disarm
				drone->drone_disarm();
                break;
			case 19:
				//virtual rc test 1: arm & disarm
				drone->virtual_rc_enable();
				usleep(20000);

				virtual_rc_data[0] = 1024;	//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;	//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024+660;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;	//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1684;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for (int i = 0; i < 100; i++){
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}

				//virtual rc test 2: yaw 
				drone->virtual_rc_enable();
				virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024-200;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for(int i = 0; i < 100; i++) {
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}
				drone->virtual_rc_disable();
				break;
			case 20:
				//sync flag
				drone->sync_flag_control(1);
				break;
			case 21:
				//set msg frequency
				drone->set_message_frequency(msg_frequency_data);
				break;

			case 22:

                // Clear the vector of previous waypoints 
                waypoint_task.mission_waypoint.clear();
                
				//mission waypoint upload
				waypoint_task.velocity_range = 10;
				waypoint_task.idle_velocity = 3;
				waypoint_task.action_on_finish = 0;
				waypoint_task.mission_exec_times = 1;
				waypoint_task.yaw_mode = 4;
				waypoint_task.trace_mode = 0;
				waypoint_task.action_on_rc_lost = 0;
				waypoint_task.gimbal_pitch_mode = 0;

                static int num_waypoints = 4; 
                static int altitude = 80;
                // Currently hard coded, should be dynamic
                static float orig_lat = 22.5401;
                static float orig_long = 113.9468;
                for(int i = 0; i < num_waypoints; i++)
                {
                    
                    // Careens in zig-zag pattern
    				waypoint.latitude = (orig_lat+=.0001);
                    if (i % 2 == 1){
    				    waypoint.longitude = orig_long+=.0001;
                    } else {
    				    waypoint.longitude = orig_long;
                    }
    				waypoint.altitude = altitude-=10;
    				waypoint.damping_distance = 0;
    				waypoint.target_yaw = 0;
    				waypoint.target_gimbal_pitch = 0;
    				waypoint.turn_mode = 0;
    				waypoint.has_action = 0;
    				/*
    				waypoint.action_time_limit = 10;
    				waypoint.waypoint_action.action_repeat = 1;
    				waypoint.waypoint_action.command_list[0] = 1;
    				waypoint.waypoint_action.command_parameter[0] = 1;
    				*/
    
    				waypoint_task.mission_waypoint.push_back(waypoint);
                } 
                
                /* 

				waypoint.latitude = 22.540015;
				waypoint.longitude = 113.94659;
				waypoint.altitude = 120;
				waypoint.damping_distance = 2;
				waypoint.target_yaw = 180;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 0;
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_list[1] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;
				waypoint.waypoint_action.command_parameter[1] = 1;


				waypoint_task.mission_waypoint.push_back(waypoint);

                */

				drone->mission_waypoint_upload(waypoint_task);
				break;
			case 23:
				//mission hotpoint upload
				hotpoint_task.latitude = 22.542813;
				hotpoint_task.longitude = 113.958902;
				hotpoint_task.altitude = 20;
				hotpoint_task.radius = 10;
				hotpoint_task.angular_speed = 10;
				hotpoint_task.is_clockwise = 0;
				hotpoint_task.start_point = 0;
				hotpoint_task.yaw_mode = 0;

				drone->mission_hotpoint_upload(hotpoint_task);
				break;
			case 24:
				//mission followme upload
				followme_task.mode = 0;
				followme_task.yaw_mode = 0;
				followme_task.initial_latitude = 23.540091;
				followme_task.initial_longitude = 113.946593;
				followme_task.initial_altitude = 10;
				followme_task.sensitivity = 1;

				drone->mission_followme_upload(followme_task);
				break;
			case 25:
				//mission start
				drone->mission_start();
				break;
			case 26:
				//mission pause
				drone->mission_pause();
				break;
			case 27:
				//mission resume
				drone->mission_resume();
				break;
			case 28:
				//mission cancel
				drone->mission_cancel();
				break;
			case 29:
				//waypoint mission download
				waypoint_task = drone->mission_waypoint_download();
				break;
			case 30:
				//mission waypoint set speed
				drone->mission_waypoint_set_speed((float)5);
				break;
			case 31:
				//mission waypoint get speed
				printf("%f", drone->mission_waypoint_get_speed());
				break;
			case 32:
				//mission hotpoint set speed
				drone->mission_hotpoint_set_speed((float)5,(uint8_t)1);
				break;
			case 33:
				//mission hotpoint set radius
				drone->mission_hotpoint_set_radius((float)5);
				break;
			case 34:
				//mission hotpoint reset yaw
				drone->mission_hotpoint_reset_yaw();
				break;
			case 35:
				//mission followme update target
				for (int i = 0; i < 20; i++)
				{
					followme_target.latitude = 22.540091 + i*0.000001;
					followme_target.longitude = 113.946593 + i*0.000001;
					followme_target.altitude = 100;
					drone->mission_followme_update_target(followme_target);
					usleep(20000);
				}
				break;
            case 37:
                printf("Mobile Data Commands mode entered. Use OSDK Mobile App to use this feature \n");
                printf("End program to exit this mode \n");
                while(1)
                {             
                ros::spinOnce();  
                }
			case 36:
				hotpoint_task = drone->mission_hotpoint_download();

            case 54: //replace with number so it works, but different number so we know it's not DJI's 

				//replace this with testing if it can read AprilTags stuff:
				//dummyTest(); //REMOVE before actual use!
				GLOBAL_ANGLE_PUBLISHER = (*nh).advertise<geometry_msgs::PointStamped>("/dji_sdk/desired_angle", 2); // queue size of 2 seems reasonable
				drone->check_version(); //TODO need to find a way to get the return value from this
				drone->request_sdk_permission_control(); //TODO need to find a way to get the return value from this
				drone->takeoff(); //TODO need to find a way to get the return value from this
				//while(drone->global_position.height< 1.5) //set low in case it doesn't reach the full 2 meters 
				//{
                //    ros::Duration(0.1).sleep(); 
                //} //let it reach the right height
				
                ros::Duration(3.0).sleep(); //3 seconds to stabilize after taking off
				printf ("Starting to listen for AprilTags on %s", AprilTagsTopicTracking );
				//listenOption(*nh);
				
                char waitKeyChar; 
				listenOptionForTracking(*nh); 
				waitKeyChar = cv::waitKey(1);
				if (waitKeyChar == 'q' || waitKeyChar == 'Q') //attempt to allow this to be exited by pressing q. Still need to test
				{
                    break;
                }
            
            default:
                break;
        }
        main_operate_code = -1;
        Display_Main_Menu();
    }
    return 0;
}

//! Callback functions for Mobile Commands
    void ObtainControlMobileCallback(DJIDrone *drone)
    {
      drone->request_sdk_permission_control();
    }

    void ReleaseControlMobileCallback(DJIDrone *drone)
    {
      drone->release_sdk_permission_control();
    }

    void TakeOffMobileCallback(DJIDrone *drone)
    {
      drone->takeoff();
    }

    void LandingMobileCallback(DJIDrone *drone)
    {
      drone->landing();
    }

    void GetSDKVersionMobileCallback(DJIDrone *drone)
    {
      drone->check_version();
    }

    void ArmMobileCallback(DJIDrone *drone)
    {
      drone->drone_arm();
    }

    void DisarmMobileCallback(DJIDrone *drone)
    {
      drone->drone_disarm();
    }

    void GoHomeMobileCallback(DJIDrone *drone)
    {
      drone->gohome();
    }

    void TakePhotoMobileCallback(DJIDrone *drone)
    {
      drone->take_picture();
    }

    void StartVideoMobileCallback(DJIDrone *drone)
    {
      drone->start_video();
    }

    void StopVideoMobileCallback(DJIDrone *drone)
    {
      drone->stop_video();
    }

    void DrawCircleDemoMobileCallback(DJIDrone *drone)
    {
        static float R = 2;
        static float V = 2;
        static float x;
        static float y;
        int circleRadius;
        int circleHeight;
        float Phi =0, circleRadiusIncrements;
        int x_center, y_center, yaw_local; 

        circleHeight = 7;
        circleRadius = 7;

        x_center = drone->local_position.x;
        y_center = drone->local_position.y;
        circleRadiusIncrements = 0.01;

        for(int j = 0; j < 1000; j ++)
        {   
            if (circleRadiusIncrements < circleRadius)
            {
                x =  x_center + circleRadiusIncrements;
                y =  y_center;
                circleRadiusIncrements = circleRadiusIncrements + 0.01;
                drone->local_position_control(x ,y ,circleHeight, 0);
                usleep(20000);
            }
                else
            {
                break;
            }
        }
        

        /* start to draw circle */
        for(int i = 0; i < 1890; i ++)
        {   
            x =  x_center + circleRadius*cos((Phi/300));
            y =  y_center + circleRadius*sin((Phi/300));
            Phi = Phi+1;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }

    }
    void DrawSquareDemoMobileCallback(DJIDrone *drone)
    {
    /*draw square sample*/
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, 3, 0, 0 );
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, 3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, -3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, -3, 0, 0);
            usleep(20000);
        }
    }

     void GimbalControlDemoMobileCallback(DJIDrone *drone)
        {
        drone->gimbal_angle_control(0, 0, 1800, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 0, -1800, 20);
        sleep(2);
        drone->gimbal_angle_control(300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(-300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 300, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, -300, 0, 20);
        sleep(2);
        drone->gimbal_speed_control(100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(-100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(0, 0, 200);
        sleep(2);
        drone->gimbal_speed_control(0, 0, -200);
        sleep(2);
        drone->gimbal_speed_control(0, 200, 0);
        sleep(2);
        drone->gimbal_speed_control(0, -200, 0);
        sleep(2);
        drone->gimbal_angle_control(0, 0, 0, 20);
        }

    void AttitudeControlDemoMobileCallback(DJIDrone *drone)
    {
        /* attitude control sample*/
        drone->takeoff();
        sleep(8);


        for(int i = 0; i < 100; i ++)
        {
            if(i < 90)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, -2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, -2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, 0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, -0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, 90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, -90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        drone->landing();

    }
    void LocalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void GlobalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void WaypointNavigationTestMobileCallback(DJIDrone *drone)
    {
        
    }
    void VirtuaRCTestMobileCallback(DJIDrone *drone)
    {
        //virtual RC test data
        uint32_t virtual_rc_data[16];
        //virtual rc test 1: arm & disarm
        drone->virtual_rc_enable();
        usleep(20000);

        virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for (int i = 0; i < 100; i++){
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }

        //virtual rc test 2: yaw 
        drone->virtual_rc_enable();
        virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for(int i = 0; i < 100; i++) {
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }
        drone->virtual_rc_disable();
    }
