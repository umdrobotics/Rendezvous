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

#define STATIONARY_TARGET true //if true, it will not try to land based on the quadcopter prediction, but on the actual location 

#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include <dji_sdk_demo/kalmanWebsite.cpp>
//#include <dji_sdk_demo/kalmanPhysics.cpp>  //need to figure out how to put this in the main folder instead of include
               volatile int degs =0; // for debugging gimbal control
             volatile int GIMBAL_TEST_SIGN = 1; //for debugging gimbal control

//#include <dji_sdk_demo/PIDcontrol.cpp> //need to figure out how to put this in the main folder instead of include
#include <dji_sdk_demo/GimbalCalculations.h>
#include <dji_sdk_demo/handleLanding.h>

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

// ALTITUDE_AT_LAST_TARGET_SIGHTING and LAST_RECORDED_HEIGHT_ABOVE_TARGET are used for landing
double ALTITUDE_AT_LAST_TARGET_SIGHTING = 0;
double LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS = 0;

cv::KalmanFilter GLOBAL_KALMAN_FILTER; 
cv::KalmanFilter GLOBAL_KALMAN_FILTER_DIST; //for debugging, let's just try to track the apriltag distance 


//these are for sending the desired angle to the PID controller node
double GLOBAL_ROLL_DJI_UNITS =0.0;
double GLOBAL_PITCH_DJI_UNITS = -450.0; //at the start we want it pointing 45 degrees down to better detect targets
double GLOBAL_YAW_DJI_UNITS =0.0;

cv::Mat STATE_STORAGE; //Need to keep track of each state so we can get an estimate if we lose track of the target. 

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

void ezExit() //shuts the program down quickly for testing purposes
{
#include <assert.h>
assert(1==0);
}


//If we lose track of the target, we need to stop flying to the waypoints that were given based on that target. The best way to do this is to override the waypoint like this
void stayInCurrentSpot(DJIDrone* drone)
{
dji_sdk::GlobalPosition copterState = drone->global_position;
dji_sdk::Waypoint currentSpot; 
currentSpot.latitude = copterState.latitude;
currentSpot.longitude = copterState.longitude;
currentSpot.altitude = copterState.altitude;
currentSpot.heading = 0; //I think this means the aircraft flies straight at it
currentSpot.staytime = 0; //don't wish to linger there

dji_sdk::WaypointList loneWaypoint; //will only be 1 element in this list
loneWaypoint.waypoint_list.push_back(currentSpot);
drone->waypoint_navigation_send_request(loneWaypoint);
}

//appears to work best if this is a global variable

void goToTargetEstimate(DJIDrone* drone, double latitude, double longitude, double altitude)
{ 
const float desiredWaypointSpeed = 0.1f ; // 10.0 had the copter go too fast and overshoot 10.0f;

//this is the old way that works, even if it's crude. Keep it in case something goes wrong with the new way
//In fact, it appears that only this works for on-the-fly changes
dji_sdk::Waypoint targetEstimate; 
targetEstimate.latitude = latitude;
targetEstimate.longitude = longitude;
targetEstimate.altitude = altitude;
targetEstimate.heading = 0; //I think this means the aircraft flies straight at it
targetEstimate.staytime = 1.0; //don't wish to linger there, but we can afford a 1 second wait on a stationary target


dji_sdk::WaypointList loneWaypoint; //will only be 1 element in this list
loneWaypoint.waypoint_list.push_back(targetEstimate);
drone->waypoint_navigation_send_request(loneWaypoint);
      

}


ros::NodeHandle* nh;
DJIDrone* drone;
#define AprilTagsTopicTracking "dji_sdk/tag_detections"
void handleGimbalPrediction(UTMobject predictedTargetUTM, UTMobject predictedCopterUTM, double heightAboveTarget, std_msgs::Header latestHeader ,DJIDrone* drone)
{

 //then need to modify the gimbal angle to have it point appropriately. This will be done with multiple variables that will be modified by a function, rather than an explicit return
   double yaw_rads;
   double pitch_rads;
   double roll_rads;
    getGimbalAngleToPointAtTarget_rads
    (
 	predictedCopterUTM, 
	heightAboveTarget, 
	predictedTargetUTM
 	,yaw_rads //This is an output variable
	,pitch_rads //This is an output variable
	,roll_rads    //This is an output variable
    );
	
/*#define NO_PITCH false // set this to true to avoid pitch during testing on the bench
#ifdef NO_PITCH
pitch_rads = degreesToRadians(-10); //small pitch so calculations still make sense
#endif */
if (YAW_RELATIVE_TO_BODY == true)
  {yaw_rads = inertialFrameToBody_yaw(yaw_rads, drone);}


   unsigned char desiredControlMode = 1; //lets you use DJI go gimbal mode selection, and thus free mode.
   unsigned char desiredDuration = 10; //actually it looks like 10 is the lowest it will go//1; //this is the durtion in tenths of a second. I think 1 is the lowest it can go. TODO verify this 

    //drone->gimbal_angle_control(radiansToDjiUnits(roll_rads), radiansToDjiUnits(pitch_rads), radiansToDjiUnits(yaw_rads), desiredDuration, desiredControlMode);
     //skip the angle control and let a PID control (outside this function loop) handle it
	 GLOBAL_ROLL_DJI_UNITS = radiansToDjiUnits(roll_rads); 
	 GLOBAL_PITCH_DJI_UNITS = radiansToDjiUnits(pitch_rads);
	 GLOBAL_YAW_DJI_UNITS = radiansToDjiUnits(yaw_rads);
 //then  we're done with this step
     
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

////////Turned this piece into a function so it could also easily be called when there is not a target detection, only a prediction
void handleTargetPrediction(cv::Mat targetLocPrediction ,std::string targetUtmZone ,dji_sdk::GlobalPosition copterState ,std_msgs::Header latestHeader ,DJIDrone* drone ,bool shouldIDescend ){
cout <<"targetLocPrediction " <<targetLocPrediction <<" "; 

 double predictedNorth = targetLocPrediction.at<double>(0,0); //access element 0,0 ie x
 double predictedEast = targetLocPrediction.at<double>(0,1); //access element 0,1 ie y
cout <<" east north zone " << predictedEast<<" "<< predictedNorth<<" "<< targetUtmZone;

UTMobject predictedTargetUTM; //will need this for later
std::get<northingIndex>(predictedTargetUTM) = predictedNorth;
std::get<eastingIndex>(predictedTargetUTM) = predictedEast;
std::get<designatorIndex>(predictedTargetUTM) = targetUtmZone;
  

UTMobject actualCopterUTM = GPStoUTM(copterState.latitude, copterState.longitude);  
//flying to these may allow more control over the copter, since it often mvoes too fast, especialyl for a stationary target
  double halfwayNorth = (predictedNorth + std::get<northingIndex>(actualCopterUTM))/2.0;
  double halfwayEast = (predictedEast + std::get<eastingIndex>(actualCopterUTM))/2.0;


  std::pair<double, double> targetLocPredictionGPS = UTMtoGPS(halfwayNorth, halfwayEast, targetUtmZone); //predictedNorth, predictedEast, targetUtmZone); 

//now we need to descend towards the target (or maybe land)


																			
  UTMobject predictedCopterUTM = GPStoUTM(copterState.latitude, copterState.longitude); 
  //Now we need to add velocity to it. But we can't simply assume it will carry on it's current velocity vector.
   //For simplicity, let's assume the same magnitude, but in the direction of the new vector
    //since predictedCopterUTM variable currently contains the actual copter position, not prediction, we can use this
   dji_sdk::Velocity copterSpeed = drone->velocity; //DJI::onboardSDK::VelocityData copterSpeed = DJI::onboardSDK::Flight::getVelocity(); 
   double newCopterAngle_rads = atan2(predictedEast - std::get<eastingIndex>(predictedCopterUTM), predictedNorth - std::get<northingIndex>(predictedCopterUTM));		   
   double velocityMagnitude = sqrt(copterSpeed.vx * copterSpeed.vx + copterSpeed.vy * copterSpeed.vy);
   
   double predictedChangeInCopterEast = sin(newCopterAngle_rads) * LATEST_DT * velocityMagnitude;
   double predictedChangeInCopterNorth = cos(newCopterAngle_rads) * LATEST_DT * velocityMagnitude;
   if(STATIONARY_TARGET != true) //it must be explicitly defined as true for us to skip this (and thus assume a stationary target)
    {
       std::get<northingIndex>(predictedCopterUTM) += predictedChangeInCopterNorth;
       std::get<eastingIndex>(predictedCopterUTM) += predictedChangeInCopterEast;
       
    }
   else
     {
       cout << "TARGET ASSUMED TO BE STATIONARY!"; 
     }
   

   //now need to handle if we should descend to the target or not
   bool shouldILand = false; 
   double desiredNewAltitude_meters = HandleLanding::getNewAltitudeForDescent(
																			   predictedNorth - std::get<northingIndex>(predictedCopterUTM)
																			   ,predictedEast - std::get<eastingIndex>(predictedCopterUTM)
																			   ,ALTITUDE_AT_LAST_TARGET_SIGHTING
																			   ,LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS
																			   
																			   ,shouldILand
																			 ) ;
	if(shouldIDescend != true)
			{desiredNewAltitude_meters = copterState.altitude; }
		
		
   
   
  // then need to set quadcopter waypoint accordingly
  goToTargetEstimate(drone ,targetLocPredictionGPS.latitudeIndex ,targetLocPredictionGPS.longitudeIndex  , desiredNewAltitude_meters); 
  // then need to to estimate where the quadcopter will actually be


  if(shouldILand == true)
   {
	
	// THIS WILL ONLY WORK FOR A STATIONARY TARGET!
   cout <<" \n\n NOTICE!!!! \n ABOUT TO LAND  !!!! \n\n";
	drone->landing();  
       cout <<" \n\n NOTICE!!!! \n SENT LANDING COMMAND  !!!! \n\n";
    //if it won't land when it should, uncomment the following 2 liens
    //while(drone.global_position.height > 0.5) //need to prevent the rest of the loop from overriding this
	//            {ros::Duration(1.0).sleep();}
   }

   double estimatedTargetHeight = copterState.altitude - copterState.height ; //still assumes target is on ground, but should avoid the timing errors associated with using ALTITUDE_AT_LAST_TARGET_SIGHTING
   printf(" target height estimate %f  altitude at last sighting %f last recorded height above target %f \n", estimatedTargetHeight,  ALTITUDE_AT_LAST_TARGET_SIGHTING ,LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS );
   if(ALTITUDE_AT_LAST_TARGET_SIGHTING ==0) //if it wasn't modified after initializing
            {estimatedTargetHeight = 0;}
   double predictedHeightAboveTarget = desiredNewAltitude_meters - estimatedTargetHeight; 
   handleGimbalPrediction(predictedTargetUTM, predictedCopterUTM, predictedHeightAboveTarget, latestHeader, drone);

}
//end handleTargetPrediction





void apriltagCheckCallbackForTracking(const apriltags_ros::AprilTagDetectionArray /*sensor_msgs::ImageConstPtr&*/ tag_detection_array)
{

	
    UTMobject latestTargetLocation; 
    //must declare before the if statements so it can be used for further estimates 
    // (by recording the UTM designator zone) if the target is lost	
	
    std_msgs::Header latestHeader; 
    //declare outside the if statements so it can be used at the end	

    //sensor_msgs::Image rgb = *tag_detection_array; //have to use this to access the data, can't use the ConstPtr for that

    apriltags_ros::AprilTagDetectionArray aprilTagsMessage;
    aprilTagsMessage = tag_detection_array;

    std::vector<apriltags_ros::AprilTagDetection> found;
    found = aprilTagsMessage.detections;

    int numTags = found.size();
  
    cv::Mat targetLocPrediction; 
    // must declare outside the if statements so we can keep estimating if the target is lost
    // If we use a local variable and then assign a global variable to it, 
    // this is less likely to cause memory overflow that just using the global variable directly

    // let's assume that if there are multiple tags, we only want to deal with the first one.
    if (numTags > 0 ) //TODO : correct flaw in logic here, such that if we lose the target we still perform he calculations based on estimates 
    {
	    bool firstDetection = true; //assume it's the first detection (and thus that we may need to disregard the information) until proven otherwise
	    FRAMES_WITHOUT_TARGET = 0; //we've found the target again
  
        apriltags_ros::AprilTagDetection current=found.at(0);
        current.pose.pose.position.y = CAMERA_Y_MULTIPLIER * current.pose.pose.position.y;  
        //since we want to ensure up, relative to the camera, is treated as positive y in the camera frame  
  
        latestHeader = current.pose.header; 

        //update the global variables containing the coordinates (in the camera frame remember) 
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
        dji_sdk::Gimbal gimbalState = drone->gimbal;
        //DJI::onboardSDK::GimbalData gimbalState = (drone->gimbal).getGimbal();
        
        dji_sdk::GlobalPosition copterState = drone->global_position; 
        //DJI::onboardSDK::PositionData copterState = drone->global_position; //DJI::onboardSDK::Flight::getPosition();
	 
	    //need to keep track of the altitude and height above target for when we land
        ALTITUDE_AT_LAST_TARGET_SIGHTING = copterState.altitude;
        
        // Note that gimbal result is in degrees, but gimbal control is in tenths of a dgree
        latestTargetLocation = targetDistanceMetersToUTM_WithHeightDifference ( current.pose.pose.position,
                                                                                degreesToRadians(gimbalState.roll), 
			                                                                    degreesToRadians(gimbalState.pitch), 
			                                                                    degreesToRadians(gimbalState.yaw/*bodyFrameToInertial_yaw(gimbalState.yaw,drone)*/),
			                                                                    // since yaw is relative to body, not inertial frame, 
			                                                                    //we need to convertdegreesToRadians(gimbalState.yaw)
			                                                                    copterState.latitude,
			                                                                    copterState.longitude,
			                                                                    copterState.altitude, //TODO decide if we should use .height instead
			                                                                    LAST_RECORDED_HEIGHT_ABOVE_TARGET_METERS // THIS IS AN OUTPUT VARIABLE
			                                                                  );
			                                                                  
        // cout <<" verify result e,n,zone "<< std::get<eastingIndex>(latestTargetLocation) << " "
        //      << std::get<northingIndex>(latestTargetLocation) <<" "<< std::get<designatorIndex>(latestTargetLocation)  <<"\n";

        //recall that x is north, y is east, and we need these values to pass the Kalman filter
        double targetX = std::get<northingIndex>(latestTargetLocation); //LATEST_TARGET_X_CAMERA; //std::get<northingIndex>(latestTargetLocation);
        double targetY = std::get<eastingIndex>(latestTargetLocation); //LATEST_TARGET_Y_CAMERA; //std::get<eastingIndex>(latestTargetLocation);

        if(! ( IS_TRACKING) )
        {
	   
	        HandleLanding::pauseDescent();  

            firstDetection = true;

            GLOBAL_KALMAN_FILTER = initializeKalmanFilterWeb(); 
            targetLocPrediction = loopStepWeb(GLOBAL_KALMAN_FILTER, LATEST_DT, targetX, targetY, !firstDetection); 
            //use !firstDetection since we don't want to descend if this is the first detection
       
            if(STATIONARY_TARGET == true)
            {
                cout <<"NOTICE: STATIONARY TARGET ASSUMED!";
                targetLocPrediction.at<double>(0) = targetX; //use the actual value since we don't need to kalman filter in this case
                targetLocPrediction.at<double>(1) = targetY; //use the actual value since we don't need to kalman filter in this case
                targetLocPrediction.at<double>(2) = 0; //vx = 0 since we're not predicting
                targetLocPrediction.at<double>(3) = 0; //vy = 0 since we're not predicting
            }       

            STATE_STORAGE=targetLocPrediction;    //in case we lose track of the target, this needs to save our data

            // String stall;
            // cout <<"\nCIN pause: press a key then hit enter to contiue\n"; 
            // cin>> stall; 
        }
        else
        {
            firstDetection = false;
            targetLocPrediction =  loopStepWeb(GLOBAL_KALMAN_FILTER, LATEST_DT, targetX, targetY, !firstDetection); 
                //use !firstDetection since we don't want to descent until we have multiple detections
       
            if(STATIONARY_TARGET == true)
            {
                cout <<"NOTICE: STATIONARY TARGET ASSUMED!";
                targetLocPrediction.at<double>(0) = targetX; //use the actual value since we don't need to kalman filter in this case
                targetLocPrediction.at<double>(1) = targetY; //use the actual value since we don't need to kalman filter in this case
                targetLocPrediction.at<double>(2) = 0; //vx = 0 since we're not predicting
                targetLocPrediction.at<double>(3) = 0; //vy = 0 since we're not predicting
            }   
        }
     
	    IS_TRACKING = true;
        handleTargetPrediction( targetLocPrediction, std::get<designatorIndex>(latestTargetLocation), copterState , latestHeader , drone, !firstDetection);
	 
    } //closing brace to if(numTags>0)
    else // if no detections then we can't track it
    {
     
  	    HandleLanding::pauseDescent();  //I don't really want to land based on a kalman filter's estimate, it's too risky
	 
        //my concern is that by declaring it false every time there isn't one, we might end up never tracking it
        //IS_TRACKING = false; //couldn't find one so we're obviously not tracking yet. 
	    if(IS_TRACKING == true)
	    {

		    FRAMES_WITHOUT_TARGET ++ ; 
	        if(FRAMES_WITHOUT_TARGET >= FRAMES_UNTIL_TARGET_LOST)
            { 
                IS_TRACKING = false; 
                FRAMES_WITHOUT_TARGET = 0;
                stayInCurrentSpot(drone);
                //reset the gimbal also, so it poitns in front and down
                GLOBAL_ROLL_DJI_UNITS =0.0;
                GLOBAL_PITCH_DJI_UNITS =-450.0; 
                GLOBAL_YAW_DJI_UNITS = -0.0;
            }
			else
            {
                targetLocPrediction = loopStepWebWithoutMeasurement(GLOBAL_KALMAN_FILTER, LATEST_DT, STATE_STORAGE);
                //targetEstimateWithoutMeasurement(GLOBAL_KALMAN_FILTER, LATEST_DT);
				
				dji_sdk::GlobalPosition copterState = drone->global_position;
					 
                STATE_STORAGE = targetLocPrediction; //so it can be reused for future predictions if needed                
                if(STATIONARY_TARGET!=true)
				{
				    handleTargetPrediction( targetLocPrediction, 
				                            std::get<designatorIndex>(latestTargetLocation), 
				                            copterState,
				                            latestHeader, 
				                            drone, 
				                            false); //put false since we don't want to land using a kalman filter prediction without data
                 }
                 else
                 {
                    cout <<"NOTICE: STATIONARY TARGET ASSUMED!";
                    stayInCurrentSpot(drone);
                }
            }	
        }
    }
}

#define numMessagesToBuffer 1 //10

void listenOptionForTracking(ros::NodeHandle& n)
{
    ros::Subscriber sub = n.subscribe(AprilTagsTopicTracking, numMessagesToBuffer, apriltagCheckCallbackForTracking);
    printf("\n After the callback line");
    ros::spin(); 
}






//following function is to test for very basic errors in the latest functions. It will be updated as more functions are added

void dummyTest()
{
    dummyTest_geolocalization();
    printf("geolocalization tests done");
    printf("\n beginning gimbal tests \n");
    dummyTest_gimbal();
    printf("\n gimbal tests done"); 

    for (int a =0; a<10; a++)
    {
        printf("running kalman filter test, check CSV file");
    }
    
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
                
        //My code: now what happens if we override it? Does it fly to the new place we want it to?

            ros::Duration(3.0).sleep();
            newWaypointList.waypoint_list.clear();
             waypoint4.altitude= 10; 
             waypoint4.latitude = 22.537018;
             waypoint4.longitude=113.95364;
             newWaypointList.waypoint_list.push_back(waypoint4);
              drone->waypoint_navigation_send_request(newWaypointList);
        //Answer: YES! There's a visible turn after the 3 second sleep is over and it then 
         
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
	//crudeTestWeb(); 




				GLOBAL_ANGLE_PUBLISHER = (*nh).advertise<geometry_msgs::PointStamped>("/dji_sdk/desired_angle", 2); // queue size of 2 seems reasonable
				drone->check_version(); //TODO need to find a way to get the return value from this
				drone->request_sdk_permission_control(); //TODO need to find a way to get the return value from this
				drone->takeoff(); //TODO need to find a way to get the return value from this
                stayInCurrentSpot(drone); //Need this in case there are waypoint navigation requests still active 
				//while(drone->global_position.height< 1.5) //set low in case it doesn't reach the full 2 meters 
				//{
                //    ros::Duration(0.1).sleep(); 
                //} //let it reach the right height
				
                ros::Duration(3.0).sleep(); //3 seconds to stabilize after taking off
				printf ("Starting to listen for AprilTags on %s", AprilTagsTopicTracking );
		//ezExit();
				//listenOption(*nh);
				
                char waitKeyChar; 
				listenOptionForTracking(*nh); 
				waitKeyChar = cv::waitKey(1);
				if (waitKeyChar == 'q' || waitKeyChar == 'Q') //attempt to allow this to be exited by pressing q. Still need to test
				{
                    break;
                }
                 ezExit(); 
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
