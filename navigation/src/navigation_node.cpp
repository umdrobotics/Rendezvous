#include "ros/ros.h"
#include <dji_sdk/dji_drone.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <navigation/conversion.h>
#include <navigation/UasMath.h>
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <sstream>
#include <fstream>

using namespace std;

#define DEFAULT_TARGET_TRACKING_LOG_FILE_NAME "/home/ubuntu/TargetTracking_"
#define DEFAULT_AUTONOMOUS_LANDING_LOG_FILE_NAME "/home/ubuntu/AutonomousLanding_"

#define TARGET_LOST_TIME_OUT_SEC (3.0)


DJIDrone* _ptrDrone;

int _nNavigationTask = 0;
bool _bIsDroneLandingPrinted = false;

// Target tracking boolean flags
bool _bIsTargetTrackingRunning = false;
bool _bIsTargetBeingTracked = false;
bool _bIsTargetLost = false;

// Landing and searching flags/variables
float _SearchCenter_x = -1;
float _SearchCenter_y = -1;
float _FlyingRadius = 1;
float _gimbalYawIncrements = 1;
int _SearchCounter = 0;
float _Phi = 0;
int _LoopCounter = 1;
float _SearchAngle_Yaw = 0;
bool _bIsSearchInitiated = false; 

// Stationary Approach and Land test
float _StationaryTargetPos_x = 0;
float _StationaryTargetPos_y = 0;
bool _bIsTestInitiated = false;


std::ofstream _ofsTragetTrackingLog;
std::ofstream _ofsAutonomousLandingLog;


sensor_msgs::LaserScan _msgUltraSonic;
geometry_msgs::Point _toTargetDistance;
geometry_msgs::PointStamped _msgTargetLocalPosition;
geometry_msgs::PointStamped _msgDesiredGimbalPoseDeg;
geometry_msgs::PointStamped _msgTargetDistance;

ros::Publisher _GimbalAnglePub;
ros::Publisher _TargetLocalPositionPub;




void ShutDown(void)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate navigation ...");
    
    delete(_ptrDrone);    
    _ofsTragetTrackingLog.close();
    _ofsAutonomousLandingLog.close();
      
    ROS_INFO("Shutting down navigation ...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();

}

void SigintHandler(int sig)
{
    ShutDown();
}


/**
 *  https://github.com/dji-sdk/Onboard-SDK/blob/3.1/doc/en/ProgrammingGuide.md
 * Example: 
 * double roll_rad, pitch_rad, yaw_rad;
 * quaternionToRPY(drone.attitude_quaternion, roll_rad, pitch_rad, yaw_rad); 
**/
 void quaternionToRPY(dji_sdk::AttitudeQuaternion q, double & roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{ 
     roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
     pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
     yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}

void ultrasonic_callback(const sensor_msgs::LaserScan& msgUltraSonic)
{    
    _msgUltraSonic.header.frame_id = msgUltraSonic.header.frame_id;
    _msgUltraSonic.header.stamp.sec = msgUltraSonic.header.stamp.sec;
    _msgUltraSonic.ranges[0] = msgUltraSonic.ranges[0];
    _msgUltraSonic.intensities[0] = msgUltraSonic.intensities[0];

}


float LocalPositionControlHelper(float desired, float current_position)
{
    
    float error = desired - current_position;
    
    return (error < 0.4) ? desired  
                         : (error < 5) ? current_position + error * 0.35 
                                       : current_position + error * 0.5;
                                       
}


void RunLocalPositionControl(geometry_msgs::Point desired_position, float desired_yaw_deg)
{
    DJIDrone& drone = *_ptrDrone;
        
    float setpoint_x = LocalPositionControlHelper(desired_position.x, drone.local_position.x);
    float setpoint_y = LocalPositionControlHelper(desired_position.y, drone.local_position.y);
    float setpoint_z = LocalPositionControlHelper(desired_position.z, drone.local_position.z);
    
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float current_yaw_deg = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    
    float yaw_error_deg = desired_yaw_deg - current_yaw_deg;
    
    float setpoint_yaw = (yaw_error_deg < 3) ? desired_yaw_deg  
                                             : (yaw_error_deg < 10) ? current_yaw_deg + yaw_error_deg * 0.35 
                                             : current_yaw_deg + yaw_error_deg * 0.5;
    
    drone.local_position_control(setpoint_x, setpoint_y, setpoint_z, setpoint_yaw);
    
}


/*
void targetDistanceCallback(const geometry_msgs::PointStamped::ConstPtr& msgTargetDistance)
{
	if (0 < sizeof(msgTargetDistance)) 
    {
		_toTargetDistance.x = msgTargetDistance->point.x;
		_toTargetDistance.y = msgTargetDistance->point.y;
		_toTargetDistance.z = msgTargetDistance->point.z;
		
		_targetLocked = 1;
		
	}
		
}
*/


void SearchForTarget(void)
{
    DJIDrone& drone = *_ptrDrone;
    ROS_INFO("------------Initializing searching-----------");
    
    //If target has been tracked, then set variables to initial values(in case next time) and return
    if(_bIsTargetBeingTracked){
        
        ROS_INFO("Target FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        _SearchCenter_x = -1;
        _SearchCenter_y = -1;

        return;
    }
    
    
    float initialRadius = 1;
    float limitRadius = 5; 
    float circleRadiusIncrements = 2.0;
    float searchAltitude = 3;
    
    ROS_INFO("Initial Radius = %f m, Increment = %f m, Max Radius = %f m, Initial Height = %f m. ", initialRadius, circleRadiusIncrements, limitRadius, searchAltitude);

    
    bool bIsSearchVariableSetUp = (-1 < _SearchCenter_x) && (-1 < _SearchCenter_y);
    if(!bIsSearchVariableSetUp)
    {
        _SearchCenter_x = drone.local_position.x;
        _SearchCenter_y = drone.local_position.y;
        
        // _bIsSearchInitiated = true;
        
        ROS_INFO("Set up search center: x = %f m, y = %f m.", _SearchCenter_x, _SearchCenter_y);
        
        _msgDesiredGimbalPoseDeg.point.x = 0.0;  // roll
        _msgDesiredGimbalPoseDeg.point.y = -45.0;  // pitch
        _msgDesiredGimbalPoseDeg.point.z = 0.0;   // yaw 
        _gimbalYawIncrements = 0.5;
        ROS_INFO("Set Up Initial Gimbal Angle: roll = %f deg, pitch = %f deg, yaw = %f deg.", 
                _msgDesiredGimbalPoseDeg.point.x, 
                _msgDesiredGimbalPoseDeg.point.y, 
                _msgDesiredGimbalPoseDeg.point.z);
                
        _SearchCounter = 0;
        _Phi = 0;
        _FlyingRadius = initialRadius;
        _LoopCounter = 1;
        _SearchAngle_Yaw = 0;
 
    }


    ROS_INFO("---------------------Searching---------------------");

    bool bIsDroneOutOfRange = _FlyingRadius > limitRadius; 
    if(!bIsDroneOutOfRange)
    {
        if(_SearchCounter < (630*_LoopCounter))
        {   
            ROS_INFO("Now Radius = %f m, loop = %d . center_Yaw = %f. ", _FlyingRadius, _LoopCounter, _SearchAngle_Yaw);
            ROS_INFO("Local Position: %f, %f. ", drone.local_position.x, drone.local_position.y);
            
            //set up drone task
            _Phi = _Phi+1;
            _SearchAngle_Yaw = 90 + _Phi/(1.75*_LoopCounter);
            if (360 < _SearchAngle_Yaw)
            {
                _SearchAngle_Yaw = _SearchAngle_Yaw - 360;
            }
            float x =  _SearchCenter_x + _FlyingRadius*cos((_Phi/(100*_LoopCounter)));
            float y =  _SearchCenter_y + _FlyingRadius*sin((_Phi/(100*_LoopCounter)));
            drone.local_position_control(x, y, searchAltitude, _SearchAngle_Yaw);
            
            
            //set up gimbal task
            //if yaw is greater than or equal to 30deg or less than or equal to 30deg. 
            if(_msgDesiredGimbalPoseDeg.point.z > 30.0 || _msgDesiredGimbalPoseDeg.point.z < -30.0) 
            { 
                _gimbalYawIncrements = -_gimbalYawIncrements;         //gimbal swing back
            }
            _msgDesiredGimbalPoseDeg.point.z += _gimbalYawIncrements;
            _GimbalAnglePub.publish(_msgDesiredGimbalPoseDeg);
            
            
            _SearchCounter++;

        } 
        else
        {
            _SearchCounter = 0;  
            _Phi = 0; 
            _FlyingRadius += circleRadiusIncrements;
            _LoopCounter++;
         
        }
    }
    else 
    {

        _SearchCenter_x = -1;
        _SearchCenter_y = -1;
        _LoopCounter = 1;
        _SearchAngle_Yaw = 0;
        ROS_INFO("Didn't find anything! Try to change searching range or search again. ");
        _nNavigationTask = 98;
    }

}

void UltrasonicTest(void)
{
    ROS_INFO("frame_id: %s, stamp: %d, distance: %f, reliability: %d", 
            _msgUltraSonic.header.frame_id.c_str(), 
            _msgUltraSonic.header.stamp.sec,
            _msgUltraSonic.ranges[0],
            (int)_msgUltraSonic.intensities[0] );
      
}

void LandingTest(void)
{
    DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.1) && (int)_msgUltraSonic.intensities[0];

    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        { 
            ROS_INFO("The drone has landed!");     
            _bIsDroneLandingPrinted = true;
        }
        return;
    }
    else
    {
        ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
        ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
        ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );     
        drone.local_position_control(drone.local_position.x, drone.local_position.y, 0.0, 0);
    }

}

void ApproachLandingTest(void)
{
	DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.1) && (int)_msgUltraSonic.intensities[0];
    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        { 
            ROS_INFO("The drone has landed!");     
            _bIsDroneLandingPrinted = true;
        }
        return;
    }

	ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
	ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );
    ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y); 
	ROS_INFO("Target Local Pos: Northing = %f m, Easting = %f m, Height = %f m.",
					_msgTargetLocalPosition.point.x,
					_msgTargetLocalPosition.point.y,
					_msgTargetLocalPosition.point.z
				 );
    

    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;
    float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    ROS_INFO("Distance_square = %f m, LimitRadius_square = %f m", distance_square, limitRadius_square);

        
    bool bIsReadyToLand = distance_square < limitRadius_square;
    if(!bIsReadyToLand)
    {
        ROS_INFO("The drone is approaching!!!!!!!!!!!!!!!!!!!!!!");
    	drone.local_position_control(_msgTargetLocalPosition.point.x, _msgTargetLocalPosition.point.y, drone.local_position.z, 0);
    }
    else
    {
        ROS_INFO("The drone is landing!!!!!!!!!!!!!!!!!!!!!!!!!!");
		drone.local_position_control(_msgTargetLocalPosition.point.x, _msgTargetLocalPosition.point.y, 0.0, 0);
    }

}

void LandingTestPlus(void)
{
    DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.1) && (int)_msgUltraSonic.intensities[0];

    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        { 
            ROS_INFO("The drone has landed!");     
            _bIsDroneLandingPrinted = true;
        }
        drone.landing();
        return;
    }
    else
    {
        ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
        ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
        ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );     
        drone.local_position_control(drone.local_position.x, drone.local_position.y, 0.0, 0);
    }
}

void KnownStationaryApproachLandingTest(void)
{
	DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.1) && (int)_msgUltraSonic.intensities[0];
    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        { 
            ROS_INFO("The drone has landed!");     
            _bIsDroneLandingPrinted = true;
            _bIsTestInitiated = false;
        }
        _nNavigationTask = 98;
        return;
    }
    
 
    if(!_bIsTestInitiated){
        float x_start = drone.local_position.x ;
        float y_start = drone.local_position.y ;    
        _StationaryTargetPos_x =  x_start + 5.0;
        _StationaryTargetPos_y =  y_start; 
        _bIsTestInitiated = true; 
    }

	ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
	ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );
    ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y); 
	ROS_INFO("Target Local Pos: Northing = %f m, Easting = %f m, Height = %f m.",
					_msgTargetLocalPosition.point.x,
					_msgTargetLocalPosition.point.y,
					_msgTargetLocalPosition.point.z
				 );
    

    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;
    float distance_square = (_StationaryTargetPos_x - drone.local_position.x)*(_StationaryTargetPos_x - drone.local_position.x) 
        + (_StationaryTargetPos_y - drone.local_position.y)*(_StationaryTargetPos_y - drone.local_position.y);
    ROS_INFO("Distance_square = %f m, LimitRadius_square = %f m", distance_square, limitRadius_square);

        
    bool bIsReadyToLand = distance_square < limitRadius_square;
    if(!bIsReadyToLand)
    {
        ROS_INFO("The drone is approaching!!!!!!!!!!!!!!!!!!!!!!");
    	drone.local_position_control(_StationaryTargetPos_x, _StationaryTargetPos_y, drone.local_position.z, 0);
    }
    else
    {
        ROS_INFO("The drone is landing!!!!!!!!!!!!!!!!!!!!!!!!!!");
		drone.local_position_control(_StationaryTargetPos_x, _StationaryTargetPos_y, 0.0, 0);
    }
}





void TemporaryTest(void)
{

    DJIDrone& drone = *_ptrDrone;
    
    geometry_msgs::Point desired_position;
    
    desired_position.x = 5;
    desired_position.y = 3;
    desired_position.z = -0.1;
    
    float yawDeg = (float)UasMath::ConvertRad2Deg(atan2(desired_position.x, desired_position.y));
    
    
    RunLocalPositionControl(desired_position, yawDeg);
    
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
     
    ROS_INFO("%f, %f, %f, %f", drone.local_position.x, drone.local_position.y, drone.local_position.z, yaw); 

}


geometry_msgs::PointStamped GetTargetOffsetFromUAV( geometry_msgs::Point& tagPosition_M,        
                                                    dji_sdk::Gimbal& gimbal)                                                    
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
    
    double targetOffsetFromCamera[3][1] = {{tagPosition_M.x}, {tagPosition_M.y}, {tagPosition_M.z}};
    
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
    
    geometry_msgs::PointStamped output;
    
    output.header.stamp = ros::Time::now();
    
    // Because drone.local_position.x means northing
    // and drone.local_position.y means easting, so is the targetOffsetFromUAV.
    // Careful utm.x means easting, utm.y means northing.
    output.point.x = targetOffsetFromUAV[0][0];
    output.point.y = targetOffsetFromUAV[1][0];
    output.point.z = targetOffsetFromUAV[2][0];
    
    return output;
   
} ///end GetTargetOffsetFromUAV()



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

    // double targetOffsetFromUAV[3][1];
    _msgTargetDistance = GetTargetOffsetFromUAV(tag.pose.pose.position, drone.gimbal);

    _msgTargetLocalPosition.header.stamp = ros::Time::now();
    // drone.local_position.x means northing
    // drone.local_position.y means easting
    _msgTargetLocalPosition.point.x = drone.local_position.x + _msgTargetDistance.point.x;
    _msgTargetLocalPosition.point.y = drone.local_position.y + _msgTargetDistance.point.y;    
    _msgTargetLocalPosition.point.z = 0;

    _TargetLocalPositionPub.publish(_msgTargetLocalPosition);


    _toTargetDistance.x = _msgTargetDistance.point.x;
	_toTargetDistance.y = _msgTargetDistance.point.y;
	_toTargetDistance.z = _msgTargetDistance.point.z;

    _bIsTargetBeingTracked = true;

    /*

    std::stringstream ss ;
    
    ss  << std::fixed << std::setprecision(7) << std::endl
        << "Time: " << ros::Time::now().toSec() << std::endl
        << "Tag Distance(x,y,z): "  << x << ","
                                    << y << ","
                                    << z << "," << std::endl
        << "To Target Distance(Northing,Easting,Height): " 	<< _msgTargetDistance.point.x << ","
                                    						<< _msgTargetDistance.point.y << ","
                                    						<< _msgTargetDistance.point.z << "," << std::endl                                
        << "Gimbal Angle Deg(y,p,r): "  << drone.gimbal.yaw << ","
										<< drone.gimbal.pitch << ","
										<< drone.gimbal.roll << "," << std::endl
        << "Desired Angle Deg(y,p,r): " << msgDesiredAngleDeg.point.z << ","
                                        << msgDesiredAngleDeg.point.y << ","
                                        << msgDesiredAngleDeg.point.x << std::endl
        << "Target Local Pos(time: x,y,z): " << _msgTargetLocalPosition.header.stamp << ","
											 << _msgTargetLocalPosition.point.x << ","
											 << _msgTargetLocalPosition.point.y << ","
											 << _msgTargetLocalPosition.point.z << "," << std::endl;											               
    ROS_INFO("%s", ss.str().c_str());
	*/
    
    
    _ofsTragetTrackingLog << std::setprecision(std::numeric_limits<double>::max_digits10) 
                        << ros::Time::now().toSec() << "," 
                        << x << "," << y << "," << z << ","  // tag distance
                        << _msgTargetDistance.point.x << ","
                        << _msgTargetDistance.point.y << ","
                        << _msgTargetDistance.point.z << "," // target distance
                        << _msgTargetLocalPosition.point.x << ","
						<< _msgTargetLocalPosition.point.y << ","
						<< _msgTargetLocalPosition.point.z << std::endl; // target local position
  
}


void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{
    // If Target Tracking is not running, do nothing.
    if (!_bIsTargetTrackingRunning) { return; }
    
    FindDesiredGimbalAngle(vecTagDetections);
 }

void RunTargetSearch()
{

}


void RunTargetTracking()
{
    // target tracking has not been initiated. Do nothing
    if (!_bIsTargetTrackingRunning) { return; }

    // time since we saw the target last time.
    ros::Duration timeElapsed = ros::Time::now() - _msgTargetLocalPosition.header.stamp;
    
    // if the time elapsed is larger than a predefined time, we have lost the target.
    _bIsTargetLost = (timeElapsed.toSec() - TARGET_LOST_TIME_OUT_SEC > 0);

    if (_bIsTargetLost)
    {
        _bIsTargetBeingTracked = false;
        RunTargetSearch();
        return;
    }

    // Now target tracking is running and the target is being tracked
    // We can predict the next target position
    // TODO: implement target prediction here. (Kalman filter)


}

void RunTimeCriticalTasks()
{
    if (_bIsTargetTrackingRunning)
    {
        RunTargetSearch();
    }    
}


void RunAutonomousLanding()
{

	DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.1) && (int)_msgUltraSonic.intensities[0];
    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        { 
            ROS_INFO("The drone has landed!");     
            _bIsDroneLandingPrinted = true;
            _bIsTestInitiated = false;
        }
        return;
    }
    
    float target_x = _msgTargetLocalPosition.point.x;
    float target_y = _msgTargetLocalPosition.point.y;
    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y; 
    float drone_z = drone.local_position.z; 
        
    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;
    
    float distance_square = _msgTargetDistance.point.x*_msgTargetDistance.point.x  
                          + _msgTargetDistance.point.y*_msgTargetDistance.point.y;
       
    bool bIsReadyToLand = distance_square < limitRadius_square;

    geometry_msgs::Point desired_position;
    
    desired_position.x = _msgTargetLocalPosition.point.x;
    desired_position.y = _msgTargetLocalPosition.point.y;
    desired_position.z = bIsReadyToLand ? -0.1 : drone_z;
            
    float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTargetDistance.point.y, _msgTargetDistance.point.x));
        
    RunLocalPositionControl(desired_position, desired_yaw);
    
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
            
    _ofsAutonomousLandingLog << std::setprecision(std::numeric_limits<double>::max_digits10) 
                            << ros::Time::now().toSec() << "," 
                            << _msgUltraSonic.ranges[0] << ","
                            << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                            << distance_square << ","                   // distance squared
                            << _msgTargetLocalPosition.point.x << "," 
                            << _msgTargetLocalPosition.point.y << ","
                            << _msgTargetLocalPosition.point.z << ","   // target local position
                            << drone.local_position.x << ","
                            << drone.local_position.y << ","
                            << drone.local_position.z << ","
                            << yaw << std::endl;                             // drone local position
            
}


void RunAutonomousLanding2()
{

	DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.1) && (int)_msgUltraSonic.intensities[0];
    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        { 
            ROS_INFO("The drone has landed!");     
            _bIsDroneLandingPrinted = true;
            _bIsTestInitiated = false;
        }
        return;
    }
    
    float delta_x = _msgTargetDistance.point.x/sqrt(_msgTargetDistance.point.x*_msgTargetDistance.point.x  
                          + _msgTargetDistance.point.y*_msgTargetDistance.point.y);
    float delta_y = _msgTargetDistance.point.y/sqrt(_msgTargetDistance.point.x*_msgTargetDistance.point.x  
                          + _msgTargetDistance.point.y*_msgTargetDistance.point.y);                      
    float target_x = _msgTargetLocalPosition.point.x - delta_x;
    float target_y = _msgTargetLocalPosition.point.y - delta_y;
    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y; 
    float drone_z = drone.local_position.z; 
        
    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;
    float distance_square = (target_x - drone_x)*(target_x - drone_x) + (target_y - drone_y)*(target_y - drone_y);
    bool bIsReadyToLand = distance_square < limitRadius_square;


    geometry_msgs::Point desired_position;
    desired_position.x = target_x;
    desired_position.y = target_y;
    desired_position.z = bIsReadyToLand ? -0.1 : drone_z;
    float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTargetDistance.point.y, _msgTargetDistance.point.x));
    RunLocalPositionControl(desired_position, desired_yaw);
    
    
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
            
    _ofsAutonomousLandingLog << std::setprecision(std::numeric_limits<double>::max_digits10) 
                            << ros::Time::now().toSec() << "," 
                            <<  _msgUltraSonic.ranges[0] << ","
                            << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                            << distance_square << ","                   // distance squared
                            << _msgTargetLocalPosition.point.x << "," 
                            << _msgTargetLocalPosition.point.y << ","
                            << _msgTargetLocalPosition.point.z << ","   // target local position
                            << drone.local_position.x << ","
                            << drone.local_position.y << ","
                            << drone.local_position.z << ","
                            << yaw << std::endl;                             // drone local position
            
}



void timerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;

    // we need to run this functioin regardless of the navigation menu.
    RunTimeCriticalTasks();


    if (_nNavigationTask < 21 || _nNavigationTask > 90)
    // we don't take care of these cases in this callback function.
    // They are taken care in navigationTaskCallback.    
    {
        return;
    }

    switch (_nNavigationTask)
    {
        case 21: 
            RunAutonomousLanding();
            break;
         
        case 22: 
            RunAutonomousLanding2();
            break;

        case 23: 
            SearchForTarget();
            break;

        case 25: 
            LandingTest();
            break;

        case 26: 
            ApproachLandingTest();
            break;

        case 27: 
            LandingTestPlus();
            break;

        case 28: 
            KnownStationaryApproachLandingTest();
            break;

        case 31: 
            UltrasonicTest();
            break;

        case 34:
            TemporaryTest(); 
            break;

        default: // It will take care of invalid inputs 
            break;
    }


}

void navigationTaskCallback(const std_msgs::UInt16 msgNavigationTask)
{

    _nNavigationTask = msgNavigationTask.data;
    
    DJIDrone& drone = *_ptrDrone;
         
    switch (_nNavigationTask)
    {
        case 1: // request control 
            drone.request_sdk_permission_control();
            ROS_INFO_STREAM("Request SDK permission.");
            break;
            
        case 2: // release control 
            drone.release_sdk_permission_control();
            ROS_INFO_STREAM("Release SDK permission.");
            break;
                
        case 3: // arm 
            drone.drone_arm();
            ROS_INFO_STREAM("Arm drone.");
            break;

        case 4: // disarm
            drone.drone_disarm();
            ROS_INFO_STREAM("Disarm drone.");
            break;
            
        case 5: // take off 
            drone.takeoff();
            ROS_INFO_STREAM("Take off.");
            break;
        case 6: // landing
            drone.landing();
            ROS_INFO_STREAM("Landing.");            
            break;
                            
        case 7: // go home
            drone.gohome();
            ROS_INFO_STREAM("Go home.");     
            break;
                 
        case 8: // Target Tracking Start
            _bIsTargetTrackingRunning = true;
            ROS_INFO_STREAM("Target Tracking start.");     
            break;

        case 9: //mission cancel
            _bIsTargetTrackingRunning = false;
            ROS_INFO_STREAM("Target Tracking Cancel.");
            break;
                           
        case 10: //mission pause
            drone.mission_pause();
            ROS_INFO_STREAM("Mission Pause.");
            break;
            
        case 11: //mission resume
            drone.mission_resume();
            ROS_INFO_STREAM("Mission Resume.");
            break;

        case 12: //mission resume
            //drone.mission_resume();
            ROS_INFO_STREAM("Mission Waypoint Download - Not implemented.");
            break;

        case 13: //mission resume
            //drone.mission_resume();
            ROS_INFO_STREAM("Mission Waypoint Set Speed - Not implemented.");
            break;

        case 14: //mission resume
            //drone.mission_resume();
            ROS_INFO_STREAM("Followme Mission Upload - Not implemented.");
            break;
         
        case 21: 
            ROS_INFO_STREAM("Autonomous Tracking and Landing.");
            break;
         
        case 22: 
            ROS_INFO_STREAM("Autonomous Tracking and Landing Two.");
            break;

        case 23: 
            ROS_INFO_STREAM("Search for Target. ");
            break;

        case 25: 
            ROS_INFO_STREAM("Landing Test.");
            _bIsDroneLandingPrinted = false;
            break;

        case 26: 
            ROS_INFO_STREAM("Approach & Landing Test. ");
            break;

        case 27: 
            ROS_INFO_STREAM("Velocity Control Test. ");
            break;

        case 28: 
            ROS_INFO_STREAM("Known Stationary Approach & Landing Test");
            break;

        case 31: 
            ROS_INFO_STREAM("Ultrasonic Test.");
            break;

        case 34: 
            ROS_INFO_STREAM("Temporary Test - Not implemented.");
            break;

        default: // It will take care of invalid inputs 
            break;
    }
    
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");
	
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);

    // Initialize global variables
    _ptrDrone = new DJIDrone(nh);
 
 
    // Log files
 
    std::stringstream ss;
    ss << DEFAULT_TARGET_TRACKING_LOG_FILE_NAME << ros::WallTime::now() << ".log";
    _ofsTragetTrackingLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsTragetTrackingLog, "Failed to open file %s", ss.str().c_str());

    _ofsTragetTrackingLog << "#Time,TagDistance(x,y,z),TargetDistance(x,y,z),TargetLocalPosition(x,y,z)" << std::endl;
    
    ss.str("");
    ss << DEFAULT_AUTONOMOUS_LANDING_LOG_FILE_NAME << ros::WallTime::now() << ".log";
    _ofsAutonomousLandingLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsAutonomousLandingLog, "Failed to open file %s", ss.str().c_str());

    _ofsAutonomousLandingLog << "#Time,UltrasonicDistance,UltrasonicReliability,TargetDistance,TargetLocalPosition(x,y,z),DroneLocation(x,y,z,yaw)" << std::endl;


    // Ultrasonic 
	_msgUltraSonic.ranges.resize(1);
	_msgUltraSonic.intensities.resize(1);
        
    // Subscribers    
	int numMessagesToBuffer = 10;
    ros::Subscriber sub1 = nh.subscribe("/navigation_menu/navigation_task", numMessagesToBuffer, navigationTaskCallback);
    ros::Subscriber sub2 = nh.subscribe("/guidance/ultrasonic", numMessagesToBuffer, ultrasonic_callback);
    ros::Subscriber sub3 = nh.subscribe("/usb_cam/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    ros::Subscriber sub4 = nh.subscribe("/dji_sdk/gimbal", numMessagesToBuffer, gimbalCallback);
    
    
    // Publishers
    _GimbalAnglePub = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 10); 
    _TargetLocalPositionPub = nh.advertise<geometry_msgs::PointStamped>("/navigation/target_local_position", 10); 
    
    //_ToTargetDistancePub = nh.advertise<geometry_msgs::PointStamped>("/target_tracking/to_target_distance", 10);

   
    // main control loop = 50 Hz
    double dTimeStepSec = 0.02;
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);
    
    ros::spin();
             
    return 0;    

}








