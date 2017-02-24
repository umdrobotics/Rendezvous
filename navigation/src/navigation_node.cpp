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

//using namespace std;


#define TARGET_LOST_TIME_OUT_SEC (3.0)


DJIDrone* _ptrDrone;

int _targetLocked = 0;
int _nNavigationTask = 0;
bool _bIsDroneLandingPrinted = false;

// Target tracking boolean flags
bool _bIsTargetTrackingRunning = false;
bool _bIsTargetBeingTracked = false;
bool _bIsTargetLost = false;

geometry_msgs::PointStamped _msgTargetLocalPosition;
sensor_msgs::LaserScan _msgUltraSonic;
geometry_msgs::Point _toTargetDistance;


ros::Publisher _GimbalAnglePub;
ros::Publisher _TargetLocalPositionPub;




void ShutDown(void)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate navigation ...");
    delete(_ptrDrone);
      
    ROS_INFO("Shutting down navigation ...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();

}

void SigintHandler(int sig)
{
    ShutDown();
}

void ultrasonic_callback(const sensor_msgs::LaserScan& msgUltraSonic)
{    
    _msgUltraSonic.header.frame_id = msgUltraSonic.header.frame_id;
    _msgUltraSonic.header.stamp.sec = msgUltraSonic.header.stamp.sec;
    _msgUltraSonic.ranges[0] = msgUltraSonic.ranges[0];
    _msgUltraSonic.intensities[0] = msgUltraSonic.intensities[0];

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


    ROS_INFO("------------Initializing searching for target-----------");
    float flyingRadius = 1;
    float limitRadius = 10; 
    float circleRadiusIncrements = 2.0;
    float droneAltitude = 3;
    float Phi = 0;
    ROS_INFO("Initial Radius = %f m, Increment = %f m, Max Radius = %f m", 
                flyingRadius, circleRadiusIncrements, limitRadius);
    ROS_INFO("Initial Height = %f m", droneAltitude);

    ROS_INFO("Local Position: %f, %f\n", drone.local_position.x, drone.local_position.y);
    float x_center = drone.local_position.x;
    float y_center = drone.local_position.y;


    float gimbalYawIncrements = 1;
    geometry_msgs::PointStamped desiredGimbalPoseDeg;

    desiredGimbalPoseDeg.point.x = 0.0;  // roll
    desiredGimbalPoseDeg.point.y = -45.0;  // pitch
    desiredGimbalPoseDeg.point.z = 0.0;   // yaw 
    ROS_INFO("Initial Gimbal Angle: roll = %f deg, pitch = %f deg, yaw = %f deg.", 
                desiredGimbalPoseDeg.point.x, 
                desiredGimbalPoseDeg.point.y, 
                desiredGimbalPoseDeg.point.z);
    


    ROS_INFO("---------------------Start searching---------------------");
    ros::spinOnce();
    while(flyingRadius < limitRadius)
    {
        if(1 == _targetLocked){
                break;
            }

        for(int i = 0; i < 1890; i ++)
        {   
            if(1 == _targetLocked){
                break;
            }

            //set up drone task
            float x =  x_center + flyingRadius*cos((Phi/300));
            float y =  y_center + flyingRadius*sin((Phi/300));
            Phi = Phi+1;
            drone.local_position_control(x, y, droneAltitude, 0);
            
            //set up gimbal task
            //if yaw is greater than or equal to 30deg or less than or equal to 30deg. 
            if(desiredGimbalPoseDeg.point.z > 30.0 || desiredGimbalPoseDeg.point.z < -30.0) 
            { 
                gimbalYawIncrements = -gimbalYawIncrements;         //gimbal swing back
            }
            
            desiredGimbalPoseDeg.point.z += gimbalYawIncrements;
            _GimbalAnglePub.publish(desiredGimbalPoseDeg);
            
            
            usleep(20000);
            ros::spinOnce();

        } 
        
        flyingRadius += circleRadiusIncrements; 
        
    }
    
    if(flyingRadius < 20 && 1 == _targetLocked)
        ROS_INFO("Target FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    else if(flyingRadius > 20)
        ROS_INFO("Didn't find anything! Try to change searching range or search again. ");


}

// Use this function to test anything you want
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

	ros::spinOnce();
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

    // float x_start = drone.local_position.x ;
    // float y_start = drone.local_position.y ;
    float z_start = drone.local_position.z ;
    // float delta_x = _toTargetDistance.x; 
    // float delta_y = _toTargetDistance.y;
    // float x_target =  x_start + delta_x;
    // float y_target =  y_start + delta_y;
    float x_target =  _msgTargetLocalPosition.point.x;
    float y_target =  _msgTargetLocalPosition.point.y;
    float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    ROS_INFO("Distance_square = %f m ", distance_square);

    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;

    if(distance_square > limitRadius_square)
    {
    	ROS_INFO("The drone is approaching!!!!!!!!!!!!!!!!!!!!!!");
    	drone.local_position_control(x_target, y_target, z_start, 0);
	    ros::Duration(0.02).sleep();
    }
    else
    {
    	ROS_INFO("The drone is landing!!!!!!!!!!!!!!!!!!!!!!!!!!");
		drone.local_position_control(x_target, y_target, 0.0, 0);
		ros::Duration(0.02).sleep();
    }

}

void VelocityControlTest(void)
{
	DJIDrone& drone = *_ptrDrone;

	ros::spinOnce();
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

    // float x_start = drone.local_position.x ;
    // float y_start = drone.local_position.y ;
    float z_start = drone.local_position.z ;
    // float delta_x = _toTargetDistance.x; 
    // float delta_y = _toTargetDistance.y;
    // float x_target =  x_start + delta_x;
    // float y_target =  y_start + delta_y;
    float x_target =  _msgTargetLocalPosition.point.x;
    float y_target =  _msgTargetLocalPosition.point.y;
    float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    ROS_INFO("Distance_square = %f m ", distance_square);

    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;

    if(distance_square > limitRadius_square)
    {
    	ROS_INFO("The drone is approaching!!!!!!!!!!!!!!!!!!!!!!");
    	drone.local_position_control(x_target, y_target, z_start, 0);
    	drone.velocity_control(1, 1, 1, 1, 0);
	    ros::Duration(0.02).sleep();
    }
    else
    {
    	ROS_INFO("The drone is landing!!!!!!!!!!!!!!!!!!!!!!!!!!");
		drone.local_position_control(x_target, y_target, 0.0, 0);
		drone.velocity_control(1, 1, 1, 1, 0);
		ros::Duration(0.02).sleep();
    }
}

void WaypointControlTest(void)
{

}


void Waypoint_mission_upload(void)
{
    DJIDrone& drone = *_ptrDrone;

	ros::spinOnce();
	ROS_INFO("To Target Distance:  North  = %f m\n", _toTargetDistance.x);
    ROS_INFO("                     East   = %f m\n", _toTargetDistance.y);
    ROS_INFO("                     Height = %f m\n", _toTargetDistance.z);

    float x_start = drone.local_position.x ;
    float y_start = drone.local_position.y ;
    float delta_x = _toTargetDistance.x; 
    float delta_y = _toTargetDistance.y;
    
    float x_target =  x_start + delta_x;
    float y_target =  y_start + delta_y; 
    float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    ROS_INFO("X_taregt = %f m, Y_target = %f m, distance_square = %f m ", x_target, y_target, distance_square);
     

    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;
    while(distance_square > limitRadius_square)
    {
    	ros::spinOnce();
    	float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    	ROS_INFO("Distance_square = %f m, Height = %f m ", distance_square, drone.global_position.height);

    	drone.local_position_control(x_target, x_target, drone.local_position.z, 0);
	    ros::Duration(0.02).sleep();

    }

    ROS_INFO("The drone is ready to descending!!!!!!!!!!!!!!!!!!!!!!");


    while(1) 
    { 
        ros::spinOnce();

        ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
        ROS_INFO("Local Position: %f, %f\n", drone.local_position.x, drone.local_position.y);
        ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f\n", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 ); 
		ROS_INFO("To Target Distance:  North  = %f m", _toTargetDistance.x);
    	ROS_INFO("                     East   = %f m", _toTargetDistance.y);
    	ROS_INFO("                     Height = %f m\n", _toTargetDistance.z);   


        if (_msgUltraSonic.ranges[0] < 0.1 && (int)_msgUltraSonic.intensities[0] == 1)
        {
            break;
        }    

        drone.local_position_control(x_target, x_target, 0.0, 0);
	    ros::Duration(0.02).sleep();
    }

    ROS_INFO("The drone is ready to land!!!!!!!!!!!!!!!!!!!!!!!!!!");
}


void TemporaryTest(void)
{


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
    // and drone.local_position.y means easting, 
    // we want to be consistent on x-y coordinates.
    output.point.x = targetOffsetFromUAV[0][0];
    output.point.y = targetOffsetFromUAV[1][0];
    output.point.z = targetOffsetFromUAV[2][0];
    
    return output;
    
    //outputDistance[0][0] = targetOffsetFromUAV[0][0];
    //outputDistance[1][0] = targetOffsetFromUAV[1][0];
    //outputDistance[2][0] = targetOffsetFromUAV[2][0];
   
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
    geometry_msgs::PointStamped targetoffset = GetTargetOffsetFromUAV(tag.pose.pose.position, drone.gimbal);

    _msgTargetLocalPosition.header.stamp = ros::Time::now();
    // drone.local_position.x means northing
    // drone.local_position.y means easting
    _msgTargetLocalPosition.point.x = drone.local_position.x + targetoffset.point.x;
    _msgTargetLocalPosition.point.y = drone.local_position.y + targetoffset.point.y;    
    _msgTargetLocalPosition.point.z = 0;

    _TargetLocalPositionPub.publish(_msgTargetLocalPosition);


    _toTargetDistance.x = targetoffset.point.x;
	_toTargetDistance.y = targetoffset.point.y;
	_toTargetDistance.z = targetoffset.point.z;

    _targetLocked = 1;


    //Create message
    // geometry_msgs::PointStamped msgToTargetDistance;
    // msgToTargetDistance.header.stamp = ros::Time::now();
    // msgToTargetDistance.point.x = targetoffset.point.x;
    // msgToTargetDistance.point.y = targetoffset.point.y;	
    // msgToTargetDistance.point.z = drone.global_position.height;

    //_ToTargetDistancePub.publish(msgToTargetDistance);   
      
    std::stringstream ss ;
    
    ss  << std::fixed << std::setprecision(7) << std::endl
        << "Time: " << ros::Time::now().toSec() << std::endl
        << "Tag Distance(x,y,z): "  << x << ","
                                    << y << ","
                                    << z << "," << std::endl
        << "To Target Distance(Northing,Easting,Height): " 	<< targetoffset.point.x << ","
                                    						<< targetoffset.point.y << ","
                                    						<< targetoffset.point.z << "," << std::endl                                
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
            break;
         
        case 22: 
            break;

        case 23: 
            break;

        case 25: 
            LandingTest();
            break;

        case 26: 
            ApproachLandingTest();
            break;

        case 27: 
            VelocityControlTest();
            break;

        case 28: 
            WaypointControlTest();
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
            ROS_INFO_STREAM("Draw Circle Sample - Not implemented.");
            break;
         
        case 22: 
            ROS_INFO_STREAM("Waypoint Mission Upload - Not implemented.");
            break;

        case 23: 
            ROS_INFO_STREAM("Search for Targetted - Not implemented.");
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
            ROS_INFO_STREAM("Waypoint Control Test.");
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
	_msgUltraSonic.ranges.resize(1);
	_msgUltraSonic.intensities.resize(1);
    
        
    // Subscribers    
	int numMessagesToBuffer = 10;
    ros::Subscriber sub1 = nh.subscribe("/navigation_menu/navigation_task", numMessagesToBuffer, navigationTaskCallback);
    ros::Subscriber sub2 = nh.subscribe("/guidance/ultrasonic", numMessagesToBuffer, ultrasonic_callback);
    ros::Subscriber sub3 = nh.subscribe("/usb_cam/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    // ros::Subscriber sub4 = nh.subscribe("/target_tracking/to_target_distance", numMessagesToBuffer, targetDistanceCallback);
    
    
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








