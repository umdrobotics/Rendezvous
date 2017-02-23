#include "ros/ros.h"
#include <dji_sdk/dji_drone.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

//using namespace std;

DJIDrone* _ptrDrone;

int _targetLocked = 0;
int _nNavigationTask = 0;
bool _bIsDroneLandingPrinted = false;

sensor_msgs::LaserScan _msgUltraSonic;
geometry_msgs::Point _toTargetDistance;
ros::Publisher _gimbal_pose_pub1;


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
            _gimbal_pose_pub1.publish(desiredGimbalPoseDeg);
            
            
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
    ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
    ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 ); 
	ROS_INFO("To Target Distance:  North  = %f m, East   = %f m, Height = %f m.\n", 
					_toTargetDistance.x, 
					_toTargetDistance.y, 
					_toTargetDistance.z);

    float x_start = drone.local_position.x ;
    float y_start = drone.local_position.y ;
    float z_start = drone.local_position.z ;
    float delta_x = _toTargetDistance.x; 
    float delta_y = _toTargetDistance.y;
    float x_target =  x_start + delta_x;
    float y_target =  y_start + delta_y;
    float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    ROS_INFO("X_taregt = %f m, Y_target = %f m, distance_square = %f m ", x_target, y_target, distance_square);

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
    ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
    ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 ); 
	ROS_INFO("To Target Distance:  North  = %f m, East   = %f m, Height = %f m.\n", 
					_toTargetDistance.x, 
					_toTargetDistance.y, 
					_toTargetDistance.z);

    float x_start = drone.local_position.x ;
    float y_start = drone.local_position.y ;
    float z_start = drone.local_position.z ;
    float delta_x = _toTargetDistance.x; 
    float delta_y = _toTargetDistance.y;
    float x_target =  x_start + delta_x;
    float y_target =  y_start + delta_y;
    float distance_square = _toTargetDistance.x*_toTargetDistance.x + _toTargetDistance.y*_toTargetDistance.y;
    ROS_INFO("X_taregt = %f m, Y_target = %f m, distance_square = %f m ", x_target, y_target, distance_square);

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

    /*draw square sample
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
                */

}


void timerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;


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
                 
        case 8: // Mission Start
            drone.mission_start();
            ROS_INFO_STREAM("Mission start.");     
            break;

        case 9: //mission cancel
            drone.mission_cancel();
            ROS_INFO_STREAM("Mission Cancel.");
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
    

    //A publisher to control the gimbal angle. 
    _gimbal_pose_pub1 = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 1000);
        
    // Subscriber    
	int numMessagesToBuffer = 10;
    ros::Subscriber sub1 = nh.subscribe("/navigation_menu/navigation_task", numMessagesToBuffer, navigationTaskCallback);
    ros::Subscriber sub2 = nh.subscribe("/guidance/ultrasonic", numMessagesToBuffer, ultrasonic_callback);
    ros::Subscriber sub3 = nh.subscribe("/target_tracking/to_target_distance", numMessagesToBuffer, targetDistanceCallback);
    // ros::Subscriber sub2 = nh.subscribe("/target_tracking/drone_utm_position", numMessagesToBuffer, droneUtmCallback);
	// ros::Subscriber sub3 = nh.subscribe("/target_tracking/target_gps_position", numMessagesToBuffer, targetGpsCallback);
	// ros::Subscriber sub4 = nh.subscribe("/target_tracking/target_utm_position", numMessagesToBuffer, targetUtmCallback);
   
    // main control loop = 50 Hz
    double dTimeStepSec = 0.02;
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);
    
    ros::spin();
             
    return 0;    

}
