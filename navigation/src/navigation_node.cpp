#include "ros/ros.h"
#include <dji_sdk/dji_drone.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

//using namespace std;


DJIDrone* _ptrDrone;

int targetLocked = 0;
float realtimeHeight = 0;
ros::Publisher gimbal_pose_pub1;

geometry_msgs::Point _droneUtmPosition;
geometry_msgs::Point _targetGpsPosition;
geometry_msgs::Point _targetUtmPosition;


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


void listernerCallback(const geometry_msgs::PointStamped::ConstPtr& msgDesiredPoseDeg)
{
    //_msgDesiredGimbalPoseDeg = *msgDesiredPoseDeg;
    //ROS_INFO_STREAM("Desired Angle (Deg) Roll:" << _msgDesiredGimbalPoseDeg.point.x 
    //                        << " Pitch:" << _msgDesiredGimbalPoseDeg.point.y 
    //                        << " Yaw:" <<  _msgDesiredGimbalPoseDeg.point.z);
}

void droneUtmCallback(const geometry_msgs::PointStamped::ConstPtr& msgDroneUtmPos)
{
	if (0 < sizeof(msgDroneUtmPos)) 
    {	
    	_droneUtmPosition.x = msgDroneUtmPos->point.x;
		_droneUtmPosition.y = msgDroneUtmPos->point.y;
		_droneUtmPosition.z = msgDroneUtmPos->point.z;
	}

	// ROS_INFO_STREAM("Drone UTM Position: X = " << msgDroneUtmPos->point.x 
 //                           << " Y = " << msgDroneUtmPos.point.y 
 //                           << " Z = " << msgDroneUtmPos.point.z);

}

void targetGpsCallback(const geometry_msgs::PointStamped::ConstPtr& msgTargetGpsPos)
{
	if (0 < sizeof(msgTargetGpsPos)) 
    {
		_targetGpsPosition.x = msgTargetGpsPos->point.x;
		_targetGpsPosition.y = msgTargetGpsPos->point.y;
		_targetGpsPosition.z = msgTargetGpsPos->point.z;
		
		targetLocked = 1;
		
	}

	// ROS_INFO_STREAM("Target GPS Position: X = " << msgTargetGpsPos->point.x 
 //                           << " Y = " << msgTargetGpsPos.point.y 
 //                           << " Z = " <<  msgTargetGpsPos.point.z);
		
}

void targetUtmCallback(const geometry_msgs::PointStamped::ConstPtr& msgTargetUtmPos)
{
    if (0 < sizeof(msgTargetUtmPos))
    {	
    	_targetUtmPosition.x = msgTargetUtmPos->point.x;
		_targetUtmPosition.y = msgTargetUtmPos->point.y;
		_targetUtmPosition.z = msgTargetUtmPos->point.z;
	}

	// ROS_INFO_STREAM("Target UTM Position: X = " << msgTargetUtmPos->point.x 
 //                           << " Y = " << msgTargetUtmPos.point.y 
 //                           << " Z = " <<  msgTargetUtmPos.point.z);

}

void guidanceCallback(sensor_msgs::LaserScan msgUltraSonicDown)
{
    if (0 < sizeof(msgUltraSonicDown)) 
        realtimeHeight = msgUltraSonicDown.ranges[0];
    ROS_INFO("Real time Height = %f m", realtimeHeight);
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
    ROS_INFO("Initial Radius = %f m, Increment = %f m, Max Radius = %f m", flyingRadius, circleRadiusIncrements, limitRadius);
    ROS_INFO("Initial Height = %f m", droneAltitude);

    ROS_INFO("Local Position: %f, %f\n", drone.local_position.x, drone.local_position.y);
    float x_center = drone.local_position.x;
    float y_center = drone.local_position.y;


    float gimbalYawIncrements = 1;
    geometry_msgs::PointStamped desiredGimbalPoseDeg;

    desiredGimbalPoseDeg.point.x = 0.0;  // roll
    desiredGimbalPoseDeg.point.y = -45.0;  // pitch
    desiredGimbalPoseDeg.point.z = 0.0;   // yaw 
    ROS_INFO("Initial Gimbal Angle: roll = %f deg, pitch = %f deg, yaw = %f deg.", desiredGimbalPoseDeg.point.x, desiredGimbalPoseDeg.point.y, desiredGimbalPoseDeg.point.z);
    


    ROS_INFO("---------------------Start searching---------------------");
    while(flyingRadius < limitRadius && 0 == targetLocked)
    {
        for(int i = 0; i < 1890 && 0 == targetLocked; i ++)
        {   
            //set up drone task
            float x =  x_center + flyingRadius*cos((Phi/300));
            float y =  y_center + flyingRadius*sin((Phi/300));
            Phi = Phi+1;
            drone.local_position_control(x, y, droneAltitude, 0);
            
            //set up gimbal task
            if(desiredGimbalPoseDeg.point.z > 30.0 || desiredGimbalPoseDeg.point.z < -30.0) //if yaw is greater than or equal to 30deg or less than or equal to 30deg. 
                gimbalYawIncrements = -gimbalYawIncrements;         //gimbal swing back
            desiredGimbalPoseDeg.point.z += gimbalYawIncrements;
            gimbal_pose_pub1.publish(desiredGimbalPoseDeg);
            
            
            usleep(20000);

        } 
        
        flyingRadius += circleRadiusIncrements; 
        
    }
    
    if(flyingRadius < 20 && 0 == targetLocked)
        ROS_INFO("Target FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    else if(flyingRadius > 20)
        ROS_INFO("Didn't find anything! Try to change searching range or search again. ");


}


void DrawCircleExample(void)
{

    std::cout << "Enter the radius of the circle in meteres (4m < x < 10m)\n";
    int desiredRadius;
    std::cin >> desiredRadius;   

    std::cout << "Enter height in meteres (Relative to take off point. 1m < x < 5m) \n";
    int desiredAltitude;
    std::cin >> desiredAltitude;  
    
    int flyingRadius = std::max(4, std::min(10, desiredRadius));
    ROS_INFO("The flying radius is %d meters\n", flyingRadius);
    
    int droneAltitude = std::max(1, std::min(5, desiredAltitude));
    ROS_INFO("The drone altitude is %d meters\n", droneAltitude);
                                    
    
    DJIDrone& drone = *_ptrDrone;
    
    ROS_INFO("Local Position: %f, %f\n", drone.local_position.x, drone.local_position.y);

    float x_center = drone.local_position.x;
    float y_center = drone.local_position.y;
                
    float circleRadiusIncrements = 0.01;
            
    for(int j = 0; j < 1000; j ++)
    {   
        if (circleRadiusIncrements < flyingRadius)
        {
            float x =  x_center + circleRadiusIncrements;
            float y =  y_center;
            circleRadiusIncrements = circleRadiusIncrements + 0.01;
            drone.local_position_control(x, y, droneAltitude, 0);
            usleep(20000);
        }
        else
        {
            break;
        }
    }
    
    int Phi = 0;
    
    for(int i = 0; i < 1000; i ++)
    {   
        float x =  x_center + flyingRadius*cos((Phi/120));
        float y =  y_center + flyingRadius*sin((Phi/120));
        Phi = Phi+1;
        drone.local_position_control(x, y, droneAltitude, 0);
        usleep(50000);
           
        ROS_INFO("Local Position: %f, %f\n", drone.local_position.x, drone.local_position.y);
        ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f\n", 
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );
                         
    } 
}


void NavigationTest(void)
{
    DJIDrone& drone = *_ptrDrone;


    float x_start = drone.local_position.x ;
    float y_start = drone.local_position.y ;

    float x =  x_start;
    float y =  y_start + 5.0;
    
    while(realtimeHeight > 0.1 ) 
    {   
        ROS_INFO("Real time Height = %f m", realtimeHeight);
        drone.local_position_control(x, y, -0.01, 0);
        usleep(20000);
    }

    ROS_INFO("The drone is ready to land!");
}


void Waypoint_mission_upload(void)
{
    DJIDrone& drone = *_ptrDrone;

//    ROS_INFO("Drone UTM Position:  X = " << _droneUtmPosition.x << "\n"
//          << "                     Y = " << _droneUtmPosition.y << "\n"
//          << "                     Z = " << _droneUtmPosition.z << "\n");
//    ROS_INFO("Target UTM Position: X = " << _targetUtmPosition.x << "\n"
//          << "                     Y = " << _targetUtmPosition.y << "\n"
//          << "                     Z = " << _targetUtmPosition.z << "\n");


    float xm = drone.local_position.x ;
    float ym = drone.local_position.y ;
    float zm = 3;
    float delta_xm = _targetUtmPosition.x - _droneUtmPosition.x; 
    float delta_ym = _targetUtmPosition.y - _droneUtmPosition.y;
    float delta_zm = _targetUtmPosition.z - _droneUtmPosition.z;
    float distance;
                

    xm += delta_xm; 
    ym += delta_ym;
    distance = abs(delta_xm) + abs(delta_ym); 

    while(distance > 0.5) 
    { 
        drone.local_position_control(xm, ym, zm, 0);
        usleep(20000);

        distance = abs(_targetUtmPosition.x - _droneUtmPosition.x) + abs(_targetUtmPosition.y - _droneUtmPosition.y);
    }


}


void DisplayMainMenu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
    printf("| [1]  Request Control          | [21] Set Msg Frequency Test      |\n");   
    printf("| [2]  Release Control          | [22] Waypoint Mission Upload     |\n");   
    printf("| [3]  Arm the Drone            | [23]                             |\n");   
    printf("| [4]  Disarm the Drone         | [24] Followme Mission Upload     |\n");   
    printf("| [5]  Takeoff                  | [25] Mission Start               |\n");   
    printf("| [6]  Landing                  | [26] Mission Pause               |\n");   
    printf("| [7]  Go Home                  | [27] Mission Resume              |\n");   
    printf("| [8]  Draw Circle Sample       | [28] Mission Cancel              |\n");   
    printf("| [9]  Search for Target        | [29] Mission Waypoint Download   |\n");   
    printf("| [10] Local Navigation Test    | [30] Mission Waypoint Set Speed  |\n");   
    printf("| [11] Global Navigation Test   | [31] Mission Waypoint Get Speed  |\n");    
    printf("| [12] Waypoint Navigation Test | [32] Mission Followme Set Target |\n");   
    printf("|                                    \n");
    printf("| [54] Geolocalization/Gimbal tests and AprilTag recognition)   |\n");
    printf("|                                    \n");
    printf("| [99] Exit                           \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a number then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}



void RunNavigation(void)
{
    int inputValue;
    
    DJIDrone& drone = *_ptrDrone;
    
    bool bIsInputValid = false;
    bool bIsExitRequested = false;
    
    
    while(1)
    {
        ros::spinOnce();
        
        if (bIsExitRequested)
        {
            break;
        }

        DisplayMainMenu();
                    
        if (!bIsInputValid)
        {
            bIsInputValid = true;
        }
        
    
        std::cout << "Enter Input Value: ";
        std::cin >> inputValue;
         
        switch (inputValue)
        {
            case 1: // request control 
                drone.request_sdk_permission_control();
                break;
                
            case 2: // release control 
                drone.release_sdk_permission_control();
                break;
                    
            case 3: // arm 
                drone.drone_arm();
                break;

            case 4: // disarm
                drone.drone_disarm();
                break;
                
            case 5: // take off 
                drone.takeoff();
                break;
                
            case 6: // landing
                drone.landing();
                break;
                                
            case 7: // go home
                drone.gohome();
                break;
                
            case 8: // draw circle sample
                DrawCircleExample();
                break;
                      
            case 9: // search for target
                SearchForTarget();
                break;

            case 10: // Navigation test
                NavigationTest();
                break;
                
            case 22: // Waypoint Mission Upload
                Waypoint_mission_upload();
                break;
                     
            case 25: // Mission Start
                drone.mission_start();
                break;
                
            case 26: //mission pause
                drone.mission_pause();
                break;
                
            case 27: //mission resume
                drone.mission_resume();
                break;
                
            case 28: //mission cancel
                drone.mission_cancel();
                break;
                
            case 99: // Exit the program 
                bIsExitRequested = true;
                std::cout << "Shutting Down.\n";
                ShutDown();
                break;
                
            default: // It will take care of invalid inputs 
                std::cout << "Undefined input value.";
                bIsInputValid = false;
                break;
        }
        
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");
	
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);


    _ptrDrone = new DJIDrone(nh);



    

    //A publisher to control the gimbal angle. 
    gimbal_pose_pub1 = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 1000);
        
	int numMessagesToBuffer = 10;
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/drone_utm_position", numMessagesToBuffer, droneUtmCallback);
	ros::Subscriber sub2 = nh.subscribe("/dji_sdk/target_gps_position", numMessagesToBuffer, targetGpsCallback);
	ros::Subscriber sub3 = nh.subscribe("/dji_sdk/target_utm_position", numMessagesToBuffer, targetUtmCallback);
    ros::Subscriber guidance_sub = nh.subscribe("/guidance/ultrasonic", numMessagesToBuffer, guidanceCallback);
   
    RunNavigation();

    ros::spin();
             
    return 0;    

}
