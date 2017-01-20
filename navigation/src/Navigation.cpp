#include "Navigation/Navigation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <signal.h>
#include <sstream>
#include <iostream>



using namespace std;
// public methods


Navigation::Navigation()
{     
    ros::NodeHandle nh;
    m_ptrDrone = new DJIDrone(nh);

}

Navigation::Navigation(ros::NodeHandle& nh)
{    
    m_ptrDrone = new DJIDrone(nh);
}


Navigation::~Navigation()
{    

}

void Navigation::RunNavigation(void)
{
    int inputValue;
    
    DJIDrone& drone = *m_ptrDrone;
    
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
				
			case 13: // positioning
				Positioning();
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
                std::cout << "Press Ctrl-C to quit.\n";
                break;
                
            default: // It will take care of invalid inputs 
                std::cout << "Undefined input value.";
                bIsInputValid = false;
                break;
        }
        
    }
}



// private
void Navigation::DisplayMainMenu(void)
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
	printf("| [13] Positioning				| 								   |\n");
    printf("|                                    \n");
    printf("| [54] Geolocalization/Gimbal tests and AprilTag recognition)   |\n");
    printf("|                                    \n");
    printf("| [99] Exit                           \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a number then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}


void Navigation::SearchForTarget(void)
{


}


void Navigation::DrawCircleExample(void)
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
                                    
    
    DJIDrone& drone = *m_ptrDrone;
    
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

void Navigation::Positioning(void)
{
	
}

void Navigation::Waypoint_mission_upload(void)
{
    DJIDrone& drone = *m_ptrDrone;
    
    
    dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
	
	
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
    static int altitude = 4.5;
    // Currently hard coded, should be dynamic
    static float orig_lat = 42.3184;      //42.318498726 -83.2265324272
    static float orig_long = -83.2265;
    
	//Waypoint 1
	waypoint.latitude = orig_lat;
	waypoint.longitude = orig_long+=.0001;
	waypoint.altitude = 3.5;
	waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
	waypoint_task.mission_waypoint.push_back(waypoint);
	
	//Waypoint 2
	waypoint.latitude = orig_lat;
	waypoint.longitude = orig_long+=.0002;
	waypoint.altitude = 2.5;
	waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
	waypoint_task.mission_waypoint.push_back(waypoint);
	
	//Waypoint 3
	waypoint.latitude = orig_lat;
	waypoint.longitude = orig_long+=.0003;
	waypoint.altitude = 1.5;
	waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
	waypoint_task.mission_waypoint.push_back(waypoint);
	
	//Waypoint 4
	waypoint.latitude = orig_lat;
	waypoint.longitude = orig_long+=.0004;
	waypoint.altitude = 0;
	waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
	waypoint_task.mission_waypoint.push_back(waypoint);
	
	
	
	/*
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
    /*
    	waypoint_task.mission_waypoint.push_back(waypoint);
    } 
    */   

	
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

	drone.mission_waypoint_upload(waypoint_task);
}





