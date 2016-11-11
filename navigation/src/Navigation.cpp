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


// Navigation::Navigation()
// {     
// }

Navigation::Navigation(DJIDrone* ptrDrone)
: m_ptrDrone(ptrDrone)                              
{    
    m_drone = &ptrDrone;
}


Navigation::~Navigation()
{    

}

void Navigation::RunNavigation(void)
{
    DisplayMainMenu();
    int inputValue;
    
    while(1)
    {
        ros::spinOnce();
        std::cout << "Enter Input Value: ";
        std::cin >> inputValue;
         
        switch (inputValue)
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
            default:
                std::cout << "Undefined input value.";
                break;
        }
        
    }
}



// private
void Navigation::DisplayMainMenu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23]                             |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Local Navigation Test    | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Global Navigation Test   | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [14] Arm the Drone            | [36]                             |\n");	
	printf("| [15] Disarm the Drone         | [37]                             |\n");
    printf("| [16] Virtual RC Test           \n");
    printf("| [54] Geolocalization/Gimbal tests and AprilTag recognition)   |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a number then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}




