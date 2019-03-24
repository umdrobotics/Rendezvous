#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <signal.h>

void ShutDown(void)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate navigation ...");

    ROS_INFO("Shutting down navigation ...") ;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();

}

void SigintHandler(int sig)
{
    ShutDown();
}

void DisplayMainMenu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ---------------------------+\n");
    printf("| [1]  Request Control            | [21] Autonomous Landing          |\n");   
    printf("| [2]  Release Control            | [22] Autonomous Landing Two      |\n");   
    printf("| [3]  Arm the Drone              | [23] Autonomous Landing Three    |\n");   
    printf("| [4]  Disarm the Drone           | [24] Search for Target           |\n");   
    printf("| [5]  Takeoff                    | [25] Go to Truck GPS location    |\n");   
    printf("| [6]  Landing                    | [26]                             |\n");   
    printf("| [7]  Go Home                    | [27]                             |\n");   
    printf("| [8]  Target Tracking Start      | [28]                             |\n");   
    printf("| [9]  Target Tracking Cancel     | [29]                             |\n");   
    printf("| [10] Mission Pause              | [30] Run Attitude Control Test   |\n");   
    printf("| [11] Mission Resume             | [31] Ultrasonic Test             |\n");   
    printf("| [12] Mission Waypoint Download  | [32] Landing Test                |\n");   
    printf("| [13] Mission Waypoint Set Speed | [33] Landing Test Plus           |\n");   
    printf("| [14] Followme Mission Upload    | [34] Temporary Test              |\n");
    printf("|                                 |                                  |\n");
    printf("| [98] Stop Current Task          |                                  |\n");
    printf("| [99] Exit                       | [0] Display this menu            |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("input a number then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}
  

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_menu");
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);

    //A publisher to control the gimbal angle. 
    ros::Publisher NavigationMenuPub = nh.advertise<std_msgs::UInt16>("/navigation_menu/navigation_task", 10);
        
    int inputValue;
    
    bool bIsInputValid = true;
    bool bIsExitRequested = false;
    
    std_msgs::UInt16 msgNavigationTask;

    while(1)
    {
        ros::spinOnce();
                    
        if (bIsInputValid)
        {
            DisplayMainMenu();
        }
        else 
        {
            std::cout << "Undefined input value." << std::endl;
            bIsInputValid = true;
        }
            
        std::cout << "Enter Input Value: ";
        std::cin >> inputValue;    

        if (std::cin.fail()) 
        {
            std::cin.clear();
            std::cin.ignore(256,'\n');
            bIsInputValid = false;
            continue;
        }

        if (0 > inputValue || 99 < inputValue) 
        {
            bIsInputValid = false;
            continue;
        }

        if (0 == inputValue)  // Do nothing but display menu again.
        {
            continue;
        }

        if (99 == inputValue)
        {
            bIsExitRequested = true;
            std::cout << "Shutting Down.\n";
            ShutDown();
            break;
        }
 
        msgNavigationTask.data = inputValue;

        // Publish navigation task if inputValue > 0 and < 99
        NavigationMenuPub.publish(msgNavigationTask);
        
    }

             
    return 0;    

}
