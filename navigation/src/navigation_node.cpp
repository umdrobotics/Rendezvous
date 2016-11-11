#include "ros/ros.h"
#include "Navigation/Navigation.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <signal.h>
#include <sstream>
#include <iostream>

using namespace std;

DJIDrone* _ptrDrone;

void SigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate navigation ...");
    // delete(_ptrDrone);
      
    ROS_INFO("Shutting down navigation ...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void RunInitialAngleTests(DJIDrone& drone)
{

}

void timerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;
}

void listernerCallback(const geometry_msgs::PointStamped::ConstPtr& msgDesiredPoseDeg)
{
    //_msgDesiredGimbalPoseDeg = *msgDesiredPoseDeg;
    //ROS_INFO_STREAM("Desired Angle (Deg) Roll:" << _msgDesiredGimbalPoseDeg.point.x 
    //                        << " Pitch:" << _msgDesiredGimbalPoseDeg.point.y 
    //                        << " Yaw:" <<  _msgDesiredGimbalPoseDeg.point.z);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    _ptrDrone = new DJIDrone(nh);

    // Gimbal Angle Tests
    DJIDrone& drone = *_ptrDrone;
        
    // double dTimeStepSec = 0.1;

    //ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);
    
    Navigation navigator(_ptrDrone);
    
    navigator.RunNavigation();
    
    
    
    signal(SIGINT, SigintHandler);
    
    ros::spin();
             
    return 0;    

}
