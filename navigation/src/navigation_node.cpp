#include "ros/ros.h"
#include "Navigation/Navigation.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <sstream>
#include <iostream>

// using namespace std;

ros::NodeHandle nh;
Navigation navigator(nh);

ros::Publisher gimbal_pose_pub1;

geometry_msgs::Point _droneUtmPosition;
geometry_msgs::Point _targetGpsPosition;
geometry_msgs::Point _targetUtmPosition;



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
		
		navigator.targetLocked = 1;
		
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");


    signal(SIGINT, SigintHandler);
    
    gimbal_pose_pub1 = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 1000);
        
	
	int numMessagesToBuffer = 2;
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/drone_utm_position", numMessagesToBuffer, droneUtmCallback);
	ros::Subscriber sub2 = nh.subscribe("/dji_sdk/target_gps_position", numMessagesToBuffer, targetGpsCallback);
	ros::Subscriber sub3 = nh.subscribe("/dji_sdk/target_utm_position", numMessagesToBuffer, targetUtmCallback);
   



    navigator.RunNavigation();
        

    
    ros::spin();
             
    return 0;    

}
