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

extern ros::Publisher gimbal_pose_pub1;
geometry_msgs::PointStamped desiredGimbalPoseDeg;

extern geometry_msgs::Point _droneUtmPosition;
extern geometry_msgs::Point _targetGpsPosition;
extern geometry_msgs::Point _targetUtmPosition;

extern int targetLocked;
extern float realtimeHeight;



int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");

    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);


    navigationHandle = Navigation(nh);

    int frequency = 40;
    navigationHandle.run(frequency)

    ROS_INFO_STREAM("Navigation has started.");

    ros::spin();
    
    return 0;
}


    
    
    
    
    
    
    
    

//~ #ifndef EKF_DEBUG
    
//~ #endif
    

    // Log files
    
    
  
    // Ultrasonic
    _msgUltraSonic.ranges.resize(1);
    _msgUltraSonic.intensities.resize(1);

//~ 
    // Subscribers
    int numMessagesToBuffer = 10;
    ros::Subscriber sub1 = nh.subscribe("/navigation_menu/navigation_task", numMessagesToBuffer, navigationTaskCallback);
    ros::Subscriber sub2 = nh.subscribe("/guidance/ultrasonic", numMessagesToBuffer, ultrasonic_callback);
    ros::Subscriber sub3 = nh.subscribe("/usb_cam/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    // ros::Subscriber sub4 = nh.subscribe("/LQR_K", numMessagesToBuffer, lqrGainCallback);
    ros::Subscriber sub5 = nh.subscribe("/truck/location_GPS", numMessagesToBuffer, truckPositionCallback);
    //~ ros::Subscriber sub6 = nh.subscribe("/truck/real_location_GPS", numMessagesToBuffer, realTruckPositionCallback);
    ros::Subscriber sub7 = nh.subscribe("/truck/velocity", numMessagesToBuffer, truckVelocityCallback);
    ros::Subscriber sub8 = nh.subscribe("/truck/start_simulation", numMessagesToBuffer, startSimCallback);
    // ros::Subscriber sub4 = nh.subscribe("/dji_sdk/gimbal", numMessagesToBuffer, gimbalCallback);


    // Publishers
    _GimbalAnglePub                 = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 10);
    _TargetLocalPositionPub         = nh.advertise<geometry_msgs::PointStamped>("/navigation/target_local_position", 10);
    _TruckLocalPositionPub          = nh.advertise<geometry_msgs::PointStamped>("/navigation/truck_local_position", 10);
    _FusedTargetLocalPositionPub    = nh.advertise<geometry_msgs::PointStamped>("/navigation/fused_target_local_position", 10);

    // main control loop = 20 Hz
    double dTimeStepSec = 0.025;
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);


    

    ros::spin();

    return 0;

}











