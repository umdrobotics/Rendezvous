#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <sstream>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "gimbal_control_tests");
    ros::NodeHandle nh;
    ros::Publisher gimbal_pose_pub 
        = nh.advertise<geometry_msgs::PointStamped>("desired_gimbal_pose", 1000);
        
    geometry_msgs::PointStamped desiredGimbalPoseDU;
       
    
    desiredGimbalPoseDU.point.x = 0.0;  // roll
    desiredGimbalPoseDU.point.y = 0.0;  // pitch
    desiredGimbalPoseDU.point.z = 0.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDU);
    ros::Duration(2.0).sleep();
    
    desiredGimbalPoseDU.point.x = 0.0;  // roll
    desiredGimbalPoseDU.point.y = 0.0;  // pitch
    desiredGimbalPoseDU.point.z = 900.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDU);
    ros::Duration(2.0).sleep();
        
    desiredGimbalPoseDU.point.x = 0.0;  // roll
    desiredGimbalPoseDU.point.y = 0.0;  // pitch
    desiredGimbalPoseDU.point.z = 0.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDU);
    ros::Duration(2.0).sleep();
    
   
    return 0;    

}

