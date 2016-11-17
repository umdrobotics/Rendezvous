#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <sstream>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "gimbal_control_pitch_tests");
    ros::NodeHandle nh;
    ros::Publisher gimbal_pose_pub 
        = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 1000);
        
    geometry_msgs::PointStamped desiredGimbalPoseDeg;
          
    
    desiredGimbalPoseDeg.point.x = 0.0;  // roll
    desiredGimbalPoseDeg.point.y = 28.0;  // pitch
    desiredGimbalPoseDeg.point.z = 0.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDeg);
    ros::Duration(2.0).sleep();
    
    desiredGimbalPoseDeg.point.x = 0.0;  // roll
    desiredGimbalPoseDeg.point.y = -45.0;  // pitch
    desiredGimbalPoseDeg.point.z = 0.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDeg);
    ros::Duration(2.0).sleep();
    
    desiredGimbalPoseDeg.point.x = 0.0;  // roll
    desiredGimbalPoseDeg.point.y = -90.0;  // pitch
    desiredGimbalPoseDeg.point.z = 0.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDeg);
    ros::Duration(2.0).sleep();
    
    
    desiredGimbalPoseDeg.point.x = 0.0;  // roll
    desiredGimbalPoseDeg.point.y = -15.0;  // pitch
    desiredGimbalPoseDeg.point.z = 0.0;  // yaw
    gimbal_pose_pub.publish(desiredGimbalPoseDeg);
    ros::Duration(2.0).sleep();
    
   
    return 0;    

}

