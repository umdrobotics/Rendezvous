#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gimbal_control/PidController.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <sstream>
#include <iostream>

using namespace std;

geometry_msgs::PointStamped g_msgDesiredGimbalPose;

void listernerCallback(const geometry_msgs::PointStamped::ConstPtr& desiredPoseMessage)
{
    g_msgDesiredGimbalPose = *desiredPoseMessage;
    cout << "I heard: " << g_msgDesiredGimbalPose.point.x << endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("desired_gimbal_pose", 1000, listernerCallback);
    //ros::spin();

    DJIDrone * pDrone = new DJIDrone(nh);
    DJIDrone & drone = *pDrone;
    
    
    double yawrate_kp = 0.0;
    double yawrate_kd = 0.0;
    double yawrate_ki = 0.0;
    nh.getParam("/YawRatePidCtrlParams/kp", yawrate_kp);
    nh.getParam("/YawRatePidCtrlParams/kd", yawrate_kd);
    nh.getParam("/YawRatePidCtrlParams/ki", yawrate_ki);  
          
    PidController yaw_rate_controller = PidController ( "YawRateCtrl", 
                                                        yawrate_kp, 
                                                        yawrate_kd, 
                                                        yawrate_ki);
        
    double pitchrate_kp = 0.0;
    double pitchrate_kd = 0.0;
    double pitchrate_ki = 0.0;
    nh.getParam("/PitchRatePidCtrlParams/kp", pitchrate_kp);
    nh.getParam("/PitchRatePidCtrlParams/kd", pitchrate_kd);
    nh.getParam("/PitchRatePidCtrlParams/ki", pitchrate_ki);     
       
    PidController pitch_rate_controller = PidController ("PitchRateCtrl", 
                                                        pitchrate_kp, 
                                                        pitchrate_kd, 
                                                        pitchrate_ki);   
    
    double rollrate_kp = 0.0;
    double rollrate_kd = 0.0;
    double rollrate_ki = 0.0;
    nh.getParam("/RollRatePidCtrlParams/kp", rollrate_kp);
    nh.getParam("/RollRatePidCtrlParams/kd", rollrate_kd);
    nh.getParam("/RollRatePidCtrlParams/ki", rollrate_ki);  
    
    PidController roll_rate_controller = PidController ("RollRateCtrl", 
                                                        rollrate_kp, 
                                                        rollrate_kd, 
                                                        rollrate_ki);
    
    cout << yaw_rate_controller;
    cout << pitch_rate_controller;
    cout << roll_rate_controller;
    
    ros::Rate loop_rate(2);
    
    int count = 0;
    
    while (ros::ok())
    {
        ros::spinOnce();
                
        double dMeasuredTimeSec = g_msgDesiredGimbalPose.header.stamp.nsec/1000000000.0
                                + g_msgDesiredGimbalPose.header.stamp.sec;
                                
        double yawRateInput = 
            yaw_rate_controller.GetPlantInput ( g_msgDesiredGimbalPose.point.z, 
                                                dMeasuredTimeSec,
                                                drone.gimbal.yaw);
        
        double pitchRateInput = 
            pitch_rate_controller.GetPlantInput (g_msgDesiredGimbalPose.point.y, 
                                                 dMeasuredTimeSec,
                                                 drone.gimbal.pitch);
        
        double rollRateInput = 
            roll_rate_controller.GetPlantInput (g_msgDesiredGimbalPose.point.x, 
                                                dMeasuredTimeSec,    
                                                drone.gimbal.roll);
           
        
        drone.gimbal_speed_control(rollRateInput, pitchRateInput, yawRateInput);
        
        loop_rate.sleep();
    }
       
    
    return 0;    

}

