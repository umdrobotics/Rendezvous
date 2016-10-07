#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gimbal_control/PidController.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <sstream>
#include <iostream>

#define YAW_LIMIT_DU 3100  // +- ~315 degrees

using namespace std;

geometry_msgs::PointStamped g_msgDesiredGimbalPoseDU = geometry_msgs::PointStamped();




void RunInitialAngleTests(DJIDrone& drone)
{

    // set yaw to 1800 DU (= 180 degrees counterclockwise) with  the time to take 1 sec.
    drone.gimbal_angle_control(0.0, 0.0, 1800.00, 10.0);    
    ros::Duration(2.0).sleep();

    // set all angles to zero.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(2.0).sleep();

    // set roll to -300 DU = -30 degrees with the time to take 1 sec.
    drone.gimbal_angle_control(-300, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set roll to 300 DU = 30 degrees with the time to take 1 sec.
    drone.gimbal_angle_control(300, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set all angles to zero.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set yaw to -1800 DU (= -180 degrees clockwise) with  the time to take 1 sec.
    drone.gimbal_angle_control(0.0, 0.0, -1800.0, 10.0);     
    ros::Duration(1.0).sleep();

    // set all angles to zero.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set pitch to -450 DU (-90 degreed pitch down)with the time to take 1 sec.
    drone.gimbal_angle_control(0.0, -450.0, 0.0, 10.0);    
    ros::Duration(2.0).sleep();

    // set pitch to -900 DU (-90 degreed pitch down)with the time to take 1 sec.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(2.0).sleep();

}

void listernerCallback(const geometry_msgs::PointStamped::ConstPtr& msgDesiredPoseDU)
{
    g_msgDesiredGimbalPoseDU = *msgDesiredPoseDU;
    cout << "Received Desired Roll:" << g_msgDesiredGimbalPoseDU.point.x 
                            << " Pitch:" <<  g_msgDesiredGimbalPoseDU.point.y 
                            << " Yaw:" <<  g_msgDesiredGimbalPoseDU.point.z << endl;
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

    // We need to read the PID parameters from launch file or parameter server.
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

    // We need to read the PID parameters from launch file or parameter server.
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

    // We need to read the PID parameters from launch file or parameter server.
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
    

    g_msgDesiredGimbalPoseDU.point.x = 0.0;
    g_msgDesiredGimbalPoseDU.point.y = -450.0;
    g_msgDesiredGimbalPoseDU.point.z = 0.0;
 


    // Gimbal Angle Tests
    RunInitialAngleTests(drone);


    ros::Rate loop_rate(50);
    
    
    while (ros::ok())
    {
        ros::spinOnce();
                
        double dMeasuredTimeSec = g_msgDesiredGimbalPoseDU.header.stamp.nsec/1000000000.0
                                + g_msgDesiredGimbalPoseDU.header.stamp.sec;
                                
        double yawRateInput = 
            yaw_rate_controller.GetPlantInput ( g_msgDesiredGimbalPoseDU.point.z, 
                                                dMeasuredTimeSec,
                                                drone.gimbal.yaw);
        
        double pitchRateInput = 
            pitch_rate_controller.GetPlantInput (g_msgDesiredGimbalPoseDU.point.y, 
                                                 dMeasuredTimeSec,
                                                 drone.gimbal.pitch);
        
        double rollRateInput = 
            roll_rate_controller.GetPlantInput (g_msgDesiredGimbalPoseDU.point.x, 
                                                dMeasuredTimeSec,    
                                                drone.gimbal.roll);
           
        
        drone.gimbal_speed_control(rollRateInput, pitchRateInput, yawRateInput);
        
        loop_rate.sleep();
    }
       
    
    return 0;    

}
