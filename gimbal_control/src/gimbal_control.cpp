#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gimbal_control/PidController.h"
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <signal.h>
#include <sstream>
#include <iostream>

#define YAW_LIMIT_DU 3100  // +- ~315 degrees

using namespace std;

geometry_msgs::PointStamped _msgDesiredGimbalPoseDU = geometry_msgs::PointStamped();
DJIDrone* _ptrDrone;
PidController *_roll_rate_controller;
PidController *_pitch_rate_controller;
PidController *_yaw_rate_controller;


void SigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate gimbal_control...");
    delete(_ptrDrone);
    delete(_roll_rate_controller);
    delete(_pitch_rate_controller);
    delete(_yaw_rate_controller);
      
    ROS_INFO("Shutting down gimbal_control...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


void RunInitialAngleTests(DJIDrone& drone)
{
    // set yaw to 1790 DU (= 179 degrees counterclockwise) with  the time to take 1 sec.
    drone.gimbal_angle_control(0.0, 0.0, 900.00, 10.0);    
    ros::Duration(1.0).sleep();

    // set yaw to -1790 DU (= -179 degrees clockwise) with  the time to take 1 sec.  
    drone.gimbal_angle_control(0.0, 0.0, -900.0, 10.0);     
    ros::Duration(1.0).sleep();

    // set all angles to zero.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set yaw to 100 DU (= 10 degrees clockwise) with  the time to take 1 sec.  
    drone.gimbal_angle_control(0.0, 0.0, 200.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set yaw to -100 DU (= -10 degrees clockwise) with  the time to take 1 sec.  
    drone.gimbal_angle_control(0.0, 0.0, -200.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set all angles to zero.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set roll to -300 DU = -15 degrees with the time to take 1 sec.
    drone.gimbal_angle_control(-150, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set roll to 300 DU = 15 degrees with the time to take 1 sec.
    drone.gimbal_angle_control(150, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set all angles to zero.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(1.0).sleep();

    // set pitch to -450 DU (-45 degreed pitch down)with the time to take 1 sec.
    drone.gimbal_angle_control(0.0, -450.0, 0.0, 10.0);    
    ros::Duration(2.0).sleep();

    // set pitch to -900 DU (-90 degreed pitch down)with the time to take 1 sec.
    drone.gimbal_angle_control(0.0, 0.0, 0.0, 10.0);    
    ros::Duration(2.0).sleep();

}


void timerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;
    
    double dMeasuredTimeSec = _msgDesiredGimbalPoseDU.header.stamp.nsec/1000000000.0
                            + _msgDesiredGimbalPoseDU.header.stamp.sec;
                            
    double yawRateInput = 
        _yaw_rate_controller->GetPlantInput(_msgDesiredGimbalPoseDU.point.z, drone.gimbal.yaw);
    
    double pitchRateInput = 
        _pitch_rate_controller->GetPlantInput (_msgDesiredGimbalPoseDU.point.y, drone.gimbal.pitch);
    
    double rollRateInput = 
        _roll_rate_controller->GetPlantInput (_msgDesiredGimbalPoseDU.point.x, drone.gimbal.roll);
           
    drone.gimbal_speed_control(rollRateInput, pitchRateInput, yawRateInput);
}

void listernerCallback(const geometry_msgs::PointStamped::ConstPtr& msgDesiredPoseDU)
{
    _msgDesiredGimbalPoseDU = *msgDesiredPoseDU;
    ROS_INFO_STREAM("Received Desired Roll:" << _msgDesiredGimbalPoseDU.point.x 
                            << " Pitch:" <<  _msgDesiredGimbalPoseDU.point.y 
                            << " Yaw:" <<  _msgDesiredGimbalPoseDU.point.z);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("desired_gimbal_pose", 1000, listernerCallback);

    double dTimeStepSec = 0.02;
    nh.getParam("/gimbal_control/gimbal_control_time_step_sec", dTimeStepSec);   
    ROS_INFO("Controller time step is %f\n", dTimeStepSec);

    _ptrDrone = new DJIDrone(nh);

    double yawrate_kp = 0.0;
    double yawrate_kd = 0.0;
    double yawrate_ki = 0.0;
    // We need to read the PID parameters from launch file or parameter server.
    nh.getParam("/YawRatePidCtrlParams/kp", yawrate_kp);   
    nh.getParam("/YawRatePidCtrlParams/kd", yawrate_kd);
    nh.getParam("/YawRatePidCtrlParams/ki", yawrate_ki);  

    double deadZoneAngleDU = 10.0; // 1 degree
    bool isIntelligentControl = true;

    _yaw_rate_controller = new PidController("YawRateCtrl", 
                                            yawrate_kp, 
                                            yawrate_kd, 
                                            yawrate_ki,
                                            dTimeStepSec,
                                            deadZoneAngleDU,
                                            isIntelligentControl);
        
    double pitchrate_kp = 0.0;
    double pitchrate_kd = 0.0;
    double pitchrate_ki = 0.0;
    // We need to read the PID parameters from launch file or parameter server.
    nh.getParam("/PitchRatePidCtrlParams/kp", pitchrate_kp);
    nh.getParam("/PitchRatePidCtrlParams/kd", pitchrate_kd);
    nh.getParam("/PitchRatePidCtrlParams/ki", pitchrate_ki);     
       
    _pitch_rate_controller = new PidController ("PitchRateCtrl", 
                                                pitchrate_kp, 
                                                pitchrate_kd, 
                                                pitchrate_ki,
                                                dTimeStepSec);   
    
    double rollrate_kp = 0.0;
    double rollrate_kd = 0.0;
    double rollrate_ki = 0.0;
    // We need to read the PID parameters from launch file or parameter server.
    nh.getParam("/RollRatePidCtrlParams/kp", rollrate_kp);
    nh.getParam("/RollRatePidCtrlParams/kd", rollrate_kd);
    nh.getParam("/RollRatePidCtrlParams/ki", rollrate_ki);  
    
    _roll_rate_controller = new PidController ("RollRateCtrl", 
                                                rollrate_kp, 
                                                rollrate_kd, 
                                                rollrate_ki,
                                                dTimeStepSec);
    
    ROS_INFO_STREAM(*_yaw_rate_controller);
    ROS_INFO_STREAM(*_pitch_rate_controller);
    ROS_INFO_STREAM(*_roll_rate_controller);
    
    _msgDesiredGimbalPoseDU.point.x = 0.0;
    _msgDesiredGimbalPoseDU.point.y = -450.0;
    _msgDesiredGimbalPoseDU.point.z = 0.0;
 
    // Gimbal Angle Tests
    DJIDrone& drone = *_ptrDrone;
    
	// RunInitialAngleTests(drone);
    

    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);
    
    signal(SIGINT, SigintHandler);
    
    ros::spin();
             
             
    return 0;    

}
