
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iostream>

#include <time.h>
#include <signal.h>
#include <math.h>
#include <vector>
#include <string>





using namespace std;
// public methods

extern ros::Publisher gimbal_pose_pub1;
geometry_msgs::PointStamped desiredGimbalPoseDeg;

extern geometry_msgs::Point _droneUtmPosition;
extern geometry_msgs::Point _targetGpsPosition;
extern geometry_msgs::Point _targetUtmPosition;

extern int targetLocked;
extern float realtimeHeight;


Navigation::Navigation()
{     
    ros::NodeHandle nh;
    m_ptrDrone = new DJIDrone(nh);
    Initialize();

}

Navigation::Navigation(ros::NodeHandle& nh)
{    
    m_ptrDrone = new DJIDrone(nh);
    Initialize();
}

Navigation::Initialize()
{ 
    m_kf = 
    m_kf.Initialize();
    m_ekf = 
    m_mpcSolver = 

    LoadNodeSettings(nh);

    InitializeLogFiles(mpc_q, mpc_kiPos, mpc_kiVec);


    // Subscribers
    int numMessagesToBuffer = 10;
    ros::Subscriber sub1 = nh.subscribe("/navigation_menu/navigation_task", numMessagesToBuffer, navigationTaskCallback);
    ros::Subscriber sub2 = nh.subscribe("/guidance/ultrasonic", numMessagesToBuffer, ultrasonic_callback);
    ros::Subscriber sub3 = nh.subscribe("/usb_cam/tag_detections", numMessagesToBuffer, tagDetectionCallback);
    ros::Subscriber sub5 = nh.subscribe("/truck/location_GPS", numMessagesToBuffer, truckPositionCallback);
    //~ ros::Subscriber sub6 = nh.subscribe("/truck/real_location_GPS", numMessagesToBuffer, realTruckPositionCallback);
    ros::Subscriber sub7 = nh.subscribe("/truck/velocity", numMessagesToBuffer, truckVelocityCallback);
    ros::Subscriber sub8 = nh.subscribe("/truck/start_simulation", numMessagesToBuffer, startSimCallback);
    // ros::Subscriber sub4 = nh.subscribe("/dji_sdk/gimbal", numMessagesToBuffer, gimbalCallback);


    // Publishers
    m_GimbalAnglePub                 = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 10);
    m_TargetLocalPositionPub         = nh.advertise<geometry_msgs::PointStamped>("/navigation/target_local_position", 10);
    m_TruckLocalPositionPub          = nh.advertise<geometry_msgs::PointStamped>("/navigation/truck_local_position", 10);
    m_FusedTargetLocalPositionPub    = nh.advertise<geometry_msgs::PointStamped>("/navigation/fused_target_local_position", 10);
}


Navigation::~Navigation()
{    

}


void GimbalSearch()
{
    // searching target with gimbal sweep
    if (m_horiDistance < 6 && !_bIsTargetFound)
    {
        float pitch_angle = -45;
        float yaw_angle = YAW_RANGE*sin(((float)_SearchGimbalPhi/(RATIO_GIMBAL*40))*6.28);
        geometry_msgs::PointStamped msgDesiredAngleDeg;
        msgDesiredAngleDeg.point.x = 0;
        msgDesiredAngleDeg.point.y = pitch_angle;
        msgDesiredAngleDeg.point.z = _bIsMovingYaw ? yaw_angle : 0;
        //~ ROS_INFO("Ptich = %f , Yaw: %f.", pitch_angle, yaw_angle);
        _GimbalAnglePub.publish(msgDesiredAngleDeg);
        _SearchGimbalPhi = _SearchGimbalPhi + 1;
        if (_SearchGimbalPhi > RATIO_GIMBAL*40)
        {
          _SearchGimbalPhi = 0;
        }
    }  
}


float AttitudeControlHelper2(geometry_msgs::Point desired_position, float& dpitch, float& droll)
{
    // MPC controller
    DJIDrone& drone = *_ptrDrone;
 
    int P = _mpc.P_;
    int nx = _mpc.nx_;
    float mpc_kiPos = _mpc.kiPos_;
    float mpc_kiVec = _mpc.kiVec_;
  
    // list current state & desired state, and prepare mpc
    Vector4d xk(drone.local_position.x, drone.local_position.y, drone.velocity.vx, drone.velocity.vy);
    Vector4d targetState(desired_position.x, desired_position.y, _msgFusedTargetVelocity.point.x, _msgFusedTargetVelocity.point.y);

    Vector4d stateError = xk - targetState;
    _mpc.SetXpInitialPoint(xk);
    
    
    // predict the target position and velocity here, KF
    // First decide now we use prediction or estimation, based whether there exists observations or not
//~ #ifdef EKF_DEBUG
    VectorXd truckPred;//= Eigen::MatrixXd(P*nx, 1);
    if(_bIsEKFEnable){
        // Fused in Extended Kalman Filter
        int nPred = _kf.nPred_; 
        _ekf.SetPredHorizon(nPred + P);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
    
        if(timeElapsed.toSec() > 0.001){
            _targetEstState = _ekf.PredictWOObservation(timeElapsed.toSec());
        
            //~ _msgFusedTargetPosition.header.stamp = ros::Time::now();
            //~ _msgFusedTargetPosition.point.x = _targetEstState(0);
            //~ _msgFusedTargetPosition.point.y = _targetEstState(1);
            //~ _msgFusedTargetPosition.point.z = 0;
//~ 
            //~ _msgFusedTargetVelocity.point.x = _targetEstState(2)*cos(_targetEstState(3));
            //~ _msgFusedTargetVelocity.point.y = _targetEstState(2)*sin(_targetEstState(3));
            //~ _msgFusedTargetVelocity.point.z = 0;
        }
        //~ _IsGPSUpdated = false;

        truckPred = _ekf.Predict(_targetEstState);
    }
    else{
//~ #else
        // Fused in Kalman Filter
        _kf.SetPredHorizon(_kf.nPred_ + P);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
        if(timeElapsed.toSec() > 0.001){
            // Time elapsed > 0.001 s, update; O.W. no need to update
            _targetEstState = _kf.PredictWOObservation(timeElapsed.toSec());
        }
        // _IsGPSUpdated = false;

        truckPred = _kf.Predict(_targetEstState, _kf.nPred_ + P);
    }
//~ #endif
    VectorXd desiredState = truckPred.tail(P*nx);
    
      

    // Get optimal control input series
    VectorXd um = MatrixXd::Zero(_mpc.M_*_mpc.nu_,1);
    um = RunConstraintedMPC(xk, desiredState);
    Vector2d uk = um.head(_mpc.nu_);
    //~ std::cout << "Um: " << um.transpose() << endl;
    
    
    
    // Initialize integrator
#ifndef FUSE_DEBUG

    float distance_x = drone.local_position.x - _msgTruckLocalPosition.point.x;
    float distance_y = drone.local_position.y - _msgTruckLocalPosition.point.y;
    
#else

    float distance_x = stateError(0);
    float distance_y = stateError(1);
    
#endif

    if (abs(distance_x) < 20)
    {
        _sumPosErrX = IntegratorCalculationPos(stateError(0)*0.55, _lastPosErrX, _sumPosErrX);
        _sumVecErrX = IntegratorCalculationVec(stateError(2), _lastVecErrX, _sumVecErrX);
        _lastPosErrX = stateError(0);
        _lastVecErrX = stateError(2);
        dpitch = _bIsIntegralEnable ? (-uk(0) - mpc_kiPos*_sumPosErrX -mpc_kiVec*_sumVecErrX): -uk(0);
    }
    else
    {
        dpitch = _bIsIntegralEnable ? (-uk(0) - mpc_kiPos*_sumPosErrX -mpc_kiVec*_sumVecErrX): -uk(0);
    }
    
    if (abs(distance_y) < 20)
    {
        _sumPosErrY = IntegratorCalculationPos(stateError(1)*0.55, _lastPosErrY, _sumPosErrY);
        _sumVecErrY = IntegratorCalculationVec(stateError(3), _lastVecErrY, _sumVecErrY);
        _lastPosErrY = stateError(1);
        _lastVecErrY = stateError(3);
        droll = _bIsIntegralEnable? (-uk(1) - mpc_kiPos*_sumPosErrY - mpc_kiVec*_sumVecErrY): -uk(1);
    }
    else
    {
        droll = _bIsIntegralEnable? (-uk(1) - mpc_kiPos*_sumPosErrY - mpc_kiVec*_sumVecErrY): -uk(1);
    }
    

    // Saturate desired pitch and roll angle to -30deg or 30deg
    float maxAngle = 35.0; 
    dpitch = dpitch > maxAngle ? maxAngle
                               : dpitch < -maxAngle ? -maxAngle
                                                    : dpitch;

    droll = droll > maxAngle ? maxAngle
                             : droll < -maxAngle ? -maxAngle
                                                 : droll;
  
    // Print out data
    ROS_INFO("IsOnTruckTop?:%d, _bIsLandingInitiated?:%d", _IsOnTruckTop, _bIsLandingInitiated);
    std::cout << "SumPosX, SumPosY, q, ki:  " << _sumPosErrX << ", " << _sumPosErrY << ", " << _mpc.q_ << ", " << mpc_kiPos << endl;
    std::cout << "truckx,y;targetx,y;fusedx,y:  " << _msgTruckLocalPosition.point.x << ", " << _msgTruckLocalPosition.point.y << ", " 
                << _msgTargetLocalPosition.point.x << ", " << _msgTargetLocalPosition.point.y << ", "
                << _msgFusedTargetPosition.point.x << "," << _msgFusedTargetPosition.point.y << endl;
    std::cout << "dronex,y,z; vx,vy: " << drone.local_position.x << "," << drone.local_position.y << "," << _msgUltraSonic.ranges[0] << "," << drone.velocity.vx << "," << drone.velocity.vy << std::endl;
    ROS_INFO(" error_px, error_py, dpitch, droll: %f, %f, %f, %f, %f, %f", distance_x, distance_y, dpitch, droll, -uk(0), -uk(1));
    
    
    // log data 
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float pitch = (float)UasMath::ConvertRad2Deg( asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1)) );
    float roll = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)) );

    _ofsMPCControllerLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            << ros::Time::now().toSec() << ","
                            <<  _msgUltraSonic.ranges[0] << ","
                            << (int)_msgUltraSonic.intensities[0] << ","   // ultrasonic
                            << _msgTruckLocalPosition.point.x << ","
                            << _msgTruckLocalPosition.point.y << ","
                            << _msgTruckLocalPosition.point.z << "," 
                            << _msgTruckVelocity.point.x << ","
                            << _msgTruckVelocity.point.y << ","       // truck local position
                            << _msgTargetLocalPosition.point.x << ","
                            << _msgTargetLocalPosition.point.y << ","
                            << _msgTargetLocalPosition.point.z << ","     // target local position
                            << _msgFusedTargetPosition.point.x << ","
                            << _msgFusedTargetPosition.point.y << ","
                            << _msgFusedTargetPosition.point.z << ","      // fused target local position
                            << _msgFusedTargetVelocity.point.x << ","
                            << _msgFusedTargetVelocity.point.y << ","
                            << _msgFusedTargetVelocity.point.z << ","      // fused target local position 
                            << _msgRealTruckLocalPosition.point.x << ","
                            << _msgRealTruckLocalPosition.point.y << "," 
                            << _msgRealTruckLocalPosition.point.z << ","                           
                            //~ << _targetEstState.transpose() << ","        // truck estimated position & velocity
                            << drone.local_position.x << ","
                            << drone.local_position.y << ","
                            << drone.local_position.z << ","
                            << drone.velocity.vx << ","
                            << drone.velocity.vy << ","
                            << drone.velocity.vz << ","
                            << dpitch << ","
                            << droll << ","
                            << _sumPosErrX << ","
                            << _sumPosErrY << ","
                            << roll << ","
                            << pitch << ","
                            << yaw << ","
                            << -uk(0) << ","
                            << -uk(1) << ","
                            << std::endl;                            

              
              
}


void RunAttitudeControl(geometry_msgs::Point desired_position, float desired_yaw_deg){

    DJIDrone& drone = *m_ptrDrone;
  
  // compute desired roll and pitch
    float setAnglePitch = 0;
    float setAngleRoll  = 0;
    AttitudeControlHelper2(desired_position, setAnglePitch, setAngleRoll); 
    
    // adjust height
    float setpoint_z = LocalPositionControlAltitudeHelper(desired_position.z, drone.local_position.z);
    
    // compute desired yaw
    float setpoint_yaw = YawControlHelper(desired_yaw_deg);
  
    // Notice: negative pitch angle means going to North; positive pitch means South.
    drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);

    // Just record the system input.
    m_msgDesiredAttitudeDeg.point.x = setAngleRoll;
    m_msgDesiredAttitudeDeg.point.y = -setAnglePitch;
    m_msgDesiredAttitudeDeg.point.z = setpoint_yaw;

}


void GoToTruckGPSLocation()
{
    DJIDrone& drone = *m_ptrDrone;

   
    if( m_bIsTargetFound )
    {
        ROS_INFO("Found target in approaching state!!");
        return;
    }

    // Go to 1m away from the target
    float deltaDistance = sqrt(pow(m_msgTruckDistance.point.x, 2) + pow(m_msgTruckDistance.point.y, 2);
    float delta_x = m_msgTruckDistance.point.x / deltaDistance;
    float delta_y = m_msgTruckDistance.point.y / deltaDistance;


    // Now the destination is 1m away from the target
    float target_x = m_msgTruckLocalPosition.point.x - delta_x;
    float target_y = m_msgTruckLocalPosition.point.y - delta_y;
    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y;
    float drone_z = drone.local_position.z;


    m_horiDistance = sqrt(pow((m_msgTruckLocalPosition.point.x - drone_x), 2) + pow((m_msgTruckLocalPosition.point.y - drone_y), 2));

    m_IsClose = m_horiDistance < 3;
    m_IsOnTruckTop = m_horiDistance < 1; // Hard coded, define if < 0.7 m2, then means on the top

    // Calculate desired postion and yaw; then send commands
    geometry_msgs::Point desired_position;
    desired_position.x = m_IsClose ? m_msgTruckLocalPosition.point.x : target_x;
    desired_position.y = m_IsClose ? m_msgTruckLocalPosition.point.y : target_y;
    desired_position.z = 3;



    float desired_yaw = 0;
    RunAttitudeControl(desired_position, desired_yaw);   
    GimbalSearch();

 
    // Print out data
    //~ ROS_INFO("Desired Local position:%f, %f ,m_horiDistance:%f, Close?:%d.",desired_position.x, desired_position.y, m_horiDistance, bIsClose);
    //~ ROS_INFO("Truck local position: %f, %f, %f.", m_msgTruckLocalPosition.point.x, m_msgTruckLocalPosition.point.y, m_msgTruckLocalPosition.point.z);
    //~ ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
    //~ ROS_INFO("Truck GPS position: lat:%f, lon:%f, alt:%f.", m_msgTruckGPSPosition.point.x, m_msgTruckGPSPosition.point.y, m_msgTruckGPSPosition.point.z);
    //~ ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f",
                    //~ drone.global_position.longitude,
                    //~ drone.global_position.latitude,
                    //~ drone.global_position.altitude,
                    //~ drone.global_position.height
                 //~ );


}


void quaternionToRPY_Radian(dji_sdk::AttitudeQuaternion q, double& roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{
    roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
    pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
    yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}

void quaternionToRPY_Degree(dji_sdk::AttitudeQuaternion q, double& roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{
    yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    pitch = (float)UasMath::ConvertRad2Deg( asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1)) );
    roll = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)) );
}

void RunAutonomousLanding3()
{

    if(!_IsOnTruckTop && !_bIsLandingInitiated)
    {
        GoToTruckGPSLocation();
        return;
    }
    else 
    {
        RunAutonomousLanding2();
        m_bIsLandingInitiated = true;
        return;
    }

}


void TimerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;

    // we need to run this functioin regardless of the navigation menu.
    RunTimeCriticalTasks();


    if (_nNavigationTask < 21 || _nNavigationTask > 90)
    // we don't take care of these cases in this callback function.
    // They are taken care in navigationTaskCallback.
    {
        return;
    }

    switch (_nNavigationTask)
    {
        case 21:
            RunStateMachine();
            break;

        case 22:
            GoToTruckGPSLocation();
            break;

        case 23:
            RunAutonomousLanding();
            break;

        case 24:
            RunTargetSearch();
            break;

        default: // It will take care of invalid inputs
            break;
    }

  //~ std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << ros::Time::now().toSec() << std::endl;
}

void navigationTaskCallback(const std_msgs::UInt16 msgNavigationTask)
{
  
    DJIDrone& drone = *_ptrDrone;

    switch (_nNavigationTask)
    {
        case 1: // request control
            drone.request_sdk_permission_control();
            ROS_INFO_STREAM("Request SDK permission.");
            break;

        case 2: // release control
            drone.release_sdk_permission_control();
            ROS_INFO_STREAM("Release SDK permission.");
            break;

        case 3: // arm
            drone.drone_arm();
            ROS_INFO_STREAM("Arm drone.");
            break;

        case 4: // disarm
            drone.drone_disarm();
            ROS_INFO_STREAM("Disarm drone.");
            break;

        case 5: // take off
            drone.takeoff();
            ROS_INFO_STREAM("Take off.");
            break;
        case 6: // landing
            drone.landing();
            ROS_INFO_STREAM("Landing.");
            break;

        case 7: // go home
            drone.gohome();
            ROS_INFO_STREAM("Go home.");
            break;

        case 8: // Target Tracking Start
            _bIsTargetTrackingRunning = true;
            ROS_INFO_STREAM("Target Tracking start.");
            break;

        case 9: //mission cancel
            _bIsTargetTrackingRunning = false;
            ROS_INFO_STREAM("Target Tracking Cancel.");
            break;

        case 10: //mission pause
            drone.mission_pause();
            ROS_INFO_STREAM("Mission Pause.");
            break;

        case 11: //mission resume
            drone.mission_resume();
            ROS_INFO_STREAM("Mission Resume.");
            break;

        case 12: //mission resume
            //drone.mission_resume();
            ROS_INFO_STREAM("Mission Waypoint Download - Not implemented.");
            break;

        case 13: //mission resume
            //drone.mission_resume();
            ROS_INFO_STREAM("Mission Waypoint Set Speed - Not implemented.");
            break;

        case 14: //mission resume
            //drone.mission_resume();
            ROS_INFO_STREAM("Followme Mission Upload - Not implemented.");
            break;

        case 21:
            ROS_INFO_STREAM("Run state machine.");
            break;

        case 22:
            ROS_INFO_STREAM("Go to target GPS position.");
            break;

        case 23:
            ROS_INFO_STREAM("Autonomous Tracking and Landing. ");
            break;

        case 24:
            ROS_INFO_STREAM("Search for Target. ");
            break;

        case 98:
            ROS_INFO_STREAM("Stopped Current Mission.");
            break;

        default: // It will take care of invalid inputs
            break;
    }

}



Navigation::Run(int frequency)
{    
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/frequency), &Navigation::TimerCallback, this);
    
}















void Navigation::Logging(void)
{

    // Calculate yaw angle
    double roll = 0, pitch = 0, yaw = 0;
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    quaternionToRPY_Degree(q, roll, pitch, yaw)


    // Record data in file
    m_ofsGoToTruckLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                     << ros::Time::now().toSec() << ","
                     <<  m_msgUltraSonic.ranges[0] << ","
                     << (int)m_msgUltraSonic.intensities[0] << "," // ultrasonic
                     << m_horiDistance << ","
                     << m_msgTruckLocalPosition.point.x << ","
                     << m_msgTruckLocalPosition.point.y << ","
                     << m_msgTruckLocalPosition.point.z << ","
                     << m_msgTruckVelocity.point.x << ","
                     << m_msgTruckVelocity.point.y << ","
                     << m_msgFusedTargetPosition.point.x << ","
                     << m_msgFusedTargetPosition.point.y << ","
                     << m_msgFusedTargetPosition.point.z << ","
                     << m_msgFusedTargetVelocity.point.x << ","
                     << m_msgFusedTargetVelocity.point.y << ","
                     << m_msgFusedTargetVelocity.point.z << ","
                     << drone.global_position.latitude << ","
                     << drone.global_position.longitude << ","
                     << drone.local_position.x << ","
                     << drone.local_position.y << ","
                     << drone.local_position.z << ","
                     << drone.velocity.vx << ","
                     << drone.velocity.vy << ","
                     << drone.velocity.vz << ","
                     << roll << ","
                     << pitch << ","
                     << yaw << std::endl;                                // drone local position                           // drone local position
}



