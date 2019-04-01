
#include <signal.h>
#include <math.h>


#include <vector>
#include <stddef.h>
#include <stdlib.h>


#include <sstream>
#include <fstream>
#include <iostream>
#include <queue>




#include <string>
#include <stdio.h>
#include <time.h>


#include "ros/ros.h"
#include <dji_sdk/dji_drone.h>

#include <navigation/conversion.h>
#include <navigation/UasMath.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>



#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "navigation/MPCController.h"
#include "navigation/KalmanFilter.h"
#include "navigation/ExtendedKalmanFilter.h"



#include "rt_nonfinite.h" 
#include "solveQP.h"
#include "solveQP_terminate.h"
#include "solveQP_emxAPI.h"
#include "solveQP_initialize.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

using namespace std;
using namespace Eigen;

//~ #define EKF_DEBUG
#define FUSE_DEBUG

//~ #define DEFAULT_TARGET_TRACKING_LOG_FILE_NAME "/home/ubuntu/TargetTracking_"
#define DEFAULT_GO_TO_TRUCK_LOG_FILE_NAME "/home/ubuntu/FlightTestLog/GoToTruck_"
#define DEFAULT_AUTONOMOUS_LANDING_LOG_FILE_NAME "/home/ubuntu/FlightTestLog/AutonomousLanding_"
#define DEFAULT_SEAECHING_RANGE_LOG_FILE_NAME "/home/ubuntu/SearchRange_"
#define DEFAULT_MPC_CONTROLLER_LOG_FILE_NAME "/home/ubuntu/FlightTestLog/MPCController_"
#define DEFAULT_KALMAN_FILTER_LOG_FILE_NAME "/home/ubuntu/FlightTestLog/KalmanFilter_"
#define DEFAULT_FULL_JOURNEY_LOG_FILE_NAME "/home/ubuntu/FlightTestLog/FullJourney_"
#define TARGET_LOST_TIME_OUT_SEC (1.0)

#define PI 3.1415926
// Searching path parameters
// Cont sin path
#define OMEGA (3.1415926)
#define HEIGHT (3.0)
#define SPEED (0.5)
// spiral path
#define LINE_VELOCITY (0.3)
#define ANGLE_VELOCITY (0.8)  //  rad/s
// common parameters
#define SEARCH_ALTITUDE (3.0)
#define HEIGHT_ERROR (0.15)  // tolerable height error
#define SEARCH_TIME (360.0)
// Gimbal searching 
#define YAW_RANGE (80.0) //degrees
#define RATIO_GIMBAL (10.0) // period time second


// PD controller parameters in Landing
#define KP (0.7)   // set 0.7 for real flight test
#define KD (0.05)   // set 0.05 for real flight test


// LQR integrator
#define KI (0.03)
#define DT (0.08)

DJIDrone* _ptrDrone;

int _nNavigationTask = 0;
bool _bIsDroneLandingPrinted = false;

// Switches
bool _bIsSimulation = false;
bool _bIsIntegralEnable = true;
bool _bIsYawControlEnable = false;
bool _bIsYawControlAlwaysAlign = true;
bool _bIsYawControlEnableSearch = false;
bool _bIsMPCEnable = true; 
bool _bIsLQREnable = false;
bool _bIsLocalLocationControlEnable = false;
bool _bIsKeepLanding = true;
bool _bIsStartSim = false;
bool _bIsLock = false; 
bool _bIsClearIntegratorError = true;
bool _bIsEKFEnable = false;
double _cutoffThreshold = 0.13;

// Target tracking boolean flags 
bool _bIsTargetTrackingRunning = false;
bool _bIsTargetBeingTracked = false;
bool _bIsTargetFound = false;
int _counter = 0;

// Searching path parameters(spiral)
bool _bIsSearchInitiated = false;
bool _bIsMovingYaw = false;

//~ float _SearchCenter_x = 0;
//~ float _SearchCenter_y = 0;
float _FlyingRadius = 0;
int _SearchGimbalPhi = 0;
float _gimbalLimitAngle = -asin(SEARCH_ALTITUDE/8.0)*180.0 / M_PI;
float _Phi = 0;

// Searching path parameters(const speed sinusoidal)
float _StartX = 0;
float _StartY = 0;
float _Theta = 3.1415926/4;

// Autonoumous landing
bool _bIsTestInitiated = false;

// Truck GPS tracking
float _GPSCircleRatio = 1;
float _limitRadius = 1;
bool _IsOnTruckTop = false;
bool _bIsLandingInitiated = false;


// MPC controller
MPCController _mpc;
float _sumPosErrX = 0;
float _sumPosErrY = 0;
float _lastPosErrX = 0;
float _lastPosErrY = 0;
float _sumVecErrX = 0;
float _sumVecErrY = 0;
float _lastVecErrX = 0;
float _lastVecErrY = 0;
bool _bIsFirstTimeReachPitch = true;
bool _bIsFirstTimeReachRoll = true;


// KF
KalmanFilter _kf;
ExtendedKalmanFilter _ekf;
Vector4d _targetEstState;
//~ bool _IsGPSUpdated = false;

// PD controller
float _error = 0;
float _error_last = 0;



// Data Record
// std::ofstream _ofsTragetTrackingLog;
std::ofstream _ofsGoToTruckLog;
std::ofstream _ofsAutonomousLandingLog;
std::ofstream _ofsSearchingRangeLog;
std::ofstream _ofsMPCControllerLog;
std::ofstream _ofsKalmanFilterLog;
std::ofstream _ofsFullJourneyLog;


// GPS & Camera data funsion
std::queue <dji_sdk::Gimbal> _queMsgGimbal;


// Sensor data
sensor_msgs::LaserScan _msgUltraSonic;
geometry_msgs::PointStamped _msgDesiredGimbalPoseDeg;

geometry_msgs::PointStamped _msgTargetLocalPosition;
geometry_msgs::PointStamped _msgTargetDistance;

geometry_msgs::PointStamped _msgTruckDistance;
geometry_msgs::PointStamped _msgTruckLocalPosition;
geometry_msgs::PointStamped _msgTruckGPSPosition;
geometry_msgs::PointStamped _msgRealTruckLocalPosition;
geometry_msgs::PointStamped _msgTruckVelocity;
geometry_msgs::PointStamped _msgFusedTargetPosition; 
geometry_msgs::PointStamped _msgFusedTargetVelocity;

geometry_msgs::PointStamped _msgDesiredAttitudeDeg;

// Publishers
ros::Publisher _GimbalAnglePub;
ros::Publisher _TargetLocalPositionPub;
ros::Publisher _TruckLocalPositionPub;
ros::Publisher _FusedTargetLocalPositionPub;



void ShutDown(void)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate navigation ...");
    
    solveQP_terminate();

    delete(_ptrDrone);
    // _ofsTragetTrackingLog.close();
    _ofsGoToTruckLog.close();
    _ofsAutonomousLandingLog.close();
    _ofsSearchingRangeLog.close();
    _ofsMPCControllerLog.close();
    ROS_INFO("Shutting down navigation ...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();

}

void SigintHandler(int sig)
{
    ShutDown();
}


/**
 *  https://github.com/dji-sdk/Onboard-SDK/blob/3.1/doc/en/ProgrammingGuide.md
 * Example:
 * double roll_rad, pitch_rad, yaw_rad;
 * quaternionToRPY(drone.attitude_quaternion, roll_rad, pitch_rad, yaw_rad);
**/
void quaternionToRPY(dji_sdk::AttitudeQuaternion q, double& roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{
     roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
     pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
     yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}

void ultrasonic_callback(const sensor_msgs::LaserScan& msgUltraSonic)
{
    _msgUltraSonic.header.frame_id = msgUltraSonic.header.frame_id;
    _msgUltraSonic.header.stamp.sec = msgUltraSonic.header.stamp.sec;
    _msgUltraSonic.ranges[0] = msgUltraSonic.ranges[0];
    _msgUltraSonic.intensities[0] = msgUltraSonic.intensities[0];

}

float PDController(float inputs, float current_position)
{

    _error = inputs - current_position;
    float output = KP * _error + KD * (_error - _error_last);
    _error_last = _error;
    return output;
}

float LocalPositionControlHelper(float desired, float current_position)
{

    float error = desired - current_position;
    float abs_error = abs(error);

    return (abs_error < 0.4) ? desired
                         : (abs_error < 5) ? current_position + error * 0.5
                                       : (abs_error < 20) ? current_position + error * 0.3
                                                      : current_position + error * 0.2;

}



float LocalPositionControlAltitudeHelper(float desired, float current_position)
{

    DJIDrone& drone = *_ptrDrone;
    if (_bIsSimulation)
    {     
      if ( desired > (current_position - HEIGHT_ERROR) && desired < (current_position + HEIGHT_ERROR) ){
      return current_position;
    }

    else{
      float setpoint_z = PDController(desired, current_position) + current_position;
      return setpoint_z;
    }
    
    }
    else
    {
    if ((_msgUltraSonic.ranges[0] > 0) && (int)_msgUltraSonic.intensities[0])
    {
      if ( desired > (_msgUltraSonic.ranges[0] - HEIGHT_ERROR) && desired < (_msgUltraSonic.ranges[0] + HEIGHT_ERROR) ){
          return current_position;
      }

      else{
    
        //~ float error = desired - _msgUltraSonic.ranges[0];
        //~ desired = desired + error;
        float setpoint_z = PDController(desired, _msgUltraSonic.ranges[0]) + current_position;
        return setpoint_z;
      }
    }
    else
    {
      if ( desired > (current_position - HEIGHT_ERROR) && desired < (current_position + HEIGHT_ERROR) ){
        return current_position;
      }

      else{
        float setpoint_z = PDController(desired, current_position) + current_position;
        return setpoint_z;
      }
      
      std::cout << "Ultrasonic of guidance is invalid!! " << std::endl;
    }
  }
}


void RunLocalPositionControl(geometry_msgs::Point desired_position, float desired_yaw_deg)
{
    DJIDrone& drone = *_ptrDrone;

    float setpoint_x = LocalPositionControlHelper(desired_position.x, drone.local_position.x);
    float setpoint_y = LocalPositionControlHelper(desired_position.y, drone.local_position.y);
    float setpoint_z = LocalPositionControlAltitudeHelper(desired_position.z, drone.local_position.z);

    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float current_yaw_deg = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );

    float yaw_error_deg = desired_yaw_deg - current_yaw_deg;
    float abs_yaw_error_deg = abs(yaw_error_deg);
    float setpoint_yaw = (abs_yaw_error_deg < 3) ? desired_yaw_deg
                                             : (abs_yaw_error_deg < 10) ? current_yaw_deg + yaw_error_deg * 0.3
                                             : current_yaw_deg + yaw_error_deg * 0.2;

    // Decide whether use yaw control 
    setpoint_yaw = _bIsYawControlEnableSearch ? setpoint_yaw : 0;
    drone.local_position_control(setpoint_x, setpoint_y, setpoint_z, setpoint_yaw);
    
    ROS_INFO(" error_px, error_py: %f, %f ", desired_position.x-drone.local_position.x, desired_position.y-drone.local_position.y);

}


float IntegratorCalculationPos(float stateError, float lastStateError, float sumError)
{

	float limition = 35;
	if((stateError >= 0 && lastStateError >= 0) || (stateError <= 0 && lastStateError <= 0)){	
		if ((sumError < limition) && (sumError > -limition)){
			sumError += stateError*DT;}
		else{
				if (sumError < 0){
					sumError = -limition;}
				else{
					sumError = limition;}
			}			
    }
    else{
        if (_bIsClearIntegratorError)
            {sumError = 0;}
        else
            {sumError += stateError*DT;}
        }

	return sumError;

}

float IntegratorCalculationVec(float stateError, float lastStateError, float sumError)
{
    float limition = 30;
    if((stateError >= 0 && lastStateError >= 0) || (stateError <= 0 && lastStateError <= 0)){  
      if ((sumError < limition) && (sumError > -limition)){
        sumError += stateError*DT;}
      else{
        if (sumError < 0){
          sumError = -limition;}
        else{
          sumError = limition;}
      }      
    }
    else{
    //~ if (_bIsFirstTimeReachPitch){
      //~ _sumPosErrX = 0;
      //~ _mpc.Dp_ = MatrixXd::Zero(_mpc.nx_,1);
      //~ _bIsFirstTimeReachPitch = false;
      //~ }
    //~ else{
    //~ sumError += lastStateError*DT;
    sumError = 0;}
    //~ }
    return sumError;
}




static void argInit_4x1_real_T(double result[4], Vector4d xk)
{
  int idx0;


  for (idx0 = 0; idx0 < 4; idx0++) {

    result[idx0] = xk(idx0);
  }
}

static emxArray_real_T *argInit_Unboundedx1_real_T(VectorXd desiredState)
{
  emxArray_real_T *result;
  static int iv2[1] = { int(_mpc.nx_*_mpc.P_) };

  int idx0;


  result = emxCreateND_real_T(1, iv2);


  for (idx0 = 0; idx0 < result->size[0U]; idx0 = idx0 + 4) {

    result->data[idx0] = desiredState(idx0);
    result->data[idx0+1] = desiredState(idx0+1);
    result->data[idx0+2] = desiredState(idx0+2);
    result->data[idx0+3] = desiredState(idx0+3);
  }

  return result;
}

static double argInit_real_T()
{
  return 0.0;
}


VectorXd RunConstraintedMPC(Vector4d xk, VectorXd desiredState){
    
    double xkArray[4];
    emxArray_real_T *rp;
    emxArray_real_T *x;
    
    // Initialize the system input
    //~ emxInitArray_real_T(&x, int(_mpc.nu_*_mpc.M_)); // system input
    emxInitArray_real_T(&x, 1);

    argInit_4x1_real_T(xkArray, xk);


    rp = argInit_Unboundedx1_real_T(desiredState);



    solveQP((double)_mpc.P_, (double)_mpc.M_, xkArray, rp, _mpc.q_, _mpc.Qk_, _mpc.Qf_, _mpc.Qb_, x);
    
    VectorXd um = MatrixXd::Zero((int)(_mpc.nu_*_mpc.M_),1);
    for( int i = 0; i < x->size[0U] ;i++){
        um(i) = x->data[i];
    } 
    
    emxDestroyArray_real_T(x);
    emxDestroyArray_real_T(rp);
    
    
    std::cout << um.transpose() << std::endl;
    
    return um;
    
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
    VectorXd truckPred;//= Eigen::MatrixXd(P*nx, 1);
    if(_bIsEKFEnable){
        // Fused in Extended Kalman Filter
        _ekf.SetPredHorizon(_ekf.stepsAhead_ + P);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
    
        if(timeElapsed.toSec() > 0.001){
            _targetEstState = _ekf.PredictWOObservation(timeElapsed.toSec());
        }
        truckPred = _ekf.Predict(_targetEstState);
    }
    else{
        // Fused in Kalman Filter
        _kf.SetPredHorizon(_kf.nPred_ + P);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
        if(timeElapsed.toSec() > 0.001){
            // Time elapsed > 0.001 s, update; O.W. no need to update
            _targetEstState = _kf.PredictWOObservation(timeElapsed.toSec());
        }
        truckPred = _kf.Predict(_targetEstState, _kf.nPred_ + P);
    }
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


float YawControlHelper(float desired_yaw_deg){
  
  DJIDrone& drone = *_ptrDrone;
  
  dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float current_yaw_deg = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float yaw_error_deg = desired_yaw_deg - current_yaw_deg;
  float abs_yaw_error_deg = abs(yaw_error_deg);
    float setpoint_yaw = (abs_yaw_error_deg < 3) ? desired_yaw_deg
                                             : (abs_yaw_error_deg < 10) ? current_yaw_deg + yaw_error_deg * 0.3
                                             : current_yaw_deg + yaw_error_deg * 0.2;
    //~ ROS_INFO("Current yaw angle, yaw error, setpoint_yaw:%f,%f,%f", current_yaw_deg, yaw_error_deg, setpoint_yaw);
    
    return setpoint_yaw;
}


void RunAttitudeControl(geometry_msgs::Point desired_position, float desired_yaw_deg){

    DJIDrone& drone = *_ptrDrone;
  
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
    //~ if (_bIsYawControlEnable){
    //~ drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);
  //~ }
  //~ else{
    //~ setpoint_yaw = 0;
      //~ drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);
  //~ }

    // Just record the system input.
    _msgDesiredAttitudeDeg.point.x = setAngleRoll;
    _msgDesiredAttitudeDeg.point.y = -setAnglePitch;
    _msgDesiredAttitudeDeg.point.z = setpoint_yaw;

}

void RunAttitudeControlTest(){

    DJIDrone& drone = *_ptrDrone;

    // Control bits: 0001 0000
    // hotizontal: tilt angle, yaw angle, both ground frame
    // verticle: position
    //
    // Parameters: roll/pitch angle(deg, max:33), height, yaw angle(deg).
    // Notice: negative pitch angle means going to North
    drone.attitude_control(0x10, 10, -10, 3, 90);

}

void UltrasonicTest(void)
{
    ROS_INFO("frame_id: %s, stamp: %d, distance: %f, reliability: %d",
            _msgUltraSonic.header.frame_id.c_str(),
            _msgUltraSonic.header.stamp.sec,
            _msgUltraSonic.ranges[0],
            (int)_msgUltraSonic.intensities[0] );

}

void LandingTest(void)
{
    DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.13) && (int)_msgUltraSonic.intensities[0];

    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        {
            ROS_INFO("The drone has landed!");
            _bIsDroneLandingPrinted = true;
        }
        return;
    }
    else
    {
        ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
        ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
        ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f",
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );
        drone.local_position_control(drone.local_position.x, drone.local_position.y, 0.0, 0);
    }

}


void LandingTestPlus(void)
{
    DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.13) && (int)_msgUltraSonic.intensities[0];

    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        {
            ROS_INFO("The drone has landed!");
            _bIsDroneLandingPrinted = true;
        }
        drone.landing();
        return;
    }
    else
    {
        ROS_INFO("Ultrasonic dist = %f m, reliability = %d", _msgUltraSonic.ranges[0], (int)_msgUltraSonic.intensities[0]);
        ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
        ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f",
                    drone.global_position.longitude,
                    drone.global_position.latitude,
                    drone.global_position.altitude,
                    drone.global_position.height
                 );
        drone.local_position_control(drone.local_position.x, drone.local_position.y, 0.0, 0);
    }
}



void TemporaryTest(void)
{

    DJIDrone& drone = *_ptrDrone;
                      
    drone.local_position_control(-10, 0, 3, 0);
    //~ drone.velocity_control(0x00,-5,0,0,0);
    std::cout << "velocity: vx, vy: " << drone.velocity.vx << ", " << drone.velocity.vy << std::endl;
                      
                      
}



geometry_msgs::PointStamped GetTargetOffsetFromUAV( geometry_msgs::Point& tagPosition_M,
                                                    dji_sdk::Gimbal& gimbal)
{
    // rotation matrix will be calculated based on instructions
    // here: http://planning.cs.uiuc.edu/node102.html
    // first, rename some variables to make calculations more readable

    double yaw = UasMath::ConvertDeg2Rad(gimbal.yaw);
    double pitch = UasMath::ConvertDeg2Rad(gimbal.pitch);
    double roll = UasMath::ConvertDeg2Rad(gimbal.roll);

    //printf("\n confirm that roll pitch yaw is respectively %f %f %f \n", roll, pitch, yaw);
    double cameraRotationMatrix[3][3] =
        {   {
                cos(yaw)*cos(pitch),
                cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),
                cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)
            },
            {
                sin(yaw)*cos(pitch),
                sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),
                sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)
            },
            {
                -1.0*sin(pitch),
                cos(pitch)*sin(roll),
                cos(pitch)*cos(roll)
            },
        };

    // WARNING: These x, y, and z values are relative to the camera frame,
    // NOT THE GROUND FRAME!
    // This is what we want and why we'll multiply by the rotation matrix

    double targetOffsetFromCamera[3][1] = {{tagPosition_M.x}, {tagPosition_M.y}, {tagPosition_M.z}};

    // perform matrix multiplication
    // (recall that you take the dot product of the 1st matrix rows with the 2nd matrix colums)
    // process is very simple since 2nd matrix is a vertical vector

    // first, convert from image plane to real-world coordinates
    double transformMatrix[3][3] = { {0,0,1}, {1,0,0}, {0,-1,0} };

    //now we can determine the distance in the inertial frame
    double distanceInRealWorld[3][1];


    for (int row = 0; row < 3; row++)
    {
        double sum = 0;
        for (int column = 0; column < 3; column++)
        {
            sum += transformMatrix[row][column] * targetOffsetFromCamera [column][0];
        }
        distanceInRealWorld[row][0] = sum;
    }
    //end matrix multiplication

    // now we can determine the actual distance from the UAV, by accounting for the camera's orientation
    double targetOffsetFromUAV[3][1];

    for (int row = 0; row < 3; row++)
    {
        double sum = 0;
        for (int column = 0; column < 3; column++)
        {
            sum += cameraRotationMatrix[row][column] * distanceInRealWorld [column][0];
        }
        targetOffsetFromUAV[row][0] = sum;
    }

    geometry_msgs::PointStamped output;

    output.header.stamp = ros::Time::now();

    // Because drone.local_position.x means northing
    // and drone.local_position.y means easting, so is the targetOffsetFromUAV.
    // Careful utm.x means easting, utm.y means northing.
    output.point.x = targetOffsetFromUAV[0][0];
    output.point.y = targetOffsetFromUAV[1][0];
    output.point.z = targetOffsetFromUAV[2][0];

    return output;

} ///end GetTargetOffsetFromUAV()


dji_sdk::Gimbal FindGimbalAngleForApriltag(apriltags_ros::AprilTagDetection& tag)
{

    dji_sdk::Gimbal gimbal;

    while(!_queMsgGimbal.empty())
    {
        gimbal = _queMsgGimbal.front();
        if (gimbal.header.stamp > tag.pose.header.stamp)
        {
            return gimbal;
        }
        _queMsgGimbal.pop();
    }

    return _ptrDrone->gimbal;

}


void FindDesiredGimbalAngle(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{

    DJIDrone& drone = *_ptrDrone;
    char zone;
    double dLastRecordedHeightAboveTargetM = 0.0;

    if (vecTagDetections.detections.empty())
    {
        // There is nothing we can do
        return;
    }

    // We assume there is only one tag detection.
    // Even if there are multiple detections, we still take the first tag.

    apriltags_ros::AprilTagDetection tag = vecTagDetections.detections.at(0);

    double x = tag.pose.pose.position.x;
    // since we want camera frame y to be above the camera, but it treats this as negative y
    double y = -tag.pose.pose.position.y;
    double z = tag.pose.pose.position.z;

    // double currentTime = tag.pose.header.stamp.nsec/1000000000.0 + tag.pose.header.stamp.sec;

    // for exaplanation of calculations, see diagram in Aaron Ward's July 20 report
    // pitch_rads = -1.0 * atan2( heightAboveTarget_Meters,  distance_Meters );
    // this is done correctly since we want to limit it to between 0 and -90 degrees (in fact could just use regular tangent)
    double pitchDeg = UasMath::ConvertRad2Deg(atan2(y, z));

    double yawDeg = UasMath::ConvertRad2Deg(atan2(x, z));
    //remember north is the x axis, east is the y axis


    geometry_msgs::PointStamped msgDesiredAngleDeg;
    msgDesiredAngleDeg.point.x = 0;
    msgDesiredAngleDeg.point.y = drone.gimbal.pitch + pitchDeg;
    msgDesiredAngleDeg.point.z = drone.gimbal.yaw + yawDeg;

    _GimbalAnglePub.publish(msgDesiredAngleDeg);
    //~ ROS_INFO("Publish desired gimbal angle!! ");
    tag.pose.pose.position.y *= -1;


    //dji_sdk::Gimbal gimbal = FindGimbalAngleForApriltag(tag);
    //_msgTargetDistance = GetTargetOffsetFromUAV(tag.pose.pose.position, gimbal);

    _msgTargetDistance = GetTargetOffsetFromUAV(tag.pose.pose.position, drone.gimbal);

    _msgTargetLocalPosition.header.stamp = ros::Time::now();
    // drone.local_position.x means northing
    // drone.local_position.y means easting
    _msgTargetLocalPosition.point.x = drone.local_position.x + _msgTargetDistance.point.x;
    _msgTargetLocalPosition.point.y = drone.local_position.y + _msgTargetDistance.point.y;
    _msgTargetLocalPosition.point.z = 0;

    _TargetLocalPositionPub.publish(_msgTargetLocalPosition);



    if(_bIsEKFEnable){
        // Update estimate by extended kalman filter
        Vector2d targetState(_msgTargetLocalPosition.point.x, _msgTargetLocalPosition.point.y);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
        _targetEstState = _ekf.UpdateWithCameraMeasurements(targetState, timeElapsed.toSec());

        if(_bIsLock){   
            return; 
        }
        else{
            _bIsLock = true;
            _msgFusedTargetPosition.header.stamp = ros::Time::now();
            _msgFusedTargetPosition.point.x = _targetEstState(0);
            _msgFusedTargetPosition.point.y = _targetEstState(1);
            _msgFusedTargetPosition.point.z = 0;

            _msgFusedTargetVelocity.point.x = _targetEstState(2)*cos(_targetEstState(3));
            _msgFusedTargetVelocity.point.y = _targetEstState(2)*sin(_targetEstState(3));
            _msgFusedTargetVelocity.point.z = 0;
            _bIsLock = false;
        }
    }
    else{

        // Update estimate by kalman filter
        Vector2d targetState(_msgTargetLocalPosition.point.x, _msgTargetLocalPosition.point.y);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
        _targetEstState = _kf.UpdateWithCameraMeasurements(targetState, timeElapsed.toSec());

        if(_bIsLock){   
            return; 
        }
        else{
            _bIsLock = true;
            _msgFusedTargetPosition.header.stamp = ros::Time::now();
            _msgFusedTargetPosition.point.x = _targetEstState(0);
            _msgFusedTargetPosition.point.y = _targetEstState(1);
            _msgFusedTargetPosition.point.z = 0;

            _msgFusedTargetVelocity.point.x = _targetEstState(2);
            _msgFusedTargetVelocity.point.y = _targetEstState(3);
            _msgFusedTargetVelocity.point.z = 0;
            _bIsLock = false;
        }
    }
    


    // Logging
    _bIsTargetBeingTracked = true;

    // _ofsTragetTrackingLog << std::setprecision(std::numeric_limits<double>::max_digits10)
    //                     << ros::Time::now().toSec() << ","
    //                     << x << "," << y << "," << z << ","  // tag distance
    //                     << _msgTargetDistance.point.x << ","
    //                     << _msgTargetDistance.point.y << ","
    //                     << _msgTargetDistance.point.z << "," // target distance
    //                     << _msgTargetLocalPosition.point.x << ","
    //                     << _msgTargetLocalPosition.point.y << ","
    //                     << _msgTargetLocalPosition.point.z << std::endl; // target local position

}



// // This the original tag_detection callback
void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{
     // If Target Tracking is not running, do nothing.
     if (!_bIsTargetTrackingRunning) { return; }

     FindDesiredGimbalAngle(vecTagDetections);
}


void truckVelocityCallback(const geometry_msgs::PointStamped msgTruckVelocity)
{

    // Record GPS position
    _msgTruckVelocity.point.x = msgTruckVelocity.point.x;
    _msgTruckVelocity.point.y = msgTruckVelocity.point.y;
    _msgTruckVelocity.point.z = msgTruckVelocity.point.z;
    
    

}

void startSimCallback(const geometry_msgs::PointStamped msgStartSim)
{
  
  _nNavigationTask = msgStartSim.point.x;
  _bIsStartSim = true;
  
}

void truckPositionCallback(const geometry_msgs::PoseStamped msgTruckPosition)
{
  
  // drone.local_position.x means northing
    // drone.local_position.y means easting
    
    DJIDrone& drone = *_ptrDrone;
    
    // Calculate the distance from truck to drone
    // Note: (msgTruckPosition.pose).position holds noisy measurement, .orientation holds true value
    _msgTruckDistance.point.x = (msgTruckPosition.pose.position.x - drone.global_position.latitude)/0.0000089354;
    _msgTruckDistance.point.y = (msgTruckPosition.pose.position.y - drone.global_position.longitude)/0.0000121249;
    _msgTruckDistance.point.z = 0;
    
    // Calculate the truck local location
    _msgTruckLocalPosition.header.stamp = ros::Time::now();
    _msgTruckLocalPosition.point.x = drone.local_position.x + _msgTruckDistance.point.x;
    _msgTruckLocalPosition.point.y = drone.local_position.y + _msgTruckDistance.point.y;
    _msgTruckLocalPosition.point.z = 0;
    _TruckLocalPositionPub.publish(_msgTruckLocalPosition);
    
    if (_bIsSimulation)
    {
    double realtruckDistance_x = (msgTruckPosition.pose.orientation.x - drone.global_position.latitude)/0.0000089354;
    double realtruckDistance_y = (msgTruckPosition.pose.orientation.y - drone.global_position.longitude)/0.0000121249;
    
    // Calculate the real truck local location
    _msgRealTruckLocalPosition.header.stamp = ros::Time::now();
    _msgRealTruckLocalPosition.point.x = drone.local_position.x + realtruckDistance_x;
    _msgRealTruckLocalPosition.point.y = drone.local_position.y + realtruckDistance_y;
    _msgRealTruckLocalPosition.point.z = 0;
    }
  
    // Record GPS position
    _msgTruckGPSPosition.point.x = msgTruckPosition.pose.position.x;
    _msgTruckGPSPosition.point.y = msgTruckPosition.pose.position.y;
    _msgTruckGPSPosition.point.z = msgTruckPosition.pose.position.z;
    


    // If the Truck is extremely far away from drone, then there must be something wrong
    if( abs(_msgTruckDistance.point.x) > 100 && abs(_msgTruckDistance.point.y) > 100 ) {
        ROS_INFO_STREAM("DANGER: There is something wrong with the Truck GPS or Drone GPS. Plz check NOW!!!!!");
        _bIsTargetTrackingRunning = false;  // mission cancel
        _nNavigationTask = 98;
        return;
    }
    
    
    
    // Update estimate by kalman filter
    Vector4d targetState(_msgTruckLocalPosition.point.x, _msgTruckLocalPosition.point.y, _msgTruckVelocity.point.x, _msgTruckVelocity.point.y);
    if(_bIsEKFEnable){
    
        _ekf.SetXhatInitialPoint(targetState);
        ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
        _targetEstState = _ekf.UpdateWithGPSMeasurements(targetState, timeElapsed.toSec());
    
        if(_bIsLock){   
            return; 
        }
        else{    
            _bIsLock = true;
            _msgFusedTargetPosition.header.stamp = ros::Time::now();
            _msgFusedTargetPosition.point.x = _targetEstState(0);
            _msgFusedTargetPosition.point.y = _targetEstState(1);
            _msgFusedTargetPosition.point.z = 0;

            _msgFusedTargetVelocity.point.x = _targetEstState(2)*cos(_targetEstState(3));
            _msgFusedTargetVelocity.point.y = _targetEstState(2)*sin(_targetEstState(3));
            _msgFusedTargetVelocity.point.z = 0;
            _bIsLock = false;
        }
    
    }
    else{
    
        _kf.SetXhatInitialPoint(targetState);

        if (!_kf.IsXhatInitialized_){ 
            _kf.IsXhatInitialized_ = true;
            _targetEstState = targetState;
        }
        else{
            ros::Duration timeElapsed = ros::Time::now() - _msgFusedTargetPosition.header.stamp;
            _targetEstState = _kf.UpdateWithGPSMeasurements(targetState, timeElapsed.toSec());
        }


        if(_bIsLock){   
            return; 
        }
        else{
            _bIsLock = true;
            _msgFusedTargetPosition.header.stamp = ros::Time::now();
            _msgFusedTargetPosition.point.x = _targetEstState(0);
            _msgFusedTargetPosition.point.y = _targetEstState(1);
            _msgFusedTargetPosition.point.z = 0;

            _msgFusedTargetVelocity.point.x = _targetEstState(2);
            _msgFusedTargetVelocity.point.y = _targetEstState(3);
            _msgFusedTargetVelocity.point.z = 0;
            _bIsLock = false;
        }
    }
    
  
    
      _ofsKalmanFilterLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            << ros::Time::now().toSec() << ","              
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
                            << _msgRealTruckLocalPosition.point.z 
                            << std::endl;  

}

//~ void realTruckPositionCallback(const geometry_msgs::PointStamped msgTruckPosition)
//~ {
//~ 
    //~ DJIDrone& drone = *_ptrDrone;
    //~ // Record GPS position
//~ 
    //~ // Calculate the distance from truck to drone
    //~ float truckDistance_x = (msgTruckPosition.point.x - drone.global_position.latitude)/0.0000089354;
    //~ float truckDistance_y = (msgTruckPosition.point.y - drone.global_position.longitude)/0.0000121249;
//~ 
//~ 
    //~ // Calculate the truck local location
    //~ // drone.local_position.x means northing
    //~ // drone.local_position.y means easting
    //~ _msgRealTruckLocalPosition.header.stamp = ros::Time::now();
    //~ _msgRealTruckLocalPosition.point.x = drone.local_position.x + truckDistance_x;
    //~ _msgRealTruckLocalPosition.point.y = drone.local_position.y + truckDistance_y;
    //~ _msgRealTruckLocalPosition.point.z = 0;
//~ 
//~ 
//~ }



/*
void gimbalCallback(const dji_sdk::Gimbal gimbal)
{
    _queMsgGimbal.push(gimbal);
}
*/

void RunTargetSearch()
{

    DJIDrone& drone = *_ptrDrone;

    //If target found, then set variables to initial values(in case next time) and return
    if( _bIsTargetFound ){

        _bIsSearchInitiated = false;
        //~ _SearchCenter_x = 0;
        //~ _SearchCenter_y = 0;
        
        //~ _nNavigationTask = 98;

        return;
    }

    // Search parameters
    float initialRadius = 0;
    float limitRadius = 6;
    // float circleRadiusIncrements = 1;

    // Initialize the search center
    if(!_bIsSearchInitiated)
    {
        _bIsSearchInitiated = true;
        //~ _SearchCenter_x = _msgFusedTargetLocalPosition.point.x;
        //~ _SearchCenter_y = _msgFusedTargetLocalPosition.point.y;
        
        // _StartX = _msgFusedTargetLocalPosition.point.x - 5;
        // _StartY = _msgFusedTargetLocalPosition.point.y;

        _FlyingRadius = initialRadius;
        _Phi = 2;

    }


    bool bIsDroneOutOfRange = _FlyingRadius > limitRadius;
    if(!bIsDroneOutOfRange)
    {

        // if(_Phi < (189*_FlyingRadius*_ratio))
        if(_Phi < (SEARCH_TIME))
        {
            

            //set up drone task, spiral path
            // calculate waypoints for searching path
            // float x =  _SearchCenter_x + LINE_VELOCITY*_Phi*cos(_Phi*ANGLE_VELOCITY);
            // float y =  _SearchCenter_y + LINE_VELOCITY*_Phi*sin(_Phi*ANGLE_VELOCITY);
            // float x_distance = x - drone.local_position.x;
            // float y_distance = y - drone.local_position.y;
            // _FlyingRadius = sqrt((x - _SearchCenter_x)*(x - _SearchCenter_x) + (y - _SearchCenter_y)*(y - _SearchCenter_y));
            
            _FlyingRadius = sqrt((drone.local_position.x - _msgTruckLocalPosition.point.x)*(drone.local_position.x - _msgTruckLocalPosition.point.x) 
              + (drone.local_position.y - _msgTruckLocalPosition.point.y)*(drone.local_position.y - _msgTruckLocalPosition.point.y));
      // const speed sin path searching
            float xdot = SPEED * cos(_Theta);
            _StartX = _StartX + xdot*0.025;
            float y = _StartY + HEIGHT * sin(OMEGA*_StartX); 
            _Theta = atan(HEIGHT * OMEGA * cos(OMEGA * _StartX));
            float x_distance = _StartX - drone.local_position.x;
            float y_distance = y - drone.local_position.y;
            
            float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(y_distance, x_distance));   
                     
            float distance_square =  x_distance*x_distance + y_distance*y_distance;
            
            float delta_x = x_distance / sqrt(distance_square) * _GPSCircleRatio;
            float delta_y = y_distance / sqrt(distance_square) * _GPSCircleRatio;

            float target_x = _StartX - delta_x;
            float target_y = y - delta_y;
            

            //~ bool bIsClose = distance_square < 3;
            bool bIsClose = true;
            geometry_msgs::Point desired_position;
            desired_position.x = bIsClose ? _StartX : target_x;
            desired_position.y = bIsClose ? y : target_y;
            desired_position.z = SEARCH_ALTITUDE;
 
            RunLocalPositionControl(desired_position, 0);
            ROS_INFO("desired posistion and calcultion and fly radius: %f, %f, %f, %f",_StartX,_StartY,xdot,_FlyingRadius);
            //~ ROS_INFO("Now Radius = %f m, Local Position: %f, %f., GimbalLimaitionAngle: %f degree.", _FlyingRadius, drone.local_position.x, drone.local_position.y, _gimbalLimitAngle);
            // RunAttitudeControl(desired_position, desired_yaw);
            // drone.local_position_control(x, y, _searchAltitude, desired_yaw);

            _Phi = _Phi + 0.05;

            // Sprail gimbal searching
            // float pitch_angle = _gimbalLimitAngle-25-(65+_gimbalLimitAngle)/2 + (65+_gimbalLimitAngle)/2*cos(((float)_SearchGimbalPhi/(_ratio_gimbal*50))*6.28);
            // float yaw_angle = _YawRange*sin(((float)_SearchGimbalPhi/(_ratio_gimbal*50))*6.28);

            // const speed sin path searching
            float pitch_angle = -45;
            float yaw_angle = YAW_RANGE*sin(((float)_SearchGimbalPhi/(RATIO_GIMBAL*40))*6.28);
            geometry_msgs::PointStamped msgDesiredAngleDeg;
            msgDesiredAngleDeg.point.x = 0;
            msgDesiredAngleDeg.point.y = pitch_angle;
            msgDesiredAngleDeg.point.z = _bIsMovingYaw ? yaw_angle : 0;
            ROS_INFO("Pitch = %f , Yaw: %f.", pitch_angle, yaw_angle);
            _GimbalAnglePub.publish(msgDesiredAngleDeg);
            _SearchGimbalPhi = _SearchGimbalPhi + 1;
            if (_SearchGimbalPhi > RATIO_GIMBAL*40){
                _SearchGimbalPhi = 0;
            }
            

        }
        else
        {
            _Phi = 2;
            // _FlyingRadius += circleRadiusIncrements;
            _bIsSearchInitiated = false;


        }
    }
    else
    {
        _bIsSearchInitiated = false;
        //~ _SearchCenter_x = 0;
        //~ _SearchCenter_y = 0;

        _Phi = 2;
        _FlyingRadius = initialRadius;

        ROS_INFO("No target found! Try to change searching range or search again. ");
        //_nNavigationTask = 98;

        // If finished searching, then maybe go back to Truck GPS location.
        _IsOnTruckTop = false;


    }
    // Record searching data
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    
    _ofsSearchingRangeLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                  << ros::Time::now().toSec() << ","
                  <<  _msgUltraSonic.ranges[0] << ","
                  << _msgUltraSonic.ranges[0] << ","
                  << drone.local_position.x << ","
                  << drone.local_position.y << ","
                  << drone.local_position.z << ","
                  << drone.velocity.vx << ","
                  << drone.velocity.vy << ","
                  << drone.velocity.vz << ","
                  << drone.gimbal.yaw << ","
                  << drone.gimbal.pitch << ","
                  << yaw << std::endl;

}


void RunTimeCriticalTasks()
{
    if (_bIsTargetTrackingRunning)
    {
        //~ RunTargetSearch();
    }
    
    // target tracking has not been initiated. Do nothing
    if (!_bIsTargetTrackingRunning) { return; }

    // time since we saw the target last time.
    ros::Duration timeElapsed = ros::Time::now() - _msgTargetLocalPosition.header.stamp;

    // Check if target still found
    // if the time elapsed is larger than a predefined time, we have lost the target.
    // But if height < 0.5m, we dont change status
    _bIsTargetFound = !(timeElapsed.toSec() - TARGET_LOST_TIME_OUT_SEC > 0);
    //~ _bIsTargetFound = _msgUltraSonic.ranges[0] < 0.5;

    if (!_bIsTargetFound)
    {
        _bIsTargetBeingTracked = false;
        //~ RunTargetSearch();
        return;
    }

    // Now target tracking is running and the target is being tracked
    // We can predict the next target position
    // TODO: implement target prediction here. (Kalman filter)
    
    
}


void RunAutonomousLanding()
{

    DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < _cutoffThreshold) && (int)_msgUltraSonic.intensities[0];
    if (bIsDroneLanded)
    {
        if (!_bIsDroneLandingPrinted)
        {
            ROS_INFO("The drone has landed!");
            _bIsDroneLandingPrinted = true;
            _bIsTestInitiated = false;
        }
        drone.landing();
        return;
    }

    float target_x = _msgTargetLocalPosition.point.x;
    float target_y = _msgTargetLocalPosition.point.y;
    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y;
    float drone_z = drone.local_position.z;

    float limitRadius = 1;
    float limitRadius_square = limitRadius*limitRadius;

    float distance_square = _msgTargetDistance.point.x*_msgTargetDistance.point.x
                          + _msgTargetDistance.point.y*_msgTargetDistance.point.y;

    bool bIsClose = distance_square < limitRadius_square;

    geometry_msgs::Point desired_position;

    desired_position.x = _msgTargetLocalPosition.point.x;
    desired_position.y = _msgTargetLocalPosition.point.y;
    desired_position.z = bIsClose ? -0.1 : drone_z;

    float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTargetDistance.point.y, _msgTargetDistance.point.x));

    RunLocalPositionControl(desired_position, desired_yaw);

    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );

    _ofsAutonomousLandingLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            << ros::Time::now().toSec() << ","
                            << _msgUltraSonic.ranges[0] << ","
                            << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                            << distance_square << ","                   // distance squared
                            << _msgTargetLocalPosition.point.x << ","
                            << _msgTargetLocalPosition.point.y << ","
                            << _msgTargetLocalPosition.point.z << ","   // target local position
                            << drone.local_position.x << ","
                            << drone.local_position.y << ","
                            << drone.local_position.z << ","
                            << drone.velocity.vx << ","
                            << drone.velocity.vy << ","
                            << drone.velocity.vz << ","
                            << yaw << std::endl;                             // drone local position

}


void RunAutonomousLanding2()
{

    DJIDrone& drone = *_ptrDrone;
  
    bool bIsDroneLanded = false;
    if(_bIsSimulation){  bIsDroneLanded = drone.local_position.z < 0.1;  }
    else{        bIsDroneLanded = (_msgUltraSonic.ranges[0] < _cutoffThreshold) && (int)_msgUltraSonic.intensities[0];}
    
    if (bIsDroneLanded)
    {
      if (!_bIsDroneLandingPrinted)
      {
          ROS_INFO("The drone has landed!");
          _bIsDroneLandingPrinted = true;
          _bIsTestInitiated = false;
      }
      drone.landing(); 
      return;
    }

    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y;
    float drone_z = drone.local_position.z;
    float limitRadius = 0.8;
    float horiDistance = 0;
    
    if (_bIsSimulation)
    {    
      horiDistance = sqrt(pow((_msgTruckLocalPosition.point.x - drone_x),2) + pow((_msgTruckLocalPosition.point.y - drone_y),2));
      bool bIsClose = horiDistance < limitRadius;
      bool bIsStartLanding = false;
      if (bIsClose)
      {
        _counter = _counter + 1;
      }
      if ((_counter > 160) && (bIsClose)) //240
      {
        bIsStartLanding = true;
        ROS_INFO("The drone has landed!*********************************************************");
      }
      // bool bIsClose = true;

      // float set_landing_point_z = LocalPositionControlAltitudeHelper(-0.1, drone.local_position.z);
  
      geometry_msgs::Point desired_position;
      desired_position.x = _msgTruckLocalPosition.point.x;
      desired_position.y = _msgTruckLocalPosition.point.y;;
      desired_position.z = bIsStartLanding ? -0.1 : drone_z;
      
      float desired_yaw = 0;
      if (_bIsYawControlEnable){  desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckVelocity.point.y, _msgTruckVelocity.point.x));  }
      if (_bIsLocalLocationControlEnable)
      {   RunLocalPositionControl(desired_position, desired_yaw);}
      else
      {   RunAttitudeControl(desired_position, desired_yaw);}
      //~ RunLocalPositionControl(desired_position, desired_yaw);
      //~ float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckLocalPosition.point.y, _msgTruckLocalPosition.point.x));
    }
    else
    {
    
#ifndef FUSE_DEBUG
    
        horiDistance = sqrt(pow((_msgTargetLocalPosition.point.x - drone_x),2) + pow((_msgTargetLocalPosition.point.y - drone_y),2));
        bool bIsClose = horiDistance < limitRadius;
        
        geometry_msgs::Point desired_position;
        desired_position.x = _msgTargetLocalPosition.point.x;
        desired_position.y = _msgTargetLocalPosition.point.y;
        desired_position.z = bIsClose ? -0.1 : drone_z;
        //~ ROS_INFO("desired_position: %f, %f, %f",desired_position.x, desired_position.y, desired_position.z);
        
        
        float desired_yaw = 0;
        if (_bIsYawControlEnable){  desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckVelocity.point.y, _msgTruckVelocity.point.x));  }
    
        if (_bIsLocalLocationControlEnable)
        {  RunLocalPositionControl(desired_position, desired_yaw);}
        else
        {  RunAttitudeControl(desired_position, desired_yaw);}
    
#else 
    
        horiDistance = sqrt(pow((_msgFusedTargetPosition.point.x - drone_x),2) + pow((_msgFusedTargetPosition.point.y - drone_y),2));
        bool bIsClose = horiDistance < limitRadius;
        
        geometry_msgs::Point desired_position;
        desired_position.x = _msgFusedTargetPosition.point.x;
        desired_position.y = _msgFusedTargetPosition.point.y;
        if (_bIsKeepLanding) {  desired_position.z = -0.1;  }
        else {          desired_position.z = bIsClose ? -0.1 : SEARCH_ALTITUDE;  }
        //~ ROS_INFO("desired_position: %f, %f, %f",desired_position.x, desired_position.y, desired_position.z);
        
        
        float desired_yaw = 0;
        if (_bIsYawControlEnable){  desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckVelocity.point.y, _msgTruckVelocity.point.x));  }
        RunAttitudeControl(desired_position, desired_yaw);   

        
#endif

    }
    
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float pitch = (float)UasMath::ConvertRad2Deg( asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1)) );
    float roll = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)) );

    _ofsAutonomousLandingLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                     << ros::Time::now().toSec() << ","
                     <<  _msgUltraSonic.ranges[0] << ","
                     << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                     << horiDistance << ","
                     << _msgTargetLocalPosition.point.x << ","
                     << _msgTargetLocalPosition.point.y << ","
                     << _msgTargetLocalPosition.point.z << ","   // target local position
                     << _msgFusedTargetPosition.point.x << ","
                     << _msgFusedTargetPosition.point.y << ","
                     << _msgFusedTargetPosition.point.z << ","  
                     << _msgFusedTargetVelocity.point.x << ","
                     << _msgFusedTargetVelocity.point.y << ","
                     << _msgFusedTargetVelocity.point.z << ","                    
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
                     << yaw << ","
                     << _msgTruckVelocity.point.x<< ","
                     << _msgTruckVelocity.point.y<< std::endl;                                // drone local position

}



void GoToTruckGPSLocation()
{
    DJIDrone& drone = *_ptrDrone;

#ifndef FUSE_DEBUG 
   
    if( _bIsTargetFound ){
        //~ _SearchCenter_x = 0;
        //~ _SearchCenter_y = 0;
        ROS_INFO("Found target in approaching state!!");
        //~ _nNavigationTask = 98;
        return;
    }

    // Go to 1m away from the target
    float delta_x = _msgTruckDistance.point.x / sqrt( _msgTruckDistance.point.x*_msgTruckDistance.point.x
                          + _msgTruckDistance.point.y*_msgTruckDistance.point.y ) * _GPSCircleRatio;
    float delta_y = _msgTruckDistance.point.y / sqrt( _msgTruckDistance.point.x*_msgTruckDistance.point.x
                          + _msgTruckDistance.point.y*_msgTruckDistance.point.y ) * _GPSCircleRatio;


    // Now the destination is 1m away from the target
    float target_x = _msgTruckLocalPosition.point.x - delta_x;
    float target_y = _msgTruckLocalPosition.point.y - delta_y;
    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y;
    float drone_z = drone.local_position.z;

    float horiDistance = sqrt(pow((_msgTruckLocalPosition.point.x - drone_x), 2) + pow((_msgTruckLocalPosition.point.y - drone_y), 2));
    // bool bIsClose = distance_square < limitRadius_square;
    //~ bool bIsClose = true;
    bool bIsClose = horiDistance < 3;
    _IsOnTruckTop = horiDistance < 1; // Hard coded, define if < 0.7 m2, then means on the top

    // Calculate desired postion and yaw; then send commands
    geometry_msgs::Point desired_position;
    desired_position.x = bIsClose ? _msgTruckLocalPosition.point.x : target_x;
    desired_position.y = bIsClose ? _msgTruckLocalPosition.point.y : target_y;
    desired_position.z = SEARCH_ALTITUDE;

#else

    float target_x = _msgFusedTargetPosition.point.x;
    float target_y = _msgFusedTargetPosition.point.y;
    float drone_x = drone.local_position.x;
    float drone_y = drone.local_position.y;
    float drone_z = drone.local_position.z;
    float horiDistance = sqrt(pow((target_x - drone_x), 2) + pow((target_y - drone_y), 2));
    
    // bool bIsClose = horiDistance < limitRadius_square;
    _IsOnTruckTop = horiDistance < _limitRadius; // Hard coded, define if < 0.7 m2, then means on the top

    // Calculate desired postion and yaw; then send commands
    geometry_msgs::Point desired_position;
    desired_position.x = target_x;
    desired_position.y = target_y;
    desired_position.z = SEARCH_ALTITUDE;  
    
#endif


    float desired_yaw = 0;
    if (_bIsYawControlEnable){  
        if (_bIsYawControlAlwaysAlign){
            desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckVelocity.point.y, _msgTruckVelocity.point.x));
        }
        else{
            desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckDistance.point.y, _msgTruckDistance.point.x));
        }  
    }
    RunAttitudeControl(desired_position, desired_yaw);   

    // Calculate yaw angle
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float pitch = (float)UasMath::ConvertRad2Deg( asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1)) );
    float roll = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)) );


  // searching target with gimbal sweep
  if (horiDistance < 6 && !_bIsTargetFound)
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
    if (_SearchGimbalPhi > RATIO_GIMBAL*40){
      _SearchGimbalPhi = 0;
        }
    }   
    // Print out data
    //~ ROS_INFO("Desired Local position:%f, %f ,horiDistance:%f, Close?:%d.",desired_position.x, desired_position.y, horiDistance, bIsClose);
    //~ ROS_INFO("Truck local position: %f, %f, %f.", _msgTruckLocalPosition.point.x, _msgTruckLocalPosition.point.y, _msgTruckLocalPosition.point.z);
    //~ ROS_INFO("Local Position: %f, %f", drone.local_position.x, drone.local_position.y);
    //~ ROS_INFO("Truck GPS position: lat:%f, lon:%f, alt:%f.", _msgTruckGPSPosition.point.x, _msgTruckGPSPosition.point.y, _msgTruckGPSPosition.point.z);
    //~ ROS_INFO("Global Position: lon:%f, lat:%f, alt:%f, height:%f",
                    //~ drone.global_position.longitude,
                    //~ drone.global_position.latitude,
                    //~ drone.global_position.altitude,
                    //~ drone.global_position.height
                 //~ );

    // Record data in file
    _ofsGoToTruckLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                     << ros::Time::now().toSec() << ","
                     <<  _msgUltraSonic.ranges[0] << ","
                     << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                     << horiDistance << ","
                     << _msgTruckLocalPosition.point.x << ","
                     << _msgTruckLocalPosition.point.y << ","
                     << _msgTruckLocalPosition.point.z << ","
                     << _msgTruckVelocity.point.x << ","
                     << _msgTruckVelocity.point.y << ","
                     << _msgFusedTargetPosition.point.x << ","
                     << _msgFusedTargetPosition.point.y << ","
                     << _msgFusedTargetPosition.point.z << ","
                     << _msgFusedTargetVelocity.point.x << ","
                     << _msgFusedTargetVelocity.point.y << ","
                     << _msgFusedTargetVelocity.point.z << ","
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


void RunAutonomousLanding3(){

    //~ ROS_INFO("IsOnTruckTop?:%d, _bIsLandingInitiated?:%d", _IsOnTruckTop, _bIsLandingInitiated);

    if(!_IsOnTruckTop && !_bIsLandingInitiated){
        GoToTruckGPSLocation();
        return;
    }
    else {
        RunAutonomousLanding2();
        _bIsLandingInitiated = true;
        return;
    }

    // if(_bIsTargetFound){
    //     _bIsSearchInitiated = false;
        
    //     return;
    // }

}


void timerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;

    // we need to run this functioin regardless of the navigation menu.
    RunTimeCriticalTasks();


    //~ if (_nNavigationTask < 21 || _nNavigationTask > 90)
    //~ // we don't take care of these cases in this callback function.
    //~ // They are taken care in navigationTaskCallback.
    //~ {
        //~ return;
    //~ }

    switch (_nNavigationTask)
    {
        case 21:
            RunAutonomousLanding();
            break;

        case 22:
            RunAutonomousLanding2();
            break;

        case 23:
            RunAutonomousLanding3();
            break;

        case 24:
            RunTargetSearch();
            break;

        case 25:
            GoToTruckGPSLocation();
            break;
            
        case 26:
            break;

        case 30:
            RunAttitudeControlTest();
            break;

        case 31:
            UltrasonicTest();
            break;

        case 32:
            LandingTest();
            break;

        case 33:
            LandingTestPlus();
            break;

        case 34:
            TemporaryTest();
            break;

        default: // It will take care of invalid inputs
            break;
    }

    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float pitch = (float)UasMath::ConvertRad2Deg( asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1)) );
    float roll = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)) );

    _ofsFullJourneyLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                     << ros::Time::now().toSec() << ","
                     <<  _msgUltraSonic.ranges[0] << ","
                     << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                     << _msgTruckLocalPosition.point.x << ","
                     << _msgTruckLocalPosition.point.y << ","
                     << _msgTruckLocalPosition.point.z << "," 
                     << _msgTruckVelocity.point.x << ","
                     << _msgTruckVelocity.point.y << ","       // truck local position
                     << _msgTargetLocalPosition.point.x << ","
                     << _msgTargetLocalPosition.point.y << ","
                     << _msgTargetLocalPosition.point.z << ","   // target local position
                     << _msgFusedTargetPosition.point.x << ","
                     << _msgFusedTargetPosition.point.y << ","
                     << _msgFusedTargetPosition.point.z << ","  
                     << _msgFusedTargetVelocity.point.x << ","
                     << _msgFusedTargetVelocity.point.y << ","
                     << _msgFusedTargetVelocity.point.z << "," 
                     << _msgRealTruckLocalPosition.point.x << ","
                     << _msgRealTruckLocalPosition.point.y << "," 
                     << _msgRealTruckLocalPosition.point.z << ","                     
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
                     << yaw << ","
                     << std::endl;   

    _FusedTargetLocalPositionPub.publish(_msgFusedTargetPosition);
}

void navigationTaskCallback(const std_msgs::UInt16 msgNavigationTask)
{
  if (!_bIsStartSim)
  {_nNavigationTask = msgNavigationTask.data;}
  
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
            ROS_INFO_STREAM("Autonomous Tracking and Landing.");
            break;

        case 22:
            ROS_INFO_STREAM("Autonomous Tracking and Landing Two.");
            break;

        case 23:
            ROS_INFO_STREAM("Autonomous Tracking and Landing Three. ");
            break;

        case 24:
            ROS_INFO_STREAM("Search for Target. ");
            break;

        case 25:
            ROS_INFO_STREAM("Go to Truck GPS location. ");
            break;

        case 31:
            ROS_INFO_STREAM("Ultrasonic Test.");
            break;

        case 32:
            ROS_INFO_STREAM("Landing Test.");
            _bIsDroneLandingPrinted = false;
            break;

        case 33:
            ROS_INFO_STREAM("Landing Test plus.");
            break;

        case 34:
            ROS_INFO_STREAM("Temporary Test - Not implemented.");
            break;

        case 98:
            ROS_INFO_STREAM("Stopped Current Mission.");

            // Reset search center
            _bIsSearchInitiated = false;
            _bIsLandingInitiated = false;
            //~ ROS_INFO_STREAM("Search center reinitialized.");

            // Go back to Truck GPS location. RunAutonomousLanding3
            _IsOnTruckTop = false;
            //~ ROS_INFO_STREAM("Time to go back to Truck GPS location.");
            
            // Reset MPC integrator sum position error
            _mpc.IsXpInitialized_ = false;
            
            _sumPosErrX = 0;
            _sumPosErrY = 0;
            //~ ROS_INFO_STREAM("Reset MPC integrator sum position error.");
            
            break;

        default: // It will take care of invalid inputs
            break;
    }

}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);

    return buf;
}


void InitializeLogFiles(float mpc_q, float mpc_kiPos, float mpc_kiVec){
    // Log files
    // std::stringstream ss;
    // ss << DEFAULT_TARGET_TRACKING_LOG_FILE_NAME << ros::WallTime::now() << ".log";
    // _ofsTragetTrackingLog.open(ss.str());
    // ROS_ASSERT_MSG(_ofsTragetTrackingLog, "Failed to open file %s", ss.str().c_str());
    // _ofsTragetTrackingLog << "#Time,TagDistance(x,y,z),TargetDistance(x,y,z),TargetLocalPosition(x,y,z)" << std::endl;

    //~ ss.str("");
    std::stringstream ss;
    ss << DEFAULT_GO_TO_TRUCK_LOG_FILE_NAME  << currentDateTime() << ".log";
    _ofsGoToTruckLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsGoToTruckLog, "Failed to open file %s", ss.str().c_str());
    _ofsGoToTruckLog << "#Time,UltrasonicDistance,UltrasonicReliability,TargetDistance,TruckLocalPosition(x,y,z),TargetLocalPosition(x,y,z)" << std::endl;

    ss.str("");
    ss << DEFAULT_AUTONOMOUS_LANDING_LOG_FILE_NAME   << currentDateTime() << ".log";
    _ofsAutonomousLandingLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsAutonomousLandingLog, "Failed to open file %s", ss.str().c_str());
    _ofsAutonomousLandingLog << "#Time,UltrasonicDistance,UltrasonicReliability,TargetDistance,TruckLocalPosition(x,y,z),TargetLocalPosition(x,y,z)" << std::endl;

    ss.str("");
    ss << DEFAULT_SEAECHING_RANGE_LOG_FILE_NAME << currentDateTime() << ".log";
    _ofsSearchingRangeLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsSearchingRangeLog, "Failed to open file %s", ss.str().c_str());
    _ofsSearchingRangeLog << "#Time,DroneLocation(x,y,z,yaw), GimbalAngle(yaw, pitch), DroneVelocity(vx, vy, vz)" << std::endl;

    // Log about MPC controller
    ss.str("");
    ss << DEFAULT_MPC_CONTROLLER_LOG_FILE_NAME << currentDateTime() << "_" << _mpc.q_ << "_" << _mpc.kiPos_ << ".log";
    _ofsMPCControllerLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsMPCControllerLog, "Failed to open file %s", ss.str().c_str());
    
    // Log about KalmanFilter
    ss.str("");
    ss << DEFAULT_KALMAN_FILTER_LOG_FILE_NAME << currentDateTime() << "_" << _ekf.stepsAhead_ << ".log";
    _ofsKalmanFilterLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsKalmanFilterLog, "Failed to open file %s", ss.str().c_str());

    // Log about everything all the journey
    ss.str("");
    ss << DEFAULT_FULL_JOURNEY_LOG_FILE_NAME << currentDateTime() << ".log";
    _ofsFullJourneyLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsFullJourneyLog, "Failed to open file %s", ss.str().c_str());
}

void LoadNodeSettings(ros::NodeHandle nh){
    
    // Load switches settings
    bool bIsSwitchDefault;
    nh.getParam("/NavigationSwitches/defaultSetting", bIsSwitchDefault);
    if(!bIsSwitchDefault){
        // if not default, then redefine the settings.        
        nh.getParam("/NavigationSwitches/bIsSimulation",              _bIsSimulation);
        nh.getParam("/NavigationSwitches/bIsIntegralEnable",          _bIsIntegralEnable);
        nh.getParam("/NavigationSwitches/bIsYawControlEnable",        _bIsYawControlEnable);
        nh.getParam("/NavigationSwitches/bIsYawControlAlwaysAlign",   _bIsYawControlAlwaysAlign);
        nh.getParam("/NavigationSwitches/bIsYawControlEnableSearch",  _bIsYawControlEnableSearch);
        nh.getParam("/NavigationSwitches/bIsMPCEnable",               _bIsMPCEnable);
        nh.getParam("/NavigationSwitches/bIsLQREnable",               _bIsLQREnable);
        nh.getParam("/NavigationSwitches/bIsKeepLanding",             _bIsKeepLanding);
        nh.getParam("/NavigationSwitches/bIsClearIntegratorError",    _bIsClearIntegratorError); 
        nh.getParam("/NavigationSwitches/bIsEKFEnable",               _bIsEKFEnable); 
        nh.getParam("/NavigationSwitches/cutoffThreshold",            _cutoffThreshold); 
    }

//~ #ifndef EKF_DEBUG
    if(!_bIsEKFEnable){
        bool bIsKFSettingDefault;
        nh.getParam("/KFParameters/defaultSetting", bIsKFSettingDefault);
        if(!bIsKFSettingDefault){
            // if not default, then redefine the settings.        
            nh.getParam("/KFParameters/sigma_ax", _kf.sigma_ax_);
            nh.getParam("/KFParameters/sigma_ay", _kf.sigma_ax_);
            nh.getParam("/KFParameters/sigma_GPSpx", _kf.sigma_GPSpx_);
            nh.getParam("/KFParameters/sigma_GPSpy", _kf.sigma_GPSpy_);
            nh.getParam("/KFParameters/sigma_GPSvx", _kf.sigma_GPSvx_);
            nh.getParam("/KFParameters/sigma_GPSvy", _kf.sigma_GPSvy_);
            nh.getParam("/KFParameters/sigma_Apriltagpx", _kf.sigma_Apriltagpx_);
            nh.getParam("/KFParameters/sigma_Apriltagpy", _kf.sigma_Apriltagpy_);
            nh.getParam("/KFParameters/nPred", _kf.nPred_);
        }
        ROS_INFO("KF enabled!");
    }
    else
    {
        bool bIsEKFSettingDefault;
        nh.getParam("/EKFParameters/defaultSetting", bIsEKFSettingDefault);
        if(!bIsEKFSettingDefault){
            // if not default, then redefine the settings.        
            nh.getParam("/EKFParameters/sigma_a", _ekf.sigma_a_);
            nh.getParam("/EKFParameters/sigma_w", _ekf.sigma_w_);
            nh.getParam("/EKFParameters/sigma_GPSpx", _ekf.sigma_GPSpx_);
            nh.getParam("/EKFParameters/sigma_GPSpy", _ekf.sigma_GPSpy_);
            nh.getParam("/EKFParameters/sigma_GPSvx", _ekf.sigma_GPSvx_);
            nh.getParam("/EKFParameters/sigma_GPSvy", _ekf.sigma_GPSvy_);
            nh.getParam("/EKFParameters/sigma_Apriltagpx", _ekf.sigma_Apriltagpx_);
            nh.getParam("/EKFParameters/sigma_Apriltagpy", _ekf.sigma_Apriltagpy_);
            nh.getParam("/EKFParameters/nPred", _ekf.stepsAhead_);
        }
        ROS_INFO("Extended KF enabled!");
    }

//~ #endif

    bool bIsMPCSettingDefault;
    nh.getParam("/MPCParameters/defaultSetting", bIsMPCSettingDefault);
    if(!bIsMPCSettingDefault){
        // if not default, then redefine the settings.        
        nh.getParam("/MPCParameters/P", _mpc.P_);
        nh.getParam("/MPCParameters/M", _mpc.M_);
        nh.getParam("/MPCParameters/q", _mpc.q_);
        nh.getParam("/MPCParameters/kiPos", _mpc.kiPos_);
        nh.getParam("/MPCParameters/kiVec", _mpc.kiVec_);
        nh.getParam("/MPCParameters/Qk", _mpc.Qk_);
        nh.getParam("/MPCParameters/Qf", _mpc.Qf_);
        nh.getParam("/MPCParameters/Qb", _mpc.Qb_);
    }
    
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");

    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);

    // Initialize global variables
    _ptrDrone = new DJIDrone(nh);

    // Initialize MPC controller
    float mpc_q = argc > 1 ? atof(argv[1]) : -1.0;
    float mpc_kiPos = argc > 2 ? atof(argv[2]) : -1.0;
    float mpc_kiVec = argc > 3 ? atof(argv[3]) : -1.0;
    _mpc.Initialize(mpc_q, mpc_kiPos, mpc_kiVec);
    
    
    
    solveQP_initialize();
    
    LoadNodeSettings(nh);
    
    
    _kf.Initialize();
    _ekf.Initialize();

    

    // Log files
    InitializeLogFiles(mpc_q, mpc_kiPos, mpc_kiVec);
    
  
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


    ROS_INFO_STREAM("Navigation has started.");

    ros::spin();

    return 0;

}
