#include "ros/ros.h"
#include <dji_sdk/dji_drone.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <navigation/conversion.h>
#include <navigation/UasMath.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
//~ #include <apriltags2_ros/AprilTagDetection.h>
//~ #include <apriltags2_ros/AprilTagDetectionArray.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <queue>

#include <string>
#include <stdio.h>
#include <time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "navigation/MPCController.h"
#include "navigation/KalmanFilter.h"

#include <math.h>
#include <nlopt.hpp>
#include <vector>

#include "navigation/rt_nonfinite.h" 
#include "navigation/solveQP.h"
#include "navigation/solveQP_terminate.h"
#include "navigation/solveQP_emxAPI.h"
#include "navigation/solveQP_initialize.h"
#include <stddef.h>
#include <stdlib.h>
#include "navigation/rtwtypes.h"
#include "navigation/solveQP_types.h"

using namespace std;
using namespace Eigen;
using namespace nlopt;

//~ #define DEFAULT_TARGET_TRACKING_LOG_FILE_NAME "/home/ubuntu/TargetTracking_"
#define DEFAULT_GO_TO_TRUCK_LOG_FILE_NAME "/home/ubuntu/GoToTruck_"
#define DEFAULT_AUTONOMOUS_LANDING_LOG_FILE_NAME "/home/ubuntu/AutonomousLanding_"
#define DEFAULT_SEAECHING_RANGE_LOG_FILE_NAME "/home/ubuntu/SearchRange_"
#define DEFAULT_MPC_CONTROLLER_LOG_FILE_NAME "/home/ubuntu/MPCController_"
#define DEFAULT_KALMAN_FILTER_LOG_FILE_NAME "/home/ubuntu/KalmanFilter_"
#define TARGET_LOST_TIME_OUT_SEC (2.0)

// Searching path parameters
// Cont sin path
#define OMEGA (3.1415926)
#define HEIGHT (2.0)
#define SPEED (0.5)
// spiral path
#define LINE_VELOCITY (0.3)
#define ANGLE_VELOCITY (0.8)  //  rad/s
// common parameters
#define SEARCH_ALTITUDE (3.0)
#define HEIGHT_ERROR (0.2)  // tolerable height error
#define SEARCH_TIME (360.0)
// Gimbal searching 
#define YAW_RANGE (80.0) //degrees
#define RATIO_GIMBAL (10.0) // period time second


// PD controller parameters in Landing
#define KP (1.0)   // set 0.7 for real flight test
#define KD (0.0)   // set 0.05 for real flight test


// LQR integrator
#define KI (0.03)
#define DT (0.08)

DJIDrone* _ptrDrone;

int _nNavigationTask = 0;
bool _bIsDroneLandingPrinted = false;

// Switches
bool _bIsIntegralEnable = true;
bool _bIsYawControlEnable = false;
bool _bIsMPCEnable = true; 
bool _bIsLQREnable = false;
bool _bIsSimulation = true;
bool _bIsYawControlEnableSearch = false;
bool _bIsLocalLocationControlEnable = false;
bool _bIsStartSim = false;
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

//~ // LQR controller
//~ float _lqrGain[8] = {-2,0,-5.69,0,-2,0,-5.69,0};
//~ float _lqrGain[8] = {-3,0,-8.5189388,0,-3,0,-8.5189388,0}; // 50m 11s overshoot < 0.3m
//~ float _lqrGain[8] = {-2.38047614,0,-6.33407922,0,-2.38047614,0,-6.33407922,0}; // 50m 9s overshoot < 0.3m
float _lqrGain[8] = {-1.52752523,0,-4.07731867,0,-1.52752523,0,-4.07731867,0}; // 50m 11s overshoot < 2.8m ki 0.03 dt 0.08
//~ float _lqrGain[8] = {-1.52752523,0,-4.07731867,0,-1.52752523,0,-4.07731867,0};


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

// Kalman Filter
KalmanFilter _kf;
Vector4d _truckEstmState;
bool _IsGPSUpdated = false;


// PD controller
float _error = 0;
float _error_last = 0;

// LQR integrator
float _integral_x = 0;
float _integral_y = 0;



// Data Record
// std::ofstream _ofsTragetTrackingLog;
std::ofstream _ofsGoToTruckLog;
std::ofstream _ofsAutonomousLandingLog;
std::ofstream _ofsSearchingRangeLog;
std::ofstream _ofsMPCControllerLog;
std::ofstream _ofsKalmanFilterLog;


// GPS & Camera data funsion
std::queue <dji_sdk::Gimbal> _queMsgGimbal;
std::deque <geometry_msgs::PointStamped> _queMsgTruckLocalPosition;
std::deque <geometry_msgs::PointStamped> _queMsgTargetLocalPosition;
geometry_msgs::PointStamped _msgFusedTargetLocalPosition;

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
 void quaternionToRPY(dji_sdk::AttitudeQuaternion q, double & roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
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
    //~ output = current_position - output;
    _error_last = _error;
    return output;
}

float LocalPositionControlHelper(float desired, float current_position)
{

    float error = desired - current_position;
    float abs_error = abs(error);

    // return (error < 0.4) ? desired
    //                      : (error < 5) ? current_position + error * 0.35
    //                                    : current_position + error * 0.5;

    //ROS_INFO("error: %f", error);

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
		
				float error = desired - _msgUltraSonic.ranges[0];
				desired = desired + error;
				float setpoint_z = PDController(desired, current_position) + current_position;
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


float AttitudeControlHelper(geometry_msgs::Point desired_position, float& dpitch, float& droll)
{

    DJIDrone& drone = *_ptrDrone;

    float error_position_x = drone.local_position.x - desired_position.x;
    float error_velocity_x = drone.velocity.vx - _msgTruckVelocity.point.x;

    float error_position_y = drone.local_position.y - desired_position.y;
    float error_velocity_y = drone.velocity.vy - _msgTruckVelocity.point.y;

    if((error_position_x >= 0 && _lastPosErrX >= 0) || (error_position_x < 0 && _lastPosErrX < 0))
    {
		_integral_x += (_lqrGain[0]*error_position_x * DT + _lqrGain[2]*error_velocity_x * DT);
		
    }
    else
    {
		_integral_x = 0;
	}
	
	if((error_position_y >= 0 && _lastPosErrY >= 0) || (error_position_y < 0 && _lastPosErrY < 0))
    {
		_integral_y += (_lqrGain[5]*error_position_y * DT + _lqrGain[7]*error_velocity_y * DT);
		
    }
    else
    {
		_integral_y = 0;
	}
		
    double Iout_pitch = _bIsIntegralEnable ? (KI * _integral_x) : 0;
    double Iout_roll = _bIsIntegralEnable ? (KI * _integral_y) : 0;

    dpitch = _lqrGain[0] * error_position_x + _lqrGain[2] * error_velocity_x + Iout_pitch;
    droll = _lqrGain[5] * error_position_y + _lqrGain[7] * error_velocity_y + Iout_roll;

    _lastPosErrX = error_position_x;
    _lastPosErrY = error_position_y;


    //~ dpitch = _lqrGain[0] * error_position_x + _lqrGain[2] * error_velocity_x;
    //~ droll = _lqrGain[5] * error_position_y + _lqrGain[7] * error_velocity_y;

    // Saturate desired pitch and roll angle to -30deg or 30deg
    float maxAngle = 30.0;
    dpitch = dpitch > maxAngle ? maxAngle
                               : dpitch < -maxAngle ? -maxAngle
                                                    : dpitch;

    droll = droll > maxAngle ? maxAngle 
                             : droll < -maxAngle ? -maxAngle
                                                 : droll;

    ROS_INFO(" error_px, error_py, dpitch, droll: %f, %f, %f, %f ", error_position_x, error_position_y, dpitch, droll);

}

float IntegratorCalculationPos(float stateError, float lastStateError, float sumError)
{
	float limition = 85;
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
		sumError += stateError*DT;}
		//~ sumError = 0;}
	//~ }
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

float AttitudeControlHelper3(geometry_msgs::Point desired_position, float& dpitch, float& droll)
{
	DJIDrone& drone = *_ptrDrone;
	Vector3d u(desired_position.x - drone.local_position.x, desired_position.y - drone.local_position.y, 0);
	Vector3d u_dot(_msgTruckVelocity.point.x - drone.velocity.vx, _msgTruckVelocity.point.y - drone.velocity.vy, 0);
    
    float lamda = 4;
    float kp = 1;
    float kd = 2;
    float m = 2.883;
    float g = 9.8;
    
    Vector3d Omiga = (u.cross(u_dot))/(u.dot(u));
    Vector3d aNor = - lamda * u_dot.norm() * (u/u.norm()).cross(Omiga);
    Vector3d aTan = u*kp + u_dot*kd;
    Vector3d a = aTan + aNor;
    
    dpitch = atan(a(0)/g + 0.705*drone.velocity.vx*fabs(drone.velocity.vx));
    droll = atan((a(1)/g + 0.705*drone.velocity.vy*fabs(drone.velocity.vy)*cos(dpitch)));
	
	float maxAngle = 30.0;
    dpitch = dpitch > maxAngle ? maxAngle
                               : dpitch < -maxAngle ? -maxAngle
                                                    : dpitch;

    droll = droll > maxAngle ? maxAngle
                             : droll < -maxAngle ? -maxAngle
                                                 : droll;
                                                 
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    ROS_INFO(" error_px, error_py, dpitch, droll:%f, %f, %f, %f",u(0),u(1), dpitch, droll); // -uk(0), -uk(1));
    _ofsMPCControllerLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            << ros::Time::now().toSec() << ","
                            <<  _msgUltraSonic.ranges[0] << ","
                            << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                            << _msgTruckLocalPosition.point.x << ","
                            << _msgTruckLocalPosition.point.y << ","
                            << _msgTruckLocalPosition.point.z << ","   // truck local position
                            << _msgTargetLocalPosition.point.x << ","
                            << _msgTargetLocalPosition.point.y << ","
                            << _msgTargetLocalPosition.point.z << ","   // target local position
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
                            << yaw << std::endl;  
	}


//~ double CostFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data){
	//~ 
	//~ int i;
	//~ 
	//~ // Cost function
	//~ // Um(R+BpQBp)Um + 2(xkApQBp - rpQBp)Um
	//~ VectorXd Um = MatrixXd::Zero(_mpc.M_*_mpc.nu_, 1);
	//~ for(i = 0; i < _mpc.M_*_mpc.nu_; i++){
		//~ Um(i) = x[i];
	//~ }
	//~ 
	//~ double cost;
	//~ cost = Um.transpose()*_mpc.K1_*Um ;//+ 2*_mpc.xk_.transpose()*_mpc.K2_*Um - 2*_mpc.rp_.transpose()*_mpc.K3_*Um;
	//~ 
	//~ // Gradient function
	//~ VectorXd gradv = 2*_mpc.K1_*Um + 2*(_mpc.xk_.transpose()*_mpc.K2_ - _mpc.rp_.transpose()*_mpc.K3_);
	//~ if (!grad.empty()) {
		//~ for ( i = 0; i < _mpc.M_; i++){
			//~ grad[i] = gradv(i);
		//~ }
	//~ }
		//~ 
		//~ 
	//~ return cost;
		//~ 
//~ }
//~ 
//~ 
//~ typedef struct {
	//~ int sign;
	//~ int index;
//~ } my_constraints_data;
//~ 
//~ 
//~ double MyConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data){
	//~ 
	//~ my_constraints_data *d  = reinterpret_cast<my_constraints_data *> (data);
	//~ int sign = d->sign;
	//~ int index = d->index;
	//~ 
	//~ int i;
	//~ 
	//~ VectorXd Um = MatrixXd::Zero(_mpc.M_*_mpc.nu_, 1);
	//~ for( i = 0; i < _mpc.M_; i ++){
		//~ Um(i) = x[i];
	//~ }
	//~ 
	//~ double constraints = sign*(_mpc.Bp_*Um)(index) - 17 + sign*(_mpc.Ap_*_mpc.xk_)(index);
	//~ 
	//~ // gradient function
	//~ if(!grad.empty()){
		//~ for(i = 0; i<_mpc.nu_*_mpc.M_ ; i++){
			//~ grad[i] = sign*_mpc.Bp_(index, i); 
		//~ }
	//~ }
	//~ 
	//~ return constraints;
//~ }


//~ Vector2d ComputeOptimalInput2(){
	

	
	// Initialize
	//~ int i;
	//~ int n = _mpc.nu_*_mpc.M_;
    //~ nlopt::opt opt(nlopt::LD_MMA, n);
    
    
    // boundaries
    //~ std::vector<double> lb(_mpc.nu_*_mpc.M_);
    //~ std::vector<double> hb(_mpc.nu_*_mpc.M_);
    //~ for ( i = 0; i<_mpc.nu_*_mpc.M_ ; i++){
		//~ lb[i] = -30;
		//~ hb[i] = 30;
	//~ }
    //~ opt.set_lower_bounds(lb);
    //~ opt.set_upper_bounds(hb);
    
    //~ opt.set_min_objective(CostFunc, NULL); 
    //~ 
    //~ 
    //~ // inequality constraints
    //~ my_constraints_data data[_mpc.P_*_mpc.nx_/2];
    //~ 
    //~ for ( i = 0; i<_mpc.P_*_mpc.nx_/2 ; i++){
		//~ (&data[i])->sign = i < _mpc.P_ ? 1 : -1;
		//~ (&data[i])->index = i < _mpc.P_ ? 4*i+2 : 4*(i-_mpc.P_)+3;
	//~ }
	//~ 
	//~ for ( i = 0; i<_mpc.P_*_mpc.nx_/2 ; i++){
		//~ opt.add_inequality_constraint(MyConstraints, &data[i], 1e-8);
	//~ }
	
	//~ // settings
	//~ opt.set_xtol_rel(1e-2);
	//~ std::vector<double> x(_mpc.nu_*_mpc.M_);
	//~ for ( i = 0; i<_mpc.nu_*_mpc.M_ ; i++){
		//~ x[i] = 1;
	//~ }
	//~ 
	//~ // Start optimize
	//~ double minf;
	//~ 
	//~ nlopt::result result = opt.optimize(x, minf);
	//~ std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
			  //~ << std::setprecision(10) << minf << std::endl;
//~ 
	//~ try{
		//~ nlopt::result result = opt.optimize(x, minf);
		//~ std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
			//~ << std::setprecision(10) << minf << std::endl;
	//~ }
	//~ catch(std::exception &e) {
		//~ std::cout << "nlopt failed: " << e.what() << std::endl;
	//~ }
	//~ 
	//~ _mpc.uk_(0) = x[0];
	//~ _mpc.uk_(1) = x[1];
		 
    
    //~ return _mpc.uk_;
//~ }



	
/* Function Definitions */
static void argInit_4x1_real_T(double result[4], Vector4d xk)
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 4; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = xk(idx0);
  }
}

static emxArray_real_T *argInit_Unboundedx1_real_T(VectorXd desiredState)
{
  emxArray_real_T *result;
  static int iv2[1] = { 48 };

  int idx0;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreateND_real_T(1, iv2);

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0 = idx0 + 4) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
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
    Vector4d targetState(desired_position.x, desired_position.y, _msgTruckVelocity.point.x, _msgTruckVelocity.point.y);
    Vector4d stateError = xk - targetState;
    _mpc.SetXpInitialPoint(xk);
    
    
    int nPred = 25; 
    _kf.SetPredHorizon(nPred + P);
    
    
    // predict the target position and velocity here, KF
    // First decide now we use prediction or estimation, based whether there exists observations or not
    if(!_IsGPSUpdated){
		_truckEstmState = _kf.PredictWOObservation();
	}
	_IsGPSUpdated = false;
	
    VectorXd truckPred = _kf.Predict(_truckEstmState);
    VectorXd desiredState = truckPred.tail(P*nx);
    //~ _mpc.xk_ = xk;
    //~ _mpc.rp_ = desiredState;
    

    // Start MPC calculation each loop
    VectorXd Xpc = _mpc.CorrectPrediction(xk);
    //~ std::cout << "xk: " << xk.transpose() << std::endl;
    //~ std::cout << "Xpc: " << Xpc.transpose()  << std::endl;
     
    //~ VectorXd statePredError = Xpc - targetState.colwise().replicate(P); // For stationary target
    VectorXd statePredError = Xpc - desiredState;   // For moving target
    //~ Vector2d uk = _mpc.ComputeOptimalInput(statePredError);
    //~ Vector2d uk = _mpc.ComputeOptimalInput2(xk, desiredState);
    
    // test
    //~ xk = MatrixXd::Zero(4,1);
    //~ VectorXd a(100,100,0,0);
    //~ desiredState = a.colwise().replicate(P);
    
    
    double xkarray[4];
    //~ double rp[80];
    emxArray_real_T *rp;
    double x_data[10];
    int x_size[1];
    //~ xk = {0,0,0,0};
    /* Initialize function 'solveQP' input arguments. */
    /* Initialize function input argument 'xk'. */
    argInit_4x1_real_T(xkarray, xk);

    /* Initialize function input argument 'rp'. */
    rp = argInit_Unboundedx1_real_T(desiredState);
    //~ rp = {100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0};
    /* Call the entry-point 'solveQP'. */
    solveQP(xkarray, rp, x_data, x_size); 
    emxDestroyArray_real_T(rp); 
    VectorXd um = MatrixXd::Zero(10,1);
    for( int i = 0; i<2 ;i++){
        um(i) = x_data[i];
    }
    
    Vector2d uk = um.head(2);
    _mpc.Um_ = um;
  
    VectorXd Xp = _mpc.Predict(xk);
    
    std::cout << um << endl;

    // Initialize integrator
    _sumPosErrX = IntegratorCalculationPos(stateError(0)*1.65, _lastPosErrX, _sumPosErrX);
    _sumPosErrY = IntegratorCalculationPos(stateError(1)*1.65, _lastPosErrY, _sumPosErrY);
    _sumVecErrX = IntegratorCalculationVec(stateError(2), _lastVecErrX, _sumVecErrX);
    _sumVecErrY = IntegratorCalculationVec(stateError(3), _lastVecErrY, _sumVecErrY);
        
    _lastPosErrX = stateError(0);
    _lastPosErrY = stateError(1);
    _lastVecErrX = stateError(2);
    _lastVecErrY = stateError(3);
    
    std::cout << "SumPosX, SumPosY, q, ki:  " << _sumPosErrX << ", " << _sumPosErrY << ", " << _mpc.q_ << ", " << mpc_kiPos << endl;

    dpitch = _bIsIntegralEnable ? (-uk(0) - mpc_kiPos*_sumPosErrX -mpc_kiVec*_sumVecErrX): -uk(0);
    droll = _bIsIntegralEnable? (-uk(1) - mpc_kiPos*_sumPosErrY - mpc_kiVec*_sumVecErrY): -uk(1);
    //~ dpitch = -uk(0);
    //~ droll = -uk(1);

    // Saturate desired pitch and roll angle to -30deg or 30deg
    float maxAngle = 35.0; 
    dpitch = dpitch > maxAngle ? maxAngle
                               : dpitch < -maxAngle ? -maxAngle
                                                    : dpitch;

    droll = droll > maxAngle ? maxAngle
                             : droll < -maxAngle ? -maxAngle
                                                 : droll;
	
    ROS_INFO(" error_px, error_py, dpitch, droll: %f, %f, %f, %f, %f, %f", drone.local_position.x - _msgRealTruckLocalPosition.point.x, drone.local_position.y - _msgRealTruckLocalPosition.point.y, dpitch, droll, -uk(0), -uk(1));
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );

    _ofsMPCControllerLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            << ros::Time::now().toSec() << ","
                            <<  _msgUltraSonic.ranges[0] << ","
                            << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                            << _msgTruckLocalPosition.point.x << ","
                            << _msgTruckLocalPosition.point.y << ","
                            << _msgTruckLocalPosition.point.z << "," 
                            << _msgRealTruckLocalPosition.point.x << "," 
                            << _msgRealTruckLocalPosition.point.y << "," 
                            << _msgRealTruckLocalPosition.point.z << "," 
                            << _msgTruckVelocity.point.x << ","
							<< _msgTruckVelocity.point.y << "," // truck local position
                            << _msgTargetLocalPosition.point.x << ","
                            << _msgTargetLocalPosition.point.y << ","
                            << _msgTargetLocalPosition.point.z << ","   // target local position
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
                            << yaw << std::endl;                             // drone local position
	
	//~ _ofsKalmanFilterLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            //~ << ros::Time::now().toSec() << ","							
                            //~ << _msgTruckLocalPosition.point.x << ","
                            //~ << _msgTruckLocalPosition.point.y << ","
                            //~ << _msgTruckLocalPosition.point.z << ","  // truck local position, mey be noisy
                            //~ << _msgTruckVelocity.point.x << ","
							//~ << _msgTruckVelocity.point.y << ","
							//~ << _msgRealTruckLocalPosition.point.x << ","
							//~ << _msgRealTruckLocalPosition.point.y << ","
							//~ << _msgRealTruckLocalPosition.point.z << "," // truck true local position 
							//~ << _truckEstmState.transpose() << ","  // truck estimated position & velocity
							//~ << truckPred.segment((nPred+1)*nx,4).transpose() << std::endl;  
							
							
}



void RunAttitudeControl(geometry_msgs::Point desired_position, float desired_yaw_deg){

    DJIDrone& drone = *_ptrDrone;

    float setAnglePitch = 0;
    float setAngleRoll  = 0;

    if(_bIsMPCEnable){
		
        AttitudeControlHelper2(desired_position, setAnglePitch, setAngleRoll);  
        
    }
    else if (_bIsLQREnable){
        AttitudeControlHelper(desired_position, setAnglePitch, setAngleRoll);
    }
    else{
		AttitudeControlHelper3(desired_position, setAnglePitch, setAngleRoll);
		}
		
    float setpoint_z = LocalPositionControlAltitudeHelper(desired_position.z, drone.local_position.z);
	
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float current_yaw_deg = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float yaw_error_deg = desired_yaw_deg - current_yaw_deg;
	float abs_yaw_error_deg = abs(yaw_error_deg);
    float setpoint_yaw = (abs_yaw_error_deg < 3) ? desired_yaw_deg
                                             : (abs_yaw_error_deg < 10) ? current_yaw_deg + yaw_error_deg * 0.3
                                             : current_yaw_deg + yaw_error_deg * 0.2;
    //~ ROS_INFO("Current yaw angle, yaw error, setpoint_yaw:%f,%f,%f", current_yaw_deg, yaw_error_deg, setpoint_yaw);
    
    if (_bIsYawControlEnable){
		drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);
	}
	else{
		setpoint_yaw = 0;
	    drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);
	}

    // Notice: negative pitch angle means going to North; positive pitch means South.
    //  drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);
    //~ drone.attitude_control(0x10, setAngleRoll, -setAnglePitch, setpoint_z, setpoint_yaw);


    // Just record the system input.
    _msgDesiredAttitudeDeg.point.x = setAngleRoll;
    _msgDesiredAttitudeDeg.point.y = -setAnglePitch;
    _msgDesiredAttitudeDeg.point.z = 0;

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

    //~ geometry_msgs::Point desired_position;
//~ 
    //~ desired_position.x = 0;
    //~ desired_position.y = 0;
    //~ desired_position.z = 3;
	//~ float start = 0;
	//~ if (_counter < 300)
	//~ {
		//~ RunAttitudeControl(desired_position, 0);
		//~ drone.attitude_control(0x10, 0, 0, 3, 0);
		//~ ROS_INFO("turning to clockwise!!");
		//~ }
	//~ else
	//~ {
		//~ RunAttitudeControl(desired_position, 179);
		//~ drone.attitude_control(0x10, 0, 0, 3, 179);
		//~ ROS_INFO("turning to counterclockwise!!");
		//~ start = 1;
		//~ }	
		//~ 
	//~ _counter += 1; 
	//~ dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    //~ float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    //~ ROS_INFO("Current yaw: %f", yaw);
	    //~ _ofsGoToTruckLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                      //~ << ros::Time::now().toSec() << ","
                      //~ << start << ","
                      //~ << yaw  << std::endl; 
                      
     drone.local_position_control(-50, -400, 3, 0);
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
	ROS_INFO("Publish desired gimbal angle!! ");
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


    // Sensor fusion.
    // Pushed into queue back
    if (_queMsgTargetLocalPosition.size() > 29){
        _queMsgTargetLocalPosition.pop_front();
    }
    _queMsgTargetLocalPosition.push_back(_msgTargetLocalPosition);
 

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


void RunSensorFusing(){

    float truckSumX=0, truckSumY=0;
    float truckAvgX=0, truckAvgY=0;

    float targetSumX=0, targetSumY=0;
    float targetAvgX=0, targetAvgY=0;

    // Calculate the mean of Truck local position
    bool IsGPSLocationEmpty = _queMsgTruckLocalPosition.size() < 1;
    if (!IsGPSLocationEmpty){
        for(int i=0; i < _queMsgTruckLocalPosition.size(); i++){
            truckSumX += _queMsgTruckLocalPosition.at(i).point.x;
            truckSumY += _queMsgTruckLocalPosition.at(i).point.y;
        }
        truckAvgX = truckSumX/_queMsgTruckLocalPosition.size();
        truckAvgY = truckSumY/_queMsgTruckLocalPosition.size();
    }

    // Calculate the mean of Target local position
    bool IsTagDetectionEmpty = _queMsgTargetLocalPosition.size() < 1;
    if (!IsTagDetectionEmpty){
        for(int i=0; i < _queMsgTargetLocalPosition.size(); i++){
            targetSumX += _queMsgTargetLocalPosition.at(i).point.x;
            targetSumY += _queMsgTargetLocalPosition.at(i).point.y;
        }
        targetAvgX = targetSumX/_queMsgTargetLocalPosition.size();
        targetAvgY = targetSumY/_queMsgTargetLocalPosition.size();
    }

    // Fuse data
    // Idea is if no GPS and tag detections, then fused position is 0
    // if only GPS or tag detections available, then his factor is 1;
    // if both are available, then fuse w/ ratio 3:1
    float truckFuseFactor = IsGPSLocationEmpty  ? 0
                                                : IsTagDetectionEmpty   ? 1
                                                                        : 0.25;
    float targetFuseFactor = IsTagDetectionEmpty ? 0 : 0.75;
    _msgFusedTargetLocalPosition.header.stamp = ros::Time::now();
    _msgFusedTargetLocalPosition.point.x = truckAvgX*truckFuseFactor + targetAvgX*targetFuseFactor;
    _msgFusedTargetLocalPosition.point.y = truckAvgY*truckFuseFactor + targetAvgY*targetFuseFactor;
    _msgFusedTargetLocalPosition.point.z = 0;
    _FusedTargetLocalPositionPub.publish(_msgFusedTargetLocalPosition);

    // DEBUG
    //~ ROS_INFO("Truck AVG = %f, %f , %d " , truckAvgX, truckAvgY , _queMsgTruckLocalPosition.size());
    //~ ROS_INFO("Target AVG = %f, %f , %d ", targetAvgX , targetAvgY, _queMsgTargetLocalPosition.size());
    //~ ROS_INFO("Fused = %f, %f " , _msgFusedTargetLocalPosition.point.x , _msgFusedTargetLocalPosition.point.y);

}



// // This the original tag_detection callback
void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray vecTagDetections)
{
     // If Target Tracking is not running, do nothing.
     if (!_bIsTargetTrackingRunning) { return; }

     FindDesiredGimbalAngle(vecTagDetections);
}

// Experimental: tag_detection callback for apriltag2_ros
//~ void tagDetectionCallback(const apriltags2_ros::AprilTagDetectionArray vecTagDetections2)
//~ {
//~ 
    //~ // If Target Tracking is not running, do nothing.
    //~ if (!_bIsTargetTrackingRunning) { return; }
//~ 
    //~ // If found nothing, do noting.
    //~ if (vecTagDetections2.detections.empty())
    //~ {
        //~ // There is nothing we can do
        //~ return;
    //~ }
//~ 
    //~ // transfer detection format from apriltag2 to apriltag
    //~ // It's just a quick fix. Later need to discard apriltag and use apriltag2
    //~ apriltags_ros::AprilTagDetection TagDetection;
    //~ apriltags_ros::AprilTagDetectionArray vecTagDetections;
    //~ // TagDetection.id            = vecTagDetections2.detections.at(0).id[0];
    //~ // TagDetection.size          = vecTagDetections2.detections.at(0).size[0];
    //~ // TagDetection.pose.header   = vecTagDetections2.detections.at(0).pose.header;
    //~ TagDetection.pose.pose     = vecTagDetections2.detections.at(0).pose.pose.pose;
    //~ vecTagDetections.detections.push_back(TagDetection);
//~ 
    //~ FindDesiredGimbalAngle(vecTagDetections);
//~ 
    //~ // If target found, sensor fuse at 30hz.
    //~ if( _bIsTargetFound ){
        //~ RunSensorFusing();
    //~ }
 //~ }


// void lqrGainCallback(const geometry_msgs::PoseWithCovariance msgLQRGain)
// {
//     for(int i=0; i<8; i++){
//         _lqrGain[i] = msgLQRGain.covariance[i];
//     }
//     //~ ROS_INFO("%f", _lqrGain[0]);
// }

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

    DJIDrone& drone = *_ptrDrone;

    // Record GPS position
    _msgTruckGPSPosition.point.x = msgTruckPosition.pose.position.x;
    _msgTruckGPSPosition.point.y = msgTruckPosition.pose.position.y;
    _msgTruckGPSPosition.point.z = msgTruckPosition.pose.position.z;
    
    float truckDistance_x = (msgTruckPosition.pose.orientation.x - drone.global_position.latitude)/0.0000089354;
    float truckDistance_y = (msgTruckPosition.pose.orientation.y - drone.global_position.longitude)/0.0000121249;


    // Calculate the truck local location
    // drone.local_position.x means northing
    // drone.local_position.y means easting
    _msgRealTruckLocalPosition.header.stamp = ros::Time::now();
    _msgRealTruckLocalPosition.point.x = drone.local_position.x + truckDistance_x;
    _msgRealTruckLocalPosition.point.y = drone.local_position.y + truckDistance_y;
    _msgRealTruckLocalPosition.point.z = 0;

    // Calculate the distance from truck to drone
    _msgTruckDistance.point.x = (msgTruckPosition.pose.position.x - drone.global_position.latitude)/0.0000089354;
    _msgTruckDistance.point.y = (msgTruckPosition.pose.position.y - drone.global_position.longitude)/0.0000121249;
    _msgTruckDistance.point.z = 0;

    // If the Truck is extremely far away from drone, then there must be something wrong
    if( abs(_msgTruckDistance.point.x) > 100 && abs(_msgTruckDistance.point.y) > 100 ) {
        ROS_INFO_STREAM("DANGER: There is something wrong with the Truck GPS or Drone GPS. Plz check NOW!!!!!");
        _bIsTargetTrackingRunning = false;  // mission cancel
        return;
    }

    // Calculate the truck local location
    // drone.local_position.x means northing
    // drone.local_position.y means easting
    _msgTruckLocalPosition.header.stamp = ros::Time::now();
    _msgTruckLocalPosition.point.x = drone.local_position.x + _msgTruckDistance.point.x;
    _msgTruckLocalPosition.point.y = drone.local_position.y + _msgTruckDistance.point.y;
    _msgTruckLocalPosition.point.z = 0;
    _TruckLocalPositionPub.publish(_msgTruckLocalPosition);

    

    // Sensor fusion.
    // Pushed into queue back
    if (_queMsgTruckLocalPosition.size() > 9){
        _queMsgTruckLocalPosition.pop_front();
    }
    _queMsgTruckLocalPosition.push_back(_msgTruckLocalPosition);

    // If target not found, sensor fuse at 10hz.
    if( !_bIsTargetFound ){
        RunSensorFusing();
    }
    
    // Update estimate by kalman filter
	Vector4d truckState(_msgTruckLocalPosition.point.x, _msgTruckLocalPosition.point.y, _msgTruckVelocity.point.x, _msgTruckVelocity.point.y);
	_kf.SetXhatInitialPoint(truckState);
	_truckEstmState = _kf.Update(truckState);
	
	_IsGPSUpdated = true;
			_ofsKalmanFilterLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                            << ros::Time::now().toSec() << ","							
                            << _msgTruckLocalPosition.point.x << ","
                            << _msgTruckLocalPosition.point.y << ","
                            << _msgTruckLocalPosition.point.z << ","  // truck local position, mey be noisy
                            << _msgTruckVelocity.point.x << ","
							<< _msgTruckVelocity.point.y << ","
							<< _msgRealTruckLocalPosition.point.x << ","
							<< _msgRealTruckLocalPosition.point.y << ","
							<< _msgRealTruckLocalPosition.point.z << "," // truck true local position 
							<< _truckEstmState.transpose() << std::endl;   

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
        
        _StartX = _msgFusedTargetLocalPosition.point.x - 5;
        _StartY = _msgFusedTargetLocalPosition.point.y;

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
            
            _FlyingRadius = sqrt((drone.local_position.x - _msgTruckLocalPosition.point.x)*(drone.local_position.x - _msgTruckLocalPosition.point.x) + (drone.local_position.y - _msgTruckLocalPosition.point.y)*(drone.local_position.y - _msgTruckLocalPosition.point.y));
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


void RunTargetTracking()
{
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

void RunTimeCriticalTasks()
{
    if (_bIsTargetTrackingRunning)
    {
        //~ RunTargetSearch();
    }
}


void RunAutonomousLanding()
{

    DJIDrone& drone = *_ptrDrone;

    bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.13) && (int)_msgUltraSonic.intensities[0];
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

    //~ bool bIsDroneLanded = (_msgUltraSonic.ranges[0] < 0.3) && (int)_msgUltraSonic.intensities[0];
    bool bIsDroneLanded = drone.local_position.z < 0.1;
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
    float limitRadius_square = limitRadius*limitRadius;
    
    if (_bIsSimulation)
    {    
		float distance_square = (_msgRealTruckLocalPosition.point.x - drone_x)*(_msgRealTruckLocalPosition.point.x - drone_x) + (_msgRealTruckLocalPosition.point.y - drone_y)*(_msgRealTruckLocalPosition.point.y - drone_y);
        bool bIsClose = distance_square < limitRadius_square;
        bool bIsStartLanding = false;
        if (bIsClose)
        {
			_counter = _counter + 1;
		}
        if ((_counter > 60) && (bIsClose))
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
        if (_bIsLocalLocationControlEnable)
        {	RunLocalPositionControl(desired_position, desired_yaw);}
        else
        {   RunAttitudeControl(desired_position, desired_yaw);}
        //~ RunLocalPositionControl(desired_position, desired_yaw);
        //~ float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckLocalPosition.point.y, _msgTruckLocalPosition.point.x));
    }
    else
    {
        float distance_square = (_msgTargetLocalPosition.point.x - drone_x)*(_msgTargetLocalPosition.point.x - drone_x) + (_msgTargetLocalPosition.point.y - drone_y)*(_msgTargetLocalPosition.point.y - drone_y);
        bool bIsClose = distance_square < limitRadius_square;
        // ROS_INFO("delta x & y: %f, %f",delta_x ,delta_y);
        // ROS_INFO("target x& y :%f, %f",target_x, target_y);
        
        geometry_msgs::Point desired_position;
        desired_position.x = _msgTargetLocalPosition.point.x;
        desired_position.y = _msgTargetLocalPosition.point.y;
        desired_position.z = bIsClose ? -0.1 : drone_z;
        ROS_INFO("desired_position: %f, %f, %f",desired_position.x, desired_position.y, desired_position.z);
        //~ float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTargetDistance.point.y, _msgTargetDistance.point.x));
        float desired_yaw = 0;
        if (_bIsLocalLocationControlEnable)
        {	RunLocalPositionControl(desired_position, desired_yaw);}
        else
        {	RunAttitudeControl(desired_position, desired_yaw);}
        //~ 

    }
    
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float roll = 0;
    float pitch = 0;

    float distance_square = 0;

    _ofsAutonomousLandingLog << std::setprecision(std::numeric_limits<double>::max_digits10)
                     << ros::Time::now().toSec() << ","
                     <<  _msgUltraSonic.ranges[0] << ","
                     << (int)_msgUltraSonic.intensities[0] << "," // ultrasonic
                     << distance_square << ","
                     << _msgTruckLocalPosition.point.x << ","
                     << _msgTruckLocalPosition.point.y << ","
                     << _msgTruckLocalPosition.point.z << ","
                     << _msgTargetLocalPosition.point.x << ","
                     << _msgTargetLocalPosition.point.y << ","
                     << _msgTargetLocalPosition.point.z << ","   // target local position
                     << drone.global_position.latitude << ","
                     << drone.global_position.longitude << ","
                     << drone.local_position.x << ","
                     << drone.local_position.y << ","
                     << drone.local_position.z << ","
                     << drone.velocity.vx << ","
                     << drone.velocity.vy << ","
                     << drone.velocity.vz << ","
                     << _msgDesiredAttitudeDeg.point.x << ","
                     << _msgDesiredAttitudeDeg.point.y << ","
                     << _msgDesiredAttitudeDeg.point.z << ","
                     << roll << ","
                     << pitch << ","
                     << yaw << std::endl;                                // drone local position

}



void GoToTruckGPSLocation()
{
    DJIDrone& drone = *_ptrDrone;


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

    float limitRadius_square = _limitRadius*_limitRadius;
    float distance_square = (   _msgTruckLocalPosition.point.x - drone_x)*(_msgTruckLocalPosition.point.x - drone_x) + (
                                _msgTruckLocalPosition.point.y - drone_y)*(_msgTruckLocalPosition.point.y - drone_y   );
    // bool bIsClose = distance_square < limitRadius_square;
    //~ bool bIsClose = true;
    bool bIsClose = distance_square < 9;
    _IsOnTruckTop = distance_square < 1; // Hard coded, define if < 0.7 m2, then means on the top

    // Calculate desired postion and yaw; then send commands
    geometry_msgs::Point desired_position;
    desired_position.x = bIsClose ? _msgTruckLocalPosition.point.x : target_x;
    desired_position.y = bIsClose ? _msgTruckLocalPosition.point.y : target_y;
    desired_position.z = SEARCH_ALTITUDE;

    float desired_yaw = (float)UasMath::ConvertRad2Deg(atan2(_msgTruckDistance.point.y, _msgTruckDistance.point.x));
    //~ RunLocalPositionControl(desired_position, desired_yaw);
    //~ RunAttitudeControl(desired_position, desired_yaw);
    
    if (bIsClose && _bIsLocalLocationControlEnable ){
		RunLocalPositionControl(desired_position, desired_yaw);
	}
	else{
		
		RunAttitudeControl(desired_position, desired_yaw);
		
	}

    // Calculate yaw angle
    dji_sdk::AttitudeQuaternion q = drone.attitude_quaternion;
    float yaw = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) );
    float pitch = (float)UasMath::ConvertRad2Deg( asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1)) );
    float roll = (float)UasMath::ConvertRad2Deg( atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)) );

	// searching target with gimbal sweep
	if (distance_square < 36)
	{
		float pitch_angle = -45;
		float yaw_angle = YAW_RANGE*sin(((float)_SearchGimbalPhi/(RATIO_GIMBAL*40))*6.28);
		geometry_msgs::PointStamped msgDesiredAngleDeg;
		msgDesiredAngleDeg.point.x = 0;
		msgDesiredAngleDeg.point.y = pitch_angle;
		msgDesiredAngleDeg.point.z = yaw_angle;
		//~ ROS_INFO("Ptich = %f , Yaw: %f.", pitch_angle, yaw_angle);
		_GimbalAnglePub.publish(msgDesiredAngleDeg);
		_SearchGimbalPhi = _SearchGimbalPhi + 1;
		if (_SearchGimbalPhi > RATIO_GIMBAL*40){
			_SearchGimbalPhi = 0;
        }
    }   
    // Print out data
    //~ ROS_INFO("Desired Local position:%f, %f ,distance_square:%f, Close?:%d.",desired_position.x, desired_position.y, distance_square, bIsClose);
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
                     << distance_square << ","
                     << _msgTruckLocalPosition.point.x << ","
                     << _msgTruckLocalPosition.point.y << ","
                     << _msgTruckLocalPosition.point.z << ","
                     << _msgTruckVelocity.point.x << ","
                     << _msgTruckVelocity.point.y << ","
                     << _msgTargetLocalPosition.point.x << ","
                     << _msgTargetLocalPosition.point.y << ","
                     << _msgTargetLocalPosition.point.z << ","   // target local position
                     << drone.global_position.latitude << ","
                     << drone.global_position.longitude << ","
                     << drone.local_position.x << ","
                     << drone.local_position.y << ","
                     << drone.local_position.z << ","
                     << drone.velocity.vx << ","
                     << drone.velocity.vy << ","
                     << drone.velocity.vz << ","
                     << _msgDesiredAttitudeDeg.point.x << ","
                     << _msgDesiredAttitudeDeg.point.y << ","
                     << _msgDesiredAttitudeDeg.point.z << ","
                     << roll << ","
                     << pitch << ","
                     << yaw << std::endl;                                // drone local position                           // drone local position
}


void RunAutonomousLanding3(){

    ROS_INFO("IsOnTruckTop?:%d, IsTargetFound?:%d", _IsOnTruckTop, _bIsTargetFound);

    if(_bIsTargetFound){
        _bIsSearchInitiated = false;
        RunAutonomousLanding2();
        return;
    }

    if(!_IsOnTruckTop){
        GoToTruckGPSLocation();
        return;
    }
    else{
        RunTargetSearch();
        return;
    }

}


void timerCallback(const ros::TimerEvent&)
{
    DJIDrone& drone = *_ptrDrone;

    // we need to run this functioin regardless of the navigation menu.
    RunTimeCriticalTasks();
    RunTargetTracking();
    //~ RunSensorFusing();


    if (_nNavigationTask < 21 || _nNavigationTask > 90)
    // we don't take care of these cases in this callback function.
    // They are taken care in navigationTaskCallback.
    {
        return;
    }

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
            ROS_INFO_STREAM("Search center reinitialized.");

            // Go back to Truck GPS location. RunAutonomousLanding3
            _IsOnTruckTop = false;
            ROS_INFO_STREAM("Time to go back to Truck GPS location.");
            
            // Reset MPC integrator sum position error
            _mpc.IsXpInitialized_ = false;
            
            _sumPosErrX = 0;
            _sumPosErrY = 0;
            ROS_INFO_STREAM("Reset MPC integrator sum position error.");
            
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
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
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


    // Log files
    // std::stringstream ss;
    // ss << DEFAULT_TARGET_TRACKING_LOG_FILE_NAME << ros::WallTime::now() << ".log";
    // _ofsTragetTrackingLog.open(ss.str());
    // ROS_ASSERT_MSG(_ofsTragetTrackingLog, "Failed to open file %s", ss.str().c_str());
    // _ofsTragetTrackingLog << "#Time,TagDistance(x,y,z),TargetDistance(x,y,z),TargetLocalPosition(x,y,z)" << std::endl;

    //~ ss.str("");
    std::stringstream ss;
    ss << DEFAULT_GO_TO_TRUCK_LOG_FILE_NAME  << currentDateTime() << "_" << "q" << mpc_q << "_" << "ki" << mpc_kiPos  << ".log";
    _ofsGoToTruckLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsGoToTruckLog, "Failed to open file %s", ss.str().c_str());
    _ofsGoToTruckLog << "#Time,UltrasonicDistance,UltrasonicReliability,TargetDistance,TruckLocalPosition(x,y,z),TargetLocalPosition(x,y,z),DroneGPS(latitude,longtitude),DroneLocation(x,y,z), DroneVelocity(x,y,z), DroneDesiredAttitude, DroneAttitude" << std::endl;

    ss.str("");
    ss << DEFAULT_AUTONOMOUS_LANDING_LOG_FILE_NAME   << currentDateTime() << "_" << "q" << mpc_q << "_" << "ki" << mpc_kiPos << ".log";
    _ofsAutonomousLandingLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsAutonomousLandingLog, "Failed to open file %s", ss.str().c_str());
    _ofsAutonomousLandingLog << "#Time,UltrasonicDistance,UltrasonicReliability,TargetDistance,TruckLocalPosition(x,y,z),TargetLocalPosition(x,y,z),DroneGPS(latitude,longtitude),DroneLocation(x,y,z), DroneVelocity(x,y,z), DroneDesiredAttitude, DroneAttitude" << std::endl;

    ss.str("");
    ss << DEFAULT_SEAECHING_RANGE_LOG_FILE_NAME << currentDateTime() << ".log";
    _ofsSearchingRangeLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsSearchingRangeLog, "Failed to open file %s", ss.str().c_str());
    _ofsSearchingRangeLog << "#Time,DroneLocation(x,y,z,yaw), GimbalAngle(yaw, pitch), DroneVelocity(vx, vy, vz)" << std::endl;

    // Log about MPC controller
    ss.str("");
    ss << DEFAULT_MPC_CONTROLLER_LOG_FILE_NAME << currentDateTime() << "_" << "q" << mpc_q << "_" << "ki" << mpc_kiPos << ".log";
    _ofsMPCControllerLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsMPCControllerLog, "Failed to open file %s", ss.str().c_str());
    
    // Log about KalmanFilter
    ss.str("");
    ss << DEFAULT_KALMAN_FILTER_LOG_FILE_NAME << currentDateTime() << ".log";
    _ofsKalmanFilterLog.open(ss.str());
    ROS_ASSERT_MSG(_ofsKalmanFilterLog, "Failed to open file %s", ss.str().c_str());
    

    // Ultrasonic
    _msgUltraSonic.ranges.resize(1);
    _msgUltraSonic.intensities.resize(1);


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
    _GimbalAnglePub = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 10);
    _TargetLocalPositionPub = nh.advertise<geometry_msgs::PointStamped>("/navigation/target_local_position", 10);
    _TruckLocalPositionPub = nh.advertise<geometry_msgs::PointStamped>("/navigation/truck_local_position", 10);
    _FusedTargetLocalPositionPub = nh.advertise<geometry_msgs::PointStamped>("/navigation/fused_target_local_position", 10);

    // main control loop = 20 Hz
    double dTimeStepSec = 0.025;
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);


    ROS_INFO_STREAM("Navigation has started.");

    ros::spin();

    return 0;

}
