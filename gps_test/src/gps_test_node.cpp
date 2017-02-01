#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
//#include "gps_test/gps_test.h"
#include <signal.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <limits>


#define DEFAULT_LOG_FILE_NAME "/GPS_data"


using namespace std;

DJIDrone* _ptrDrone;
std::ofstream m_ofslog;


void SigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate gps_test...");
    delete(_ptrDrone);
    
    m_ofslog.close();
    ROS_INFO("Finish writing log.");
    
      
    ROS_INFO("Shutting down gps_test...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


void TestGPS(const ros::TimerEvent&){

    dji_sdk::AttitudeQuaternion q;
    double roll_rad;
    double pitch_rad;
    double yaw_rad;
    
     
	DJIDrone& drone=*_ptrDrone;

	
    
    //Calculate roll,pitch,yaw of drone.
    q = drone.attitude_quaternion;
    roll_rad  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
    pitch_rad = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
    yaw_rad   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    
    //cout << "Position output:" << drone.global_position.latitude << "," << drone.global_position.longitude << "," << drone.global_position.altitude << "," << drone.global_position.height << endl ;
    //cout << "Roll Pitch Yaw output:" << roll_rad << "," << pitch_rad << "," << yaw_rad << endl ;
    
    //Write data to the log.
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
            	<< ros::Time::now().toSec() << "," 
                << drone.global_position.latitude << "," 
                << drone.global_position.longitude << "," 
                << drone.global_position.altitude << "," 
                << drone.global_position.height << "," 
                << yaw_rad << "," 
                << pitch_rad << ","   
                << roll_rad << endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "gps_test_node");
    ros::NodeHandle nh;
    ROS_INFO("GPS TESTING IS INITIALIZING!!!!!");
    
    //Initialize variables.
	char* rosHome = getenv ("ROS_HOME");
    ROS_ASSERT_MSG(rosHome, "Can't find the environment variable, ROS_HOME");
    
    stringstream ss;
    ss << rosHome << DEFAULT_LOG_FILE_NAME << ".log";
    m_ofslog.open(ss.str());
    ROS_ASSERT_MSG(m_ofslog, "Failed to open file %s", ss.str().c_str());
    
    m_ofslog << "#Time,Latitude(deg),Longitude(deg),Altitude(meter),Height(meter),Yaw(rad),Pitch(rad),Roll(rad)" << endl;
    
    
    
    
    double dTimeStepSec = 0.02;
    ROS_INFO("Controller time step is %f\n", dTimeStepSec);
    
    _ptrDrone = new DJIDrone(nh);
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), TestGPS);
    
    
    
    signal(SIGINT, SigintHandler);
    
    ros::spin();        
             
    return 0; 

} 


//Use this line to debug:
/*ROS_INFO("Initialize dPrevGimbalAngle with %f\n", m_dPrevOutputDeg);    */




