#include "ros/ros.h"
#include "Navigation/Navigation.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <sstream>
#include <iostream> // std::cout, std::end, ...
#include <iomanip> // std::setprecision
#include <dji_sdk/dji_drone.h>
#include <fstream>
#include <unistd.h>
#include <limits>


#define DEFAULT_LOG_FILE_NAME "/Navigation_data_"

using namespace std;

int targetLocked = 0;
std::ofstream m_ofslog;
DJIDrone* _ptrDrone;

ros::Publisher gimbal_pose_pub1;

geometry_msgs::Point _droneUtmPosition;
geometry_msgs::Point _targetGpsPosition;
geometry_msgs::Point _targetUtmPosition;



void SigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    ROS_INFO("It is requested to terminate navigation ...");
    // delete(_ptrDrone);

    m_ofslog.close();
    ROS_INFO("Finish writing log.");
      
    ROS_INFO("Shutting down navigation ...");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


void listernerCallback(const geometry_msgs::PointStamped::ConstPtr& msgDesiredPoseDeg)
{
    //_msgDesiredGimbalPoseDeg = *msgDesiredPoseDeg;
    //ROS_INFO_STREAM("Desired Angle (Deg) Roll:" << _msgDesiredGimbalPoseDeg.point.x 
    //                        << " Pitch:" << _msgDesiredGimbalPoseDeg.point.y 
    //                        << " Yaw:" <<  _msgDesiredGimbalPoseDeg.point.z);
}

void droneUtmCallback(const geometry_msgs::PointStamped::ConstPtr& msgDroneUtmPos)
{
	if (0 < sizeof(msgDroneUtmPos)) 
    {	
    	_droneUtmPosition.x = msgDroneUtmPos->point.x;
		_droneUtmPosition.y = msgDroneUtmPos->point.y;
		_droneUtmPosition.z = msgDroneUtmPos->point.z;
	}

	// ROS_INFO_STREAM("Drone UTM Position: X = " << msgDroneUtmPos->point.x 
 //                           << " Y = " << msgDroneUtmPos.point.y 
 //                           << " Z = " << msgDroneUtmPos.point.z);

}

void targetGpsCallback(const geometry_msgs::PointStamped::ConstPtr& msgTargetGpsPos)
{
	if (0 < sizeof(msgTargetGpsPos)) 
    {
		_targetGpsPosition.x = msgTargetGpsPos->point.x;
		_targetGpsPosition.y = msgTargetGpsPos->point.y;
		_targetGpsPosition.z = msgTargetGpsPos->point.z;
		
		targetLocked = 1;
		
	}

	// ROS_INFO_STREAM("Target GPS Position: X = " << msgTargetGpsPos->point.x 
 //                           << " Y = " << msgTargetGpsPos.point.y 
 //                           << " Z = " <<  msgTargetGpsPos.point.z);
		

}
void targetUtmCallback(const geometry_msgs::PointStamped::ConstPtr& msgTargetUtmPos)
{
    if (0 < sizeof(msgTargetUtmPos))
    {	
    	_targetUtmPosition.x = msgTargetUtmPos->point.x;
		_targetUtmPosition.y = msgTargetUtmPos->point.y;
		_targetUtmPosition.z = msgTargetUtmPos->point.z;
	}

	// ROS_INFO_STREAM("Target UTM Position: X = " << msgTargetUtmPos->point.x 
 //                           << " Y = " << msgTargetUtmPos.point.y 
 //                           << " Z = " <<  msgTargetUtmPos.point.z);

}

void WritingLog(const ros::TimerEvent&){

	DJIDrone& drone=*_ptrDrone;

    dji_sdk::AttitudeQuaternion q;
    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    q = drone.attitude_quaternion;
    roll_rad  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
    pitch_rad = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
    yaw_rad   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
	


	m_ofslog  	<< std::setprecision(std::numeric_limits<double>::max_digits10) 
				//Time
        		<< ros::Time::now().toSec() << "," 
				//Global Position
        		<< drone.global_position.latitude << ","
                << drone.global_position.longitude << ","
                << drone.global_position.altitude << ","
                << drone.global_position.height << ","
				//Local Position
				<< drone.local_position.x << ","
                << drone.local_position.y << ","
                << drone.local_position.z << ","
				//Target GPS location
				<< _targetGpsPosition.x << ","
				<< _targetGpsPosition.y << ","
				<< _targetGpsPosition.z << ","
				//Orientation
                << yaw_rad << "," 
                << pitch_rad << ","   
                << roll_rad << ","
				//Velocity
				<< drone.velocity.vx << ","
                << drone.velocity.vy << ","
                << drone.velocity.vz << ","				
				//Gimbal Angle
        		<< drone.gimbal.yaw << ","
                << drone.gimbal.pitch << ","
                << drone.gimbal.roll << std::endl;
				
}
                                                    

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_node");
	ros::NodeHandle nh;
	DJIDrone& drone=*_ptrDrone;

    signal(SIGINT, SigintHandler);
    
    gimbal_pose_pub1 = nh.advertise<geometry_msgs::PointStamped>("/gimbal_control/desired_gimbal_pose", 1000);
        
	
	int numMessagesToBuffer = 2;
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/drone_utm_position", numMessagesToBuffer, droneUtmCallback);
	ros::Subscriber sub2 = nh.subscribe("/dji_sdk/target_gps_position", numMessagesToBuffer, targetGpsCallback);
	ros::Subscriber sub3 = nh.subscribe("/dji_sdk/target_utm_position", numMessagesToBuffer, targetUtmCallback);
   

	char* rosHome = "/home/ubuntu";
    ROS_ASSERT_MSG(rosHome, "Can't find the environment variable, ROS_HOME");
    
    std::stringstream ss;
	int time_now = ros::Time::now().toSec() - int(ros::Time::now().toSec()/10000) * 10000;
    ss << rosHome << DEFAULT_LOG_FILE_NAME << time_now << ".log";
    m_ofslog.open(ss.str());
    ROS_ASSERT_MSG(m_ofslog, "Failed to open file %s", ss.str().c_str());
	
	m_ofslog << "#Time,Latitude(deg),Longitude(deg),Altitude(meter),Height(meter),Local Position.x(meter),Local Position.y(meter),Local Position.z(meter),Target GPS location.x(deg),Target GPS location.y(deg),Target GPS location.z(meter),Yaw(rad),Pitch(rad),Roll(rad),Velocity.vx(m/s),Velocity.vy(m/s),Velocity.vz(m/s),Gimbal.yaw(deg),Gimbal.pitch(deg),Gimbal.roll(deg)" << endl;

    _ptrDrone = new DJIDrone(nh);


    double dTimeStepSec = 0.02;
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), WritingLog);

	Navigation navigator(nh);
    navigator.RunNavigation();

	signal(SIGINT, SigintHandler);

    ros::spin();
             
    return 0;    

}
