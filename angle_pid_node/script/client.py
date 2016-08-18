//publisher and subscriber in same node described here: http://answers.ros.org/question/48727/publisher-and-subscriber-in-the-same-node/


#include <angle_pid_node/kalmanPhysics.cpp>  //need to figure out how to put this in the main folder instead of include
               volatile int degs =0; // for debugging gimbal control
             volatile int GIMBAL_TEST_SIGN = 1; //for debugging gimbal control

#include <angle_pid_node/PIDcontrol.cpp> //need to figure out how to put this in the main folder instead of include

#define YAW_RELATIVE_TO_BODY true // if gimbal yaw command is relative to the body, this is true, if it's relative to the inertial frame, it's false
///Begin global tracking variables
//These variables need to be global so tha Apriltag Detection Callback can access them

double LATEST_TIMESTAMP;
double LATEST_DT; //the time interval




//Note: The angle control system needs to have access to the DJIDrone* object to send the gimbal commands
// or else we'll need to simultaneously publish to and subscribe from the node that does the PID calculations.
//Because of that, I'm placing the PID control within this node for now. 
PIDController* GLOBAL_ROLL_CONTROLLER = new PIDController();
PIDController* GLOBAL_PITCH_CONTROLLER = new PIDController();
PIDController* GLOBAL_YAW_CONTROLLER = new PIDController();
double GLOBAL_ROLL_DJI_UNITS =0.0;
double GLOBAL_PITCH_DJI_UNITS =0.0;
double GLOBAL_YAW_DJI_UNITS =0.0;


//no need to expend any processing power if you haven't gotten a message yet
bool RECIEVED_FIRST_MESSAGE = false; 

///End global tracking variables


#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//following includes are to allow integration with AprilTags:
//instructions on how to get Catkin to see them are here:
// http://answers.ros.org/question/206257/catkin-use-ros-message-from-another-package/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/image_encodings.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include<vector>
#include <image_transport/image_transport.h>


//begin includes and definitions for geolocalization
#include <math.h> //used for sine and cosine for rotation matrix when geolocalizing target
#include <angle_pid_node/conversions.h> //provides GPS-UTM conversions
//end includes for integration with AprilTags
#include <tuple>


//DJI's gimbal angles are in tenths of a degree, but the drone->gimbal.yaw, etc, returns it in degrees. We need to be able to convert forward and back
double radiansToDjiUnits(double angle_rads)
 {
  return (10*180.0/M_PI)* angle_rads;
 }
double djiUnitsToRadians(double angle_dji)
 {
  return ( (M_PI*angle_dji)/ (10.0*180.0) );
 }

double radiansToDegrees(double angle_rads)
 {
  return (180.0/M_PI)* angle_rads;
 }
double degreesToRadians(double angle_degrees)
 {
       return ( (M_PI*angle_degrees)/ (1.0*180.0) );
 }
 
double degreesToDjiUnits(double angle_degrees)
 {
       return (10.0*angle_degrees); 
 }
double djiUnitsToDegrees(double angle_dji)
 {
       return ( (angle_dji)/ (10.0) );
 }
 
//I think this is how you get the yaw of the quadcopter body
 //this method is based off the documentation for onboard-sdk  on Github https://github.com/dji-sdk/Onboard-SDK/blob/3.1/doc/en/ProgrammingGuide.md
 void quaternionToRPY(   dji_sdk::AttitudeQuaternion q, double & roll, double& pitch,  double& yaw) //roll pitch and yaw are output variables
{ 
     roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
     pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
     yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}


//also need to be able to convert from the body frame to the coordinate frame, for gimbal yaw
//so calculate the angle you want relative to inertal frame, then use this function to get the command you need
double inertialFrameToBody_yaw(double angleToInertial_rads, DJIDrone* dronePointer)
   {

     //first need to convert from quaternion format to RPY to get drone's yaw 
        double roll_body;
  	double pitch_body;
 	double yaw_body;
        quaternionToRPY(dronePointer->attitude_quaternion, roll_body, pitch_body, yaw_body); //roll_body pitch_body and yaw_body are output variables that will be altered by this function
        //rotation to inertial frame = rotation to body + body's rotation to inertial frame
			//rotation to body = rotation to inertial frame - body's rotation to inertial frame
        double angleToBody = angleToInertial_rads - yaw_body;
      
         //now keep the angle between -180 and 180 degrees (ie -pi and pi)
        while(angleToBody < -1.0*M_PI) {angleToBody += 2.0*M_PI;}
        while(angleToBody > M_PI) {angleToBody -= 2.0*M_PI;}
     cout << " desired angle inertial " << angleToInertial_rads<< "body yaw " << yaw_body << "resulting angle for command " <<  angleToBody <<"\n";

       return angleToBody;
    }


double bodyFrameToInertial_yaw(double angleToBody_rads, DJIDrone* dronePointer)
   {

     //first need to convert from quaternion format to RPY to get drone's yaw 
        double roll_body;
  	double pitch_body;
 	double yaw_body;
        quaternionToRPY(dronePointer->attitude_quaternion, roll_body, pitch_body, yaw_body); //roll_body pitch_body and yaw_body are output variables that will be altered by this function
        //rotation to inertial frame = rotation to body + body's rotation to inertial frame
        double angleToInertial = angleToBody_rads + yaw_body;
      
         //now keep the angle between -180 and 180 degrees (ie -pi and pi)
        while(angleToInertial < -1.0*M_PI) {angleToInertial += 2.0*M_PI;}
        while(angleToInertial > M_PI) {angleToInertial -= 2.0*M_PI;}


       return angleToInertial;
    }




//ros::init(argc, argv, "sdk_client");//must come before declaring NodeHandle
ros::NodeHandle* nh; //had to make global so global DJIDRone declaration can use it
DJIDrone* drone;// = new DJIDrone(nh);//had to make global so callback can see it
#define DesiredAngleTopic "dji_sdk/desired_angle"

//this callback will only update what the desired angle is, not actually activate the PID control, since we want to control it faster than the messages come in
void pidCallback(const geometry_msgs::PointStamped::ConstPtr& desiredAngleMessage)//(const apriltags_ros::AprilTagDetectionArray /*sensor_msgs::ImageConstPtr&*/ tag_detection_array)
{
	 RECIEVED_FIRST_MESSAGE = true; 
	//for simplicity, I'm going to send the desired angle in the form of a pointStamped message (point with timestamp)
	//I will translate the indices according to their standard order, ie, since x comes before y and roll comes before pitch.
	// roll will be the x element and pitch will be the y element
	#define rollIndex x
	#define pitchIndex  y
	#define yawIndex z
	geometry_msgs::PointStamped current;
	current = desiredAngleMessage;
   
   double currentTime = current.header.stamp.nsec/1000000000.0 + current.header.stamp.sec;
   LATEST_DT = currentTime - LATEST_TIMESTAMP ;
 //be sure to keep track of time
   LATEST_TIMESTAMP = currentTime;
   
   GLOBAL_ROLL_DJI_UNITS = current.point.rollIndex;
   GLOBAL_PITCH_DJI_UNITS = current.point.pitchIndex;
   GLOBAL_YAW_DJI_UNITS = current.point.yawIndex;

   

	
}

void controlGimbal(ros::NodeHandle& n, char[] angleTopic, int numMessagesToBuffer, DJIDrone* drone)
{
	  ros::Subscriber sub = n.subscribe(angleTopic, numMessagesToBuffer, pidCallback);
	 char waitKeyChar = 0; //initialize to prevent errors 
	 waitKeyChar = cv::waitKey(1);

	while (ros::ok() && ! (waitKeyChar == 'q' || waitKeyChar == 'Q' ))
	{
		ros::spinOnce();
		if(RECIEVED_FIRST_MESSAGE == true)
		{
			
			dji_sdk::Gimbal currentAngle = drone->gimbal;
			double rollSpeedDesired  = getRequiredVelocityPID(GLOBAL_ROLL_DJI_UNITS, degreesToDjiUnits(currentAngle.roll),defaultTimeStep, GLOBAL_ROLL_CONTROLLER);
			double pitchSpeedDesired  = getRequiredVelocityPID(GLOBAL_PITCH_DJI_UNITS, degreesToDjiUnits(currentAngle.pitch),defaultTimeStep, GLOBAL_PITCH_CONTROLLER);
			double yawSpeedDesired  = getRequiredVelocityPID(GLOBAL_YAW_DJI_UNITS, degreesToDjiUnits(currentAngle.yaw),defaultTimeStep, GLOBAL_YAW_CONTROLLER);
			drone->gimbal_speed_control(rollSpeedDesired, pitchSpeedDesired, yawSpeedDesired);

		}
	 waitKeyChar = cv::waitKey(1);
	 if (waitKeyChar == 'q' || waitKeyChar == 'Q') //attempt to allow this to be exited by pressing q. Still need to test
	   {bre