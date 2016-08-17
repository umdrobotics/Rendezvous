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
	   {break;}	
	}
}
static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [a] SDK Version Query         | [s] Virtual RC Test             |\n");
	printf("| [b] Request Control           | [t] Set Sync Flag Test          |\n");	
	printf("| [c] Release Control           | [u] Set Msg Frequency Test      |\n");	
	printf("| [d] Takeoff                   | [v] Waypoint Mission Upload     |\n");	
	printf("| [e] Landing                   | [w] Hotpoint Mission Upload     |\n");	
	printf("| [f] Go Home                   | [x] Followme Mission Upload     |\n");	
	printf("| [g] Gimbal Control Sample     | [y] Mission Start               |\n");	
	printf("| [h] Attitude Control Sample   | [z] Mission Pause               |\n");	
	printf("| [i] Draw Circle Sample        | [1] Mission Resume              |\n");	
	printf("| [j] Draw Square Sample        | [2] Mission Cancel              |\n");	
	printf("| [k] Take a Picture            | [3] Mission Waypoint Download   |\n");	
	printf("| [l] Start Record Video        | [4] Mission Waypoint Set Speed  |\n");	 
	printf("| [m] Stop Record Video         | [5] Mission Waypoint Get Speed  |\n");	
	printf("| [n] Local Navigation Test     | [6] Mission Hotpoint Set Speed  |\n");	
	printf("| [o] Global Navigation Test    | [7] Mission Hotpoint Set Radius |\n");	
	printf("| [p] Waypoint Navigation Test  | [8] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [q] Arm the Drone             | [9] Mission Followme Set Target |\n");	
	printf("| [r] Disarm the Drone          | [0] test PID controller   |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
    printf("input: ");
}

int main(int argc, char **argv)
{
    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");//must come before declaring NodeHandle
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle n; //had to make global so global DJIDRone declaration can use it
    nh = &n;
    //DJIDrone* drone = new DJIDrone(nh);//had to make global so callback can see it
     drone = new DJIDrone(n);
	//virtual RC test data
	uint32_t virtual_rc_data[16];
	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
	
    Display_Main_Menu();
    while(1)
    {
		ros::spinOnce();
        temp32 = getchar();
        if(temp32 != 10)
        {
            if(valid_flag == false)
            {
                main_operate_code = temp32;
                valid_flag = true;
            }
            else
            {
                err_flag = true;
            }
            continue;
        }
        else
        {
            if(err_flag == true)
            {
                printf("input: ERROR\n");
                Display_Main_Menu();
                err_flag = valid_flag = false;
                continue;
            }
        }
        switch(main_operate_code)
        {
			case 'a':
				/* SDK version query*/
				drone->check_version();
				break;
            case 'b':
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 'c':
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 'd':
                /* take off */
                drone->takeoff();
                break;
            case 'e':
                /* landing*/
                drone->landing();
                break;
            case 'f':
                /* go home*/
                drone->gohome();
                break;
            case 'g':
                /*gimbal test*/

              /* drone->gimbal_angle_control(0, -900, 0, 11);

                sleep(1);



                drone->gimbal_angle_control(0, 500, 0, 10);
                sleep(2);
               drone->gimbal_angle_control(0, -300, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 100, 0, 10);
                sleep(2);
               drone->gimbal_angle_control(0, -900, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -900, -1800, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -900, -900, 10);
                sleep(2);
printf("\n and camera is roll %f pitch %f yaw %f ", drone->gimbal.roll, drone->gimbal.pitch, drone->gimbal.yaw);*/

                /*drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(0.8);
                drone->gimbal_speed_control(0, 0, 200); 
                sleep(2);
                drone->gimbal_speed_control(0, 0, -1800); //input of 400  seems to mvoe it 20 degrees, leading me to conclude that it moves for 0.5 seconds
                sleep(1);
                drone->gimbal_speed_control(0, 0, 1600); //input of 400  seems to mvoe it 20 degrees, leading me to conclude that it moves for 0.5 seconds
                sleep(0.75);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
               drone->gimbal_angle_control(0, 500, 0, 10);
                sleep(3);

                drone->gimbal_speed_control(0, -1800, 0); //1800 is the maximum speed for pitch (-1800 worked, -1801 didnt execute). This is also maximum for yaw(-1800 works, -1801 doesn't)

                sleep(2);
               // drone->gimbal_speed_control(0, -900, 0); */
                
               drone->gimbal_angle_control(0, -450, 450, 10);
                sleep(3);
            GIMBAL_TEST_SIGN = 1;
               while(1)
            {


            degs = (degs+10)%100;
            // //oscillate between 10, 20, 30, ... 90 degrees/second
             if(drone->gimbal.pitch >= 20.0) {drone->gimbal_speed_control(0, -1*10*degs, 0); GIMBAL_TEST_SIGN=-1;} else if (drone->gimbal.pitch < -20) {GIMBAL_TEST_SIGN=1; drone->gimbal_speed_control(0, 1*10*degs, 0);} //it doesn't seem to be flipping to a negative sign even after I declared it volatile
            cout<<"degs " <<degs <<"sign " << GIMBAL_TEST_SIGN <<"\n"; 
            }
  
                sleep(2);

                /*drone->gimbal_angle_control(0, 0, -1800, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, -2700, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 900, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 1800, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 2700, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 10);
                sleep(4);

                drone->gimbal_angle_control(0, 0, -1800, 10);
                sleep(2);
                drone->gimbal_angle_control(300, 0, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(-300, 0, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, 300, 0, 10);
                sleep(2);
                drone->gimbal_angle_control(0, -300, 0, 10);
                sleep(2);

                drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(0, 0, 200);
                sleep(2);
                drone->gimbal_speed_control(0, 0, -200);
                sleep(2);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
                drone->gimbal_speed_control(0, -200, 0);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 20);/**/
                break;

            case 'h':
                /* attitude control sample*/
                drone->takeoff();
                sleep(8);


                for(int i = 0; i < 100; i ++)
                {
                    if(i < 90)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, -2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, -2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, 0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, -0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, 90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, -90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                drone->landing();

                break;

            case 'i':
                /*draw circle sample*/
                static float time = 0;
                static float R = 2;
                static float V = 2;
                static float vx;
                static float vy;
                /* start to draw circle */
                for(int i = 0; i < 300; i ++)
                {
                    vx = V * sin((V/R)*time/50.0f);
                    vy = V * cos((V/R)*time/50.0f);
        
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            vx, vy, 0, 0 );
                    usleep(20000);
                    time++;
                }
                break;

            case 'j':
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, 3, 0, 0 );
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, -3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, -3, 0, 0);
                    usleep(20000);
                }
                break;
            case 'k':
                /*take a picture*/
                drone->take_picture();
                break;
            case 'l':
                /*start video*/
                drone->start_video();
                break;
            case 'm':
                /*stop video*/
                drone->stop_video();
                break;
            case 'n':
                /* Local Navi Test */
                drone->local_position_navigation_send_request(-100, -100, 100);
                break;
            case 'o':
                /* GPS Navi Test */
                drone->global_position_navigation_send_request(22.535, 113.95, 100);
                break;
            case 'p':
                /* Waypoint List Navi Test */
                {
                    waypoint0.latitude = 22.535;
                    waypoint0.longitude = 113.95;
                    waypoint0.altitude = 100;
                    waypoint0.staytime = 5;
                    waypoint0.heading = 0;
                }
                newWaypointList.waypoint_list.push_back(waypoint0);

                {
                    waypoint1.latitude = 22.535;
                    waypoint1.longitude = 113.96;
                    waypoint1.altitude = 100;
                    waypoint1.staytime = 0;
                    waypoint1.heading = 90;
                }
                newWaypointList.waypoint_list.push_back(waypoint1);

                {
                    waypoint2.latitude = 22.545;
                    waypoint2.longitude = 113.96;
                    waypoint2.altitude = 100;
                    waypoint2.staytime = 4;
                    waypoint2.heading = -90;
                }
                newWaypointList.waypoint_list.push_back(waypoint2);

                {
                    waypoint3.latitude = 22.545;
                    waypoint3.longitude = 113.96;
                    waypoint3.altitude = 10;
                    waypoint3.staytime = 2;
                    waypoint3.heading = 180;
                }
                newWaypointList.waypoint_list.push_back(waypoint3);

                {
                    waypoint4.latitude = 22.525;
                    waypoint4.longitude = 113.93;
                    waypoint4.altitude = 50;
                    waypoint4.staytime = 0;
                    waypoint4.heading = -180;
                }
                newWaypointList.waypoint_list.push_back(waypoint4);

                drone->waypoint_navigation_send_request(newWaypointList);
                break;
			case 'q':
				//drone arm
				drone->drone_arm();
                break;
			case 'r':
				//drone disarm
				drone->drone_disarm();
                break;
			case 's':
				//virtual rc test 1: arm & disarm
				drone->virtual_rc_enable();
				usleep(20000);

				virtual_rc_data[0] = 1024-660;	//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024-660;	//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024-660;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024+660;	//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1684;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for (int i = 0; i < 100; i++){
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}

				//virtual rc test 2: yaw 
				drone->virtual_rc_enable();
				virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024+660;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for(int i = 0; i < 100; i++) {
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}
				drone->virtual_rc_disable();
				break;
			case 't':
				//sync flag
				drone->sync_flag_control(1);
				break;
			case 'u':
				//set msg frequency
				drone->set_message_frequency(msg_frequency_data);
				break;

			case 'v':
				//mission waypoint upload
				waypoint_task.velocity_range = 10;
				waypoint_task.idle_velocity = 3;
				waypoint_task.action_on_finish = 0;
				waypoint_task.mission_exec_times = 1;
				waypoint_task.yaw_mode = 4;
				waypoint_task.trace_mode = 0;
				waypoint_task.action_on_rc_lost = 0;
				waypoint_task.gimbal_pitch_mode = 0;

				waypoint.latitude = 22.540091;
				waypoint.longitude = 113.946593;
				waypoint.altitude = 100;
				waypoint.damping_distance = 0;
				waypoint.target_yaw = 0;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 0;
				/*
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;
				*/

				waypoint_task.mission_waypoint.push_back(waypoint);

				waypoint.latitude = 22.540015;
				waypoint.longitude = 113.94659;
				waypoint.altitude = 120;
				waypoint.damping_distance = 2;
				waypoint.target_yaw = 180;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 0;
				/*
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_list[1] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;
				waypoint.waypoint_action.command_parameter[1] = 1;
				*/

				waypoint_task.mission_waypoint.push_back(waypoint);

				drone->mission_waypoint_upload(waypoint_task);
				break;
			case 'w':
				//mission hotpoint upload
				hotpoint_task.latitude = 22.540091;
				hotpoint_task.longitude = 113.946593;
				hotpoint_task.altitude = 20;
				hotpoint_task.radius = 10;
				hotpoint_task.angular_speed = 10;
				hotpoint_task.is_clockwise = 0;
				hotpoint_task.start_point = 0;
				hotpoint_task.yaw_mode = 0;

				drone->mission_hotpoint_upload(hotpoint_task);
				break;
			case 'x':
				//mission followme upload
				followme_task.mode = 0;
				followme_task.yaw_mode = 0;
				followme_task.initial_latitude = 23.540091;
				followme_task.initial_longitude = 113.946593;
				followme_task.initial_altitude = 10;
				followme_task.sensitivity = 1;

				drone->mission_followme_upload(followme_task);
				break;
			case 'y':
				//mission start
				drone->mission_start();
				break;
			case 'z':
				//mission pause
				drone->mission_pause();
				break;
			case '1':
				//mission resume
				drone->mission_resume();
				break;
			case '2':
				//mission cancel
				drone->mission_cancel();
				break;
			case '3':
				//waypoint mission download
				waypoint_task = drone->mission_waypoint_download();
				break;
			case '4':
				//mission waypoint set speed
				drone->mission_waypoint_set_speed((float)5);
				break;
			case '5':
				//mission waypoint get speed
				printf("%f", drone->mission_waypoint_get_speed());
				break;
			case '6':
				//mission hotpoint set speed
				drone->mission_hotpoint_set_speed((float)5,(uint8_t)1);
				break;
			case '7':
				//mission hotpoint set radius
				drone->mission_hotpoint_set_radius((float)5);
				break;
			case '8':
				//mission hotpoint reset yaw
				drone->mission_hotpoint_reset_yaw();
				break;
			case '9':
				//mission followme update target
				for (int i = 0; i < 20; i++)
				{
					followme_target.latitude = 22.540091 + i*0.000001;
					followme_target.longitude = 113.946593 + i*0.000001;
					followme_target.altitude = 100;
					drone->mission_followme_update_target(followme_target);
					usleep(20000);
				}
				break;
			case '0':

				printf ("Starting to listen for angle on %s", DesiredAngleTopic );
				int defaultMessagesToBuffer = 1;
				controlGimbal(*nh, cDesiredAngleTopic, defaultMessagesToBuffer, drone) ;
				cout << "done with pid test";

            default:
                break;
        }
        main_operate_code = -1;
        err_flag = valid_flag = false;
        Display_Main_Menu();
    }
    return 0;
}
