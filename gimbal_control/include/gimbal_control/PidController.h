#include "ros/ros.h" //note this must come before including 
//#include <string>
//#include <sstream> //stringstream, for turning all PID controller params into a string for ROS
//#include <stdlib.h>    //for absolute value
//#include <ctype.h> //for isspace

//#include <fstream>
#include <ostream>
//#include <vector>
//#include <algorithm>    // std::transform
//#include<cstring> //std::strlen
//#include "std_msgs/String.h" //for publishing to ROS

//#include <geometry_msgs/PointStamped.h>
//#include <dji_sdk/dji_drone.h>


class PidController {

private:

    //std::vector<std::string> listOfParams = {"kp", "kd", "ki", "deadzone_djiunits"} ;

    //static constexpr double min_Kp = 0.125 ; 
    //can't assign a value lower than this in order to rpevent user errors. 
    // Use 0.125 since it can be represented precivesly using floats
	
	std::string m_sID; //assign this on creation for debugging on ROS
 
	double m_dKp; 
	// initial guess of 0.5 was somewhat slow, try 0.8 was whippy,
	// 2 works well, try 4 for more speed
	double m_dKd; //initial guess
	double m_dKi; //initial guess was 0.0
	
	double m_dDeadZoneAngleDU; //DJI Unit anything less than 1 degrees, we don't care
	
	double m_dAccumulatedError; //need to start out at 0-element
	
	double m_dTimeStepSec;
	
	double m_dLastMeasuredTimeSec;
	
	/**
	* Normalize angle to be in (-pi, pi]
	*/
	double NormalizeAngleDU(double dAngleDU);
	
	
	
public:
	
	PidController();
		
    PidController(std::string sID, double kp, double kd, double ki);
    
    double CalculateDesiredVelocity(double error_DU, double timeSinceLastStep); 
    
    double GetPlantInput(double dDesiredAngleDU, 
                         double dMeasuredTimeDeg,
                         double dGimbalAngleDeg);

    std::ostream& GetString(std::ostream& os);
    
    
/*    
    void publishAllParamValues ();
				
    double getRequiredVelocityPID ( double desiredAngle_djiUnits, 
                                    double currentAngle_djiUnits,
                                    double latest_dt);
                                
                                
    double getRequiredVelocityPID_yaw ( double desiredAngle_DU, 
                                        double currentAngle_DU, 
                                        int signOfMovement,
                                        // make positive if you started traveling in positive direction, 
                                        // negative otherwise
                                        bool& isUnwinding, 
                                        double tolerance_DjiUnits,
                                        double latest_dt);
                        
    void setParamFromString(std::string param, double paramVal);
          */                          											
};

std::ostream& operator<<(std::ostream& os, PidController& pc);

