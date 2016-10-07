#include "ros/ros.h" //note this must come before including 
#include <ostream>
#include <fstream>

//#include <geometry_msgs/PointStamped.h>
//#include <dji_sdk/dji_drone.h>


class PidController {

private: // members

	std::string m_sID; //assign this on creation for debugging on ROS
 
 	// initial guess of kp=0.5 was somewhat slow, try 0.8 was whippy,
	// 2 works well, try 4 for more speed
	double m_dKp; 
	double m_dKd; 
	double m_dKi; 
	
	double m_dTimeStepSec;

	double m_dDeadZoneAngleDU; //DJI Unit anything less than 1 degrees, we don't care
	
	double m_dAccumulatedError; //need to start out at 0-element

	double m_dLastMeasuredTimeSec;
	
	std::ofstream m_ofslog;
	
	
private: // methods
	
	/**
	* Normalize angle to be in (-pi, pi]
	*/
	double NormalizeAngleDU(double dAngleDU);
	

	
private: // NOT IMPLEMENTED
	PidController(const PidController&);  // copy constructor
	PidController& operator=(const PidController&); // assignment
	
public: // methods
	
	PidController();
		
    PidController ( std::string sID = "NA", 
                    double kp = 1.0, 
                    double kd = 0.0, 
                    double ki = 0.0,
                    double deadZoneAngleDU = 10.0,
                    double timeStepSec = 0.02);
    
    
    virtual ~PidController();
    
    double CalculateDesiredVelocity(double error_DU, double timeSinceLastStep); 
    
    double GetPlantInput(double dDesiredAngleDU, 
                         double dMeasuredTimeDeg,
                         double dGimbalAngleDeg);

    std::ostream& GetString(std::ostream& os);
    
    
/*    
				
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

          */                          											
};

// non-member methods
std::ostream& operator<<(std::ostream& os, PidController& pc);

