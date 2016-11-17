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
    
    double m_dDeadZoneAngleDeg; //DJI Unit anything less than 1 degrees, we don't care
	
    // If m_bIsIntelligentControl is false, the gimbal will rotate only from -180 to 180 degrees.
    // If m_bIsIntelligentControl is true, the gimbal may rotate from -315 to 315 degrees.
    bool m_bIsIntelligentControl;  
	
	double m_dPrevOutputDeg;
	
	double m_dPrevErrorDeg;
	
	bool m_bIsPrevValueInitialized;
	                            
	std::ofstream m_ofslog;
	
private: // methods
	
    /**
    * Normalize an angle about the center angle. 
    * If the center angle is 0, then the output will be (-180, 180]
    * If the center angle is 90, then the output will be (-90, 270]
    * The output is always less then or equal to dCenter+180 and less than dCenter-180.
    */
    double NormalizeAngleAboutDeg(double dAngleDeg, double dCenter);
	
    /**
    * Normalize an angle, the output angle will be (-180, 180]
    * If the center angle is 90, then the output will be (-90, 270]
    */
    double NormalizeAngleDeg(double dAngleDeg);
	
    void ConstructorHelper();

    /**
    * RunIntelligentControl
    * Returns the plant input  
    * Good for yaw control
    */
    double RunIntelligentControl(double dDesiredAngleDeg, double dGimbalAngleDeg);
    
    /**
    * RunNormalControl
    * Returns the plant input  
    * Good for pitch and roll control
    */
    double RunNormalControl(double dDesiredAngleDeg, double dGimbalAngleDeg);
    
private: // NOT IMPLEMENTED
	PidController(const PidController&);  // copy constructor
	PidController& operator=(const PidController&); // assignment
	
public: // methods
	
	PidController();
		
    PidController ( std::string sID = "NA", 
                    double      kp = 2.0, 
                    double      kd = 0.0, 
                    double      ki = 0.0,
                    double      timeStepSec = 0.02,
                    double      deadZoneAngleDeg = 1.0,
                    bool        isIntelligentControl = false
                    );
    
    
    virtual ~PidController();
    
    //double CalculateDesiredVelocity(double error_DU, double timeSinceLastStep); 
    
    double GetPlantInput(double dDesiredAngleDeg, double dGimbalAngleDeg);

    std::ostream& GetString(std::ostream& os);

};

// non-member methods
std::ostream& operator<<(std::ostream& os, PidController& pc);

