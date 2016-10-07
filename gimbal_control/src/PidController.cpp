#include "gimbal_control/PidController.h"
#include "std_msgs/String.h"
#include <sstream>
#include <unistd.h>

#define GIMBAL_SPEED_LIMIT_DU 1800.0
#define DEFAULT_LOG_FILE_NAME "/PidController_"

using namespace std;


// public methods

PidController::PidController()
                            : m_sID("NA")
                            , m_dKp(0.0)
                            , m_dKd(0.0)
                            , m_dKi(0.0)
                            , m_dTimeStepSec(0.05)
                            , m_dDeadZoneAngleDU(10)
                            , m_dAccumulatedError(0)
                            , m_dLastMeasuredTimeSec(0)
{}


PidController::PidController(std::string sID, 
                            double kp, 
                            double kd, 
                            double ki,                            
                            double timeStepSec,
                            double deadZoneAngleDU)
                            : m_sID(sID)
                            , m_dKp(kp)
                            , m_dKd(kd)
                            , m_dKi(ki)
                            , m_dTimeStepSec(timeStepSec)
                            , m_dDeadZoneAngleDU(deadZoneAngleDU)
                            , m_dAccumulatedError(0)
                            , m_dLastMeasuredTimeSec(0)
{

    // string s_cwd(getcwd(NULL,0));
    // cout << "CWD is: " << s_cwd << endl;
    
    stringstream ss;
    ss << getenv("HOME") << DEFAULT_LOG_FILE_NAME << m_sID << ".log";
    m_ofslog.open(ss.str());
    ROS_ASSERT_MSG(m_ofslog, "Failed to open file %s", ss.str().c_str());

    m_ofslog << "Desired Angle (DU), Measured Time (s), Gimbal Angle(Deg), PlantInput (DU)";
}

PidController::~PidController()
{
    m_ofslog.close();
    ROS_INFO("Destructing PidController.");
}


double PidController::GetPlantInput(double dDesiredAngleDU, 
                                    double dMeasuredTimeSec,
                                    double dGimbalAngleDeg)
{
         
    double errorDU = NormalizeAngleDU(dDesiredAngleDU) - dGimbalAngleDeg * 10.0;
    
    stringstream ss;
    double plantInput = 0.0;
    
    if (abs(errorDU) < m_dDeadZoneAngleDU) 
    {
        ss << m_sID << "," << dDesiredAngleDU << "," << dMeasuredTimeSec << "," 
                            << dGimbalAngleDeg << "," << plantInput;
    }
    else
    { 
        plantInput = std::max( std::min( m_dKp * errorDU, GIMBAL_SPEED_LIMIT_DU), 
                               -GIMBAL_SPEED_LIMIT_DU);

        ss << m_sID << "," << dDesiredAngleDU << "," << dMeasuredTimeSec << "," 
           << dGimbalAngleDeg << "," << plantInput;
    }
    

    // m_dLastMeasuredTimeSec = dMeasuredTimeSec;

    return plantInput;		
    
}

// private methods      
double PidController::NormalizeAngleDU(double dAngleDU)
{
    while (dAngleDU <= -1800.0) { dAngleDU += 3600.0; }
    while (dAngleDU > 1800.0)  { dAngleDU -= 3600.0; }
    return dAngleDU;
}

                         
ostream& PidController::GetString(ostream& os)
{
    return os << "ID: " << m_sID 
              << ", Kp: " << m_dKp
              << ", Kd: " << m_dKd
              << ", Ki: " << m_dKi << endl;
}


//nonmember methods
ostream& operator<<(ostream& os, PidController& pid)
{
    return pid.GetString(os);
}




	
	
	/*



double getRequiredVelocityPID_yaw ( double desiredAngle_djiUnits, 
                                    double currentAngle_djiUnits, 
                                    int signOfMovement,
                                    // make positive if you started traveling in positive direction, 
                                    // negative otherwise
                                    bool& isUnwinding, 
                                    double tolerance_DjiUnits,
                                    double latest_dt,
                                    PIDController * pidInstance)
{

    double currentError = desiredAngle_djiUnits - currentAngle_djiUnits;
    bool needToUnwind = needToUniwndGimbal (desiredAngle_djiUnits,
                                            currentAngle_djiUnits, 
                                            signOfMovement, 
                                            tolerance_DjiUnits, 
                                            false);

    if (needToUnwind == false && isUnwinding == false)
    {
        currentError = limitAngleToPi_DjiUnits(currentError);
    } //uncomment this if you aren't limiting the angles to avoid rotations of more than 180 degrees
    else if (needToUnwind == true || isUnwinding == true) 
        //the isUnwinding prevents it from ceasing to unwind partway through, which causes oscillations 
    {    
        //testt = true;
        if(isUnwinding == false)
        {
            isUnwinding = true;
        }

        if (signOfMovement >= 1 && needToUnwind == true)
        {
            while(currentError >= 0.0) 
            {
                currentError -= 3600;
            }
        }
        else if (signOfMovement <= -1)
        {
            while(currentError <= 0.0) 
            {
                currentError += 3600;
            }
        }             
        
        //if it's gone too far. 
        // Too far and sign = 1 means it's gone too far clockwise, 
        // too far and sign = -1 means it's gone too far counterclockwise. 
        // So if the starting sign was 1, we want to always return a negative value if it's gone too far. 
        // If the starting sign was -1, we want to always return a positive value if it's gone too far.  
        // With the current set up, sign of the error is the sign of velocity, 
        // so we'd always want a negative error if starting sign was 1. 
    }

	double requiredVelocity =  (*pidInstance).calculateDesiredVelocity(currentError, latest_dt);
   
    return requiredVelocity;
	
}
		*/
