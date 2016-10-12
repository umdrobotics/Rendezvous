#include "gimbal_control/PidController.h"
#include "std_msgs/String.h"
#include <sstream>
#include <unistd.h>
#include <limits>

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
                            , m_bIsIntelligentControl(false)
{    
    ConstructorHelper();   
}


PidController::PidController(std::string sID, 
                            double kp, 
                            double kd, 
                            double ki,                            
                            double timeStepSec,
                            double deadZoneAngleDU,
                            bool isIntelligentControl)
                            : m_sID(sID)
                            , m_dKp(kp)
                            , m_dKd(kd)
                            , m_dKi(ki)
                            , m_dTimeStepSec(timeStepSec)
                            , m_dDeadZoneAngleDU(deadZoneAngleDU)
                            , m_bIsIntelligentControl(isIntelligentControl)
{
    ConstructorHelper();   
}

PidController::~PidController()
{
    m_ofslog.close();
    ROS_INFO("Destructing PidController.");
}


double PidController::GetPlantInput(double dDesiredAngleDU, 
                                    double dGimbalAngleDeg)
{
    
    return m_bIsIntelligentControl ? RunIntelligentControl(dDesiredAngleDU, dGimbalAngleDeg)
                                   : RunNormalControl(dDesiredAngleDU, dGimbalAngleDeg);       
    
}


ostream& PidController::GetString(ostream& os)
{
    return os << "ID:" << m_sID 
              << ", Kp:" << m_dKp
              << ", Kd:" << m_dKd
              << ", Ki:" << m_dKi
              << ", Ts:" << m_dTimeStepSec
              << ", IntelligentControl:" << m_bIsIntelligentControl;
}

// private methods      

void PidController::ConstructorHelper()
{

    char* rosHome = getenv ("ROS_HOME");
    ROS_ASSERT_MSG(rosHome, "Can't find the environment variable, ROS_HOME");
    
    stringstream ss;
    ss << rosHome << DEFAULT_LOG_FILE_NAME << m_sID << ".log";
    m_ofslog.open(ss.str());
    ROS_ASSERT_MSG(m_ofslog, "Failed to open file %s", ss.str().c_str());

    m_ofslog << "#Time,Desired Angle (DU),Normlized Desired(Deg), Error (DU), Gimbal Angle(Deg), PlantInput (DU)" << endl;
    
}

double PidController::RunIntelligentControl(double dDesiredAngleDU, double dGimbalAngleDeg)
{
     
    // 1. The gimbal reading (dGimbalAngleDeg) is in [-180, 180]
    // 2. We don't want the gimbal rotate ~360 degrees 
    //    when it is ~179 deg and desired angle is ~181.
    // 3. When desired angle is 180 and there is a small overshoot or disturbance,
    //    the gimbal angle could be 181 degrees, which is angle measurement of -179.
    //    If it happens we don't want the gimbal rotate 360 degrees to be 180. 

    static double dActualGimbalAngleDeg = 10000.0; // set it with an unrealistic number.
    
    
    if (dActualGimbalAngleDeg == 10000.0)
    {
        dActualGimbalAngleDeg = dGimbalAngleDeg;
    }
    else if (dActualGimbalAngleDeg - dGimbalAngleDeg > 300.0) 
    {   // actual angle is greater than 180, but reading is near -180.
        
        // We need to adjust the actual angle to be greater than 180.
        dActualGimbalAngleDeg = dGimbalAngleDeg + 360.0;     
    }
    else if (dActualGimbalAngleDeg - dGimbalAngleDeg < -300.0) 
    {   // actual angle is less than -180, but reading is near 180.
     
        // We need to adjust the actual angle to be less than -180.
        dActualGimbalAngleDeg = dGimbalAngleDeg - 360.0;     
    }
    else
    {
        dActualGimbalAngleDeg = dGimbalAngleDeg; 
    }
        
    // Normalize desired angle so that the difference between the two is always less than |180.0|
    double dNormalizedDesiredAngleDeg = NormalizeAngleAboutDeg(dDesiredAngleDU/10,                  
                                                               dActualGimbalAngleDeg);   


    double dAdjustedDesiredAngleDeg = dNormalizedDesiredAngleDeg;

    if (dNormalizedDesiredAngleDeg > 270.0)
    {
        // If the normalized desired angle is greater than 270,
        // the actual angle could be too large. So we need to unwind. 
        dAdjustedDesiredAngleDeg -= 360.0;      
    }
    else if (dNormalizedDesiredAngleDeg < -270.0)
    {                                          
        // If the normalized desired angle is less than -270,
        // the actual angle could be too small. So we need to unwind. 
        dAdjustedDesiredAngleDeg += 360.0;
    }
   
    double dErrorDU = (dAdjustedDesiredAngleDeg - dActualGimbalAngleDeg) * 10;
    
    double plantInput = (abs(dErrorDU) < m_dDeadZoneAngleDU) ? 
                        0.0 : 
                        std::max( std::min( m_dKp * dErrorDU, GIMBAL_SPEED_LIMIT_DU), 
                                    -GIMBAL_SPEED_LIMIT_DU);
    
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << ros::Time::now().toSec() << "," 
                << dDesiredAngleDU << "," 
                << dNormalizedDesiredAngleDeg << "," 
                << dAdjustedDesiredAngleDeg << "," 
                << dErrorDU << "," 
                << dGimbalAngleDeg << "," 
                << plantInput << endl;

    return plantInput;		
    
    
}
    
double PidController::RunNormalControl(double dDesiredAngleDU, double dGimbalAngleDeg)
{     
    // The gimbal reading (dGimbalAngleDeg) is in (-180, 180].      

    // Normalize desired angle so that the desired angle is always in (-180, 180].
    double dNormalizedDesiredAngleDeg = NormalizeAngleDeg(dDesiredAngleDU/10);   
    

    double dErrorDU = (dNormalizedDesiredAngleDeg - dGimbalAngleDeg) * 10;
    
    double plantInput = (abs(dErrorDU) < m_dDeadZoneAngleDU) ? 
                        0.0 : 
                        std::max( std::min( m_dKp * dErrorDU, GIMBAL_SPEED_LIMIT_DU), 
                                    -GIMBAL_SPEED_LIMIT_DU);
    
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << ros::Time::now().toSec() << "," 
                << dDesiredAngleDU << "," 
                << dNormalizedDesiredAngleDeg << "," 
                // The following line should be adjusted desired angle,
                // but in this case it is the same as normalized angle.
                << dNormalizedDesiredAngleDeg << ","   
                << dErrorDU << "," 
                << dGimbalAngleDeg << "," 
                << plantInput << endl;

    return plantInput;		
}
    
    

double PidController::NormalizeAngleAboutDeg(double dAngleDeg, double dCenter)
{
    while (dAngleDeg <= -180.0 + dCenter) { dAngleDeg += 360.0; }
    while (dAngleDeg > 180.0 + dCenter)  { dAngleDeg -= 360.0; }
    return dAngleDeg;
    return dAngleDeg;
}

double PidController::NormalizeAngleDeg(double dAngleDeg)
{
    return NormalizeAngleAboutDeg(dAngleDeg, 0);
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
