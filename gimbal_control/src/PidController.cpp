#include "gimbal_control/PidController.h"
#include "std_msgs/String.h"
#include <sstream>
#include <unistd.h>
#include <limits>

#define GIMBAL_SPEED_LIMIT_DU 1800.0
#define DEFAULT_LOG_FILE_NAME "/PidController_"

#define INITIAL_VALUE 100000.0 // initial static value which is unrealistic.
 
using namespace std;

// public methods

PidController::PidController()
                            : m_sID("NA")
                            , m_dKp(0.0)
                            , m_dKd(0.0)
                            , m_dKi(0.0)
                            , m_dTimeStepSec(0.02)
                            , m_dDeadZoneAngleDeg(10)
                            , m_bIsIntelligentControl(false)
{    
    ConstructorHelper();   
}

PidController::PidController(std::string sID, 
                            double kp, 
                            double kd, 
                            double ki,                            
                            double timeStepSec,
                            double deadZoneAngleDeg,
                            bool isIntelligentControl)
                            : m_sID(sID)
                            , m_dKp(kp)
                            , m_dKd(kd)
                            , m_dKi(ki)
                            , m_dTimeStepSec(timeStepSec)
                            , m_dDeadZoneAngleDeg(deadZoneAngleDeg)
                            , m_bIsIntelligentControl(isIntelligentControl)
{
    ConstructorHelper();   
}

PidController::~PidController()
{
    m_ofslog.close();
    ROS_INFO("Destructing PidController.");
}

double PidController::GetPlantInput(double dDesiredAngleDeg, 
                                    double dGimbalAngleDeg)
{
    
    return m_bIsIntelligentControl ? RunIntelligentControl(dDesiredAngleDeg, dGimbalAngleDeg)
                                   : RunNormalControl(dDesiredAngleDeg, dGimbalAngleDeg);       
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

    m_ofslog << "#Time,Desired Angle (Deg),Normlized Desired(Deg), Adj Desired(Deg), Error (Deg), Gimbal Angle(Deg), PlantInput (DU)" << endl;
    
}

double PidController::RunIntelligentControl(double dDesiredAngleDeg, double dGimbalAngleDeg)
{
     
    // 1. The gimbal reading (dGimbalAngleDeg) is in [-180, 180]
    // 2. We don't want the gimbal rotate ~360 degrees 
    //    when it is ~179 deg and desired angle is ~181.
    // 3. When desired angle is 180 and there is a small overshoot or disturbance,
    //    the gimbal angle could be 181 degrees, which is angle measurement of -179.
    //    If it happens we don't want the gimbal rotate 360 degrees to be 180. 

    static double dActualGimbalAngleDeg = INITIAL_VALUE; // set it with an unrealistic number.
    static double dPrevGimbalAngleDeg = INITIAL_VALUE;   // set it with an unrealistic number.
    
    if (INITIAL_VALUE == dActualGimbalAngleDeg)   // the very first time this if statement is called.
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
    double dNormalizedDesiredAngleDeg = NormalizeAngleAboutDeg(dDesiredAngleDeg,                  
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

    if (INITIAL_VALUE == dPrevGimbalAngleDeg) // the very first time this if statement is called.
    {
        dPrevGimbalAngleDeg = dActualGimbalAngleDeg;
    }

       
    double dErrorDU = 10.0 * (dAdjustedDesiredAngleDeg - dActualGimbalAngleDeg);
    double dErrorRateDU = 10.0 * (dActualGimbalAngleDeg - dPrevGimbalAngleDeg) / m_dTimeStepSec;
    
    double plantInputDU = (abs(dErrorDU) < m_dDeadZoneAngleDeg*10) ? 
                        0.0 : 
                        std::max( std::min( m_dKp*dErrorDU + m_dKd*dErrorRateDU, GIMBAL_SPEED_LIMIT_DU), 
                                    -GIMBAL_SPEED_LIMIT_DU);
    
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << ros::Time::now().toSec() << "," 
                << dDesiredAngleDeg << "," 
                << dNormalizedDesiredAngleDeg << "," 
                << dAdjustedDesiredAngleDeg << "," 
                << dErrorDU << "," 
                << dGimbalAngleDeg << "," 
                << plantInputDU << endl;

    dPrevGimbalAngleDeg = dActualGimbalAngleDeg;
    
    return plantInputDU;		
    
    
}
    
double PidController::RunNormalControl(double dDesiredAngleDeg, double dGimbalAngleDeg)
{     
    // The gimbal reading (dGimbalAngleDeg) is in (-180, 180].      


    static double dPrevGimbalAngleDeg = INITIAL_VALUE;   // set it with an unrealistic number.
    
    if (INITIAL_VALUE == dPrevGimbalAngleDeg) // the very first time this if statement is called.
    {
        dPrevGimbalAngleDeg = dGimbalAngleDeg;
    }
    
    // Normalize desired angle so that the desired angle is always in (-180, 180].
    double dNormalizedDesiredAngleDeg = NormalizeAngleDeg(dDesiredAngleDeg);   
    
    double dErrorDU = 10.0 * (dNormalizedDesiredAngleDeg - dGimbalAngleDeg);
    double dErrorRateDU = 10.0 * (dGimbalAngleDeg - dPrevGimbalAngleDeg) / m_dTimeStepSec;
        
    double plantInputDU = (abs(dErrorDU) < m_dDeadZoneAngleDeg*10) ? 
                        0.0 : 
                        std::max( std::min( m_dKp*dErrorDU + m_dKd*dErrorRateDU, GIMBAL_SPEED_LIMIT_DU), 
                                    -GIMBAL_SPEED_LIMIT_DU);
    
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << ros::Time::now().toSec() << "," 
                << dDesiredAngleDeg << "," 
                << dNormalizedDesiredAngleDeg << "," 
                // The following line should be adjusted desired angle,
                // but in this case it is the same as normalized angle.
                << dNormalizedDesiredAngleDeg << ","   
                << dErrorDU << "," 
                << dGimbalAngleDeg << "," 
                << plantInputDU << endl;

    dPrevGimbalAngleDeg = dGimbalAngleDeg;

    return plantInputDU;		
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



