#include "gimbal_control/PidController.h"
#include "std_msgs/String.h"
#include <time.h>
#include <sstream>
#include <unistd.h>
#include <limits>

#define GIMBAL_SPEED_LIMIT_DPS 180.0   // 180 degrees per second
#define DEFAULT_LOG_FILE_NAME "/PidController_"

#define INITIAL_VALUE 10000000.0
 
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
                            , m_dPrevOutputDeg(0.0)
                            , m_dPrevErrorDeg(0.0)
                            , m_bIsPrevValueInitialized(false)
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
                            , m_dPrevOutputDeg(0.0)
                            , m_dPrevErrorDeg(0.0)
                            , m_bIsPrevValueInitialized(false)
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
    ss << rosHome << DEFAULT_LOG_FILE_NAME << m_sID << "_" << ros::WallTime::now() << ".log";
    m_ofslog.open(ss.str());
    ROS_ASSERT_MSG(m_ofslog, "Failed to open file %s", ss.str().c_str());

    m_ofslog << "#Time,Desired Angle(deg),Normlized Desired(deg),Adj Desired(deg),Gimbal Angle(deg),Error(deg),Outut Rate (DPS),Plant Input(DPS)" << endl;
    
}

double PidController::RunIntelligentControl(double dDesiredAngleDeg, double dGimbalAngleDeg)
{
    // This method has been designed particularly for yaw control. 
    // 1. The gimbal reading (dGimbalAngleDeg) is in [-180, 180]
    // 2. We don't want the gimbal to try to rotate to 359 degrees instead of -1 degree.
    
    static double dActualGimbalAngleDeg = INITIAL_VALUE; // set it with an unrealistic number.
        
    if (!m_bIsPrevValueInitialized)
    {
        m_bIsPrevValueInitialized = true;
        m_dPrevOutputDeg = dGimbalAngleDeg;
        ROS_INFO("Initialize dPrevGimbalAngle with %f\n", m_dPrevOutputDeg);
    }
        
    
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

       
    double dErrorDeg = dAdjustedDesiredAngleDeg - dActualGimbalAngleDeg;
    double dOutputRateDPS = -1.0 * (dActualGimbalAngleDeg - m_dPrevOutputDeg) / m_dTimeStepSec;
    
    double plantInputDPS = (abs(dErrorDeg) < m_dDeadZoneAngleDeg) ? 
                           0.0 : 
                           std::max( std::min( m_dKp*dErrorDeg + m_dKd*dOutputRateDPS, GIMBAL_SPEED_LIMIT_DPS), 
                                    -GIMBAL_SPEED_LIMIT_DPS);
    
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << ros::Time::now().toSec() << "," 
                << dDesiredAngleDeg << "," 
                << dNormalizedDesiredAngleDeg << "," 
                << dAdjustedDesiredAngleDeg << "," 
                << dGimbalAngleDeg << "," 
                << dErrorDeg << "," 
                << dOutputRateDPS << ","   
                << plantInputDPS << endl;

    m_dPrevOutputDeg = dActualGimbalAngleDeg;
    
    return plantInputDPS * 10.0;		
    
    
}
    
double PidController::RunNormalControl(double dDesiredAngleDeg, double dGimbalAngleDeg)
{     
    // The gimbal reading (dGimbalAngleDeg) is in (-180, 180].      

    if (!m_bIsPrevValueInitialized)
    {
        m_bIsPrevValueInitialized = true;
        m_dPrevOutputDeg = dGimbalAngleDeg;
        ROS_INFO("Initialize dPrevGimbalAngle with %f\n", m_dPrevOutputDeg);
    }
    
    // Normalize desired angle so that the desired angle is always in (-180, 180].
    double dNormalizedDesiredAngleDeg = NormalizeAngleDeg(dDesiredAngleDeg);   
    
    double dErrorDeg = dNormalizedDesiredAngleDeg - dGimbalAngleDeg;
      
    double dOutputRateDPS = -1.0 * (dGimbalAngleDeg - m_dPrevOutputDeg) / m_dTimeStepSec;
        
    double plantInputDPS = (abs(dErrorDeg) < m_dDeadZoneAngleDeg) ? 
                           0.0 : 
                           std::max( std::min( m_dKp*dErrorDeg + m_dKd*dOutputRateDPS, GIMBAL_SPEED_LIMIT_DPS), 
                                    -GIMBAL_SPEED_LIMIT_DPS
                                   );
    
    m_ofslog    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << ros::Time::now().toSec() << "," 
                << dDesiredAngleDeg << "," 
                << dNormalizedDesiredAngleDeg << "," 
                << dNormalizedDesiredAngleDeg << "," 
                << dGimbalAngleDeg << "," 
                << dErrorDeg << "," 
                << dOutputRateDPS << ","   
                << plantInputDPS << endl;

    m_dPrevOutputDeg = dGimbalAngleDeg;

    return plantInputDPS * 10.0;		
}
    
    

double PidController::NormalizeAngleAboutDeg(double dAngleDeg, double dCenter)
{
    while (dAngleDeg <= -180.0 + dCenter) { dAngleDeg += 360.0; }
    while (dAngleDeg > 180.0 + dCenter)  { dAngleDeg -= 360.0; }
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



