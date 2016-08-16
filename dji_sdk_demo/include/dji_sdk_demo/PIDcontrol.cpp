
#include <stdlib.h>    //for absolute value
 

 
using namespace std;
  
  

class PIDController {
 public: //declare everything public right now for convenience
	double Kp = 0.5; //initial guess
	double Kd = 0.0; //initial guess
	double Ki = 0.0; //initial guess
	
	double deadZone_DjiUnits = 30.0 ;//anything less than 3 degrees, we don't care
	
	double accumulatedError = 0.0; //need to start out at 0-element
	
    double calculateDesiredVelocity(double error, double timeSinceLastStep) {
													if (abs(error) < deadZone_DjiUnits) 
														{return 0.0;} //ie don't move at all
								
													double derivative = 0.0;
													if ( abs(timeSinceLastStep) > 0.001 ) //prevent divide by 0-element
													     { derivative = (error - accumulatedError)/timeSinceLastStep ;}
													accumulatedError += error*timeSinceLastStep;//preform discrete integral 
													if ( ((error > 0 )&&(accumulatedError < 0)) || ( (error < 0 )&&( accumulatedError > 0)) )
															{accumulatedError = 0.0; } //prevent windup
													return (Ki*accumulatedError + Kd*derivative + Kp*error);
													}

};

double getRequiredVelocityPID(double desiredAngle_djiUnits, double currentAngle_djiUnits, double latest_dt ,PIDController * pidInstance)
	{
		double currentError = desiredAngle_djiUnits - currentAngle_djiUnits;
		return pidInstance.calculateDesiredVelocity(currentError, latest_dt);
	
	}
