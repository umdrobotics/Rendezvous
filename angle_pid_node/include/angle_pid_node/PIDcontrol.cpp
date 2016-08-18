
#include <stdlib.h>    //for absolute value
 
//if it receives a command like "go to angle 181 degrees", when it reaches 180.1,
//it will change the sensor measure from 180.1 to -179.9
//and the PID will try to spin another 360.9 degrees! It will continue until the gimbal can't physically turn any more
//stop it by limiting it to +- 180 degrees
//this function will be used both when calculating the error and when recieving the commands from the subscriber
double limitAngleToPi_DjiUnits(double angle_djiUnits)
  {
    while(angle_djiUnits < -1800.0)
        {angle_djiUnits += 3600.0;}
    while(angle_djiUnits > 1800.0)
        {angle_djiUnits -= 3600.0;}
    return angle_djiUnits;
   }



//another option that might be better at preventing mechanical stoppage is to
// only allow it to swing within +- 175 degrees, and eliminate "shortcutting"
//So if You went from -175 to +175 you'd swing the full positive 350, not -10
//that would eliminate a lot of the mechanical stoppage problems
double limitAngleTo175Degs_DjiUnits(double angle_djiUnits)
  {
    angle_djiUnits = limitAngleToPi_DjiUnits( angle_djiUnits);
    if (angle_djiUnits > 1750.0)
            {angle_djiUnits = 1750.0;}
    else if (angle_djiUnits < -1750.0)
             {angle_djiUnits = -1750.0;}   

    return angle_djiUnits;
   }
 

//need following behavior patterns:
//All angles will be between +-180 degrees (can be handled with other fucntions)
// Angles should use shortcuts (ie, going from 175 to -175 should use a +10 degree swing, not -350
// Will avoid mechanical lockup by using a full swing, instead of shortcutting, when the resulting angle would send it beyond 270 degrees of total travel.
//This can be tracked as follows:
    // If you've been traveling in a positive direction from 0, then sign of angle changes, and then the desired angle is between 90 and -90, you need to "unwind" (note that some of this would happen anyway, and I'm not sure what the sign of the movement would need to be)
     // If you've been traveling in a negative direction from 0, then sign of angle changes, and then the desired angle is between 90 and -90, you need to "unwind" (note that some of this would happen anyway, and I'm not sure what the sign of the movement would need to be)
  //NOTE: I assume that the desired and current angles are already between +- 180
bool needToUniwndGimbal (double desiredAngle_DjiUnits ,double currentAngle_DjiUnits ,int startingSign/*make positive if you started traveling in positive direction, negative in negative direction, 0 if not outside of a small range from start*/ ,double tolerance_DjiUnits, bool verbose) //tolerance is how close to the 180 -180 dividing line you want to get. So a tolerance of 50 (50 DjiUnits, 5 degrees) would consider that zone to be between 175 and -175 (as in 175 to 180 then -180 to -175, only a 10 degree zone)
    {   

      if(verbose)
            {cout <<"desired angle (tenths of degree" << desiredAngle_DjiUnits <<" bool1 :" << (bool)(startingSign >=1) << " bool2A :" <<  (bool)(abs(currentAngle_DjiUnits) > 1800 - tolerance_DjiUnits) << " bool2B :" << (bool)(currentAngle_DjiUnits < -1.0*(900-tolerance_DjiUnits)) << " bool3 :" << (bool)( -900 <= desiredAngle_DjiUnits && desiredAngle_DjiUnits <=900) << "\n";  }

//note: based on testing in python, a negative speed in yaw is always counterclockwise (from the drone's perspective)
      if (startingSign >=1 && (  (abs(currentAngle_DjiUnits) > 1800 - tolerance_DjiUnits) || currentAngle_DjiUnits < -1.0*(900-tolerance_DjiUnits) )&& ( -900 <= desiredAngle_DjiUnits && desiredAngle_DjiUnits <=900)  )
        {return true;}
      else if (startingSign <= -1 && (  (abs(currentAngle_DjiUnits) > 1800 - tolerance_DjiUnits) || currentAngle_DjiUnits > (900-tolerance_DjiUnits) )&& ( -900 <= desiredAngle_DjiUnits && desiredAngle_DjiUnits <=900)  )
        {return true;}        
      else
        {return false;}
    }


using namespace std;
  
  

class PIDController {
 public: //declare everything public right now for convenience
	double Kp = 3.0; //initial guess of 0.5 was somewhat slow, try 0.8 was whippy, 2 works well, try 4 for more speed
	double Kd = 0.0; //initial guess
	double Ki = 0.0; //initial guess was 0.0
	
	double deadZone_DjiUnits = 10.0 ;//anything less than 1 degrees, we don't care
	
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
double getRequiredVelocityPID(double desiredAngle_djiUnits, double currentAngle_djiUnits ,double latest_dt ,PIDController * pidInstance)
	{
		double currentError = desiredAngle_djiUnits - currentAngle_djiUnits;

		return (*pidInstance).calculateDesiredVelocity(currentError, latest_dt);
	
	}

double getRequiredVelocityPID_yaw(double desiredAngle_djiUnits, double currentAngle_djiUnits, int signOfMovement/*make positive if you started traveling in positive direction, negative otherwise*/ ,bool& isUnwinding, double tolerance_DjiUnits ,double latest_dt ,PIDController * pidInstance)
	{
//bool testt =false;
//note: based on testing in python, a negative speed in yaw is always counterclockwise (from the drone's perspective)
		double currentError = desiredAngle_djiUnits - currentAngle_djiUnits;
        bool needToUnwind = needToUniwndGimbal (desiredAngle_djiUnits ,currentAngle_djiUnits, signOfMovement, tolerance_DjiUnits, false);
        if (needToUnwind == false && isUnwinding == false)
            {currentError = limitAngleToPi_DjiUnits(currentError);} //uncomment this if you aren't limiting the angles to avoid rotations of more than 180 degrees

        else if(needToUnwind == true || isUnwinding == true) //the isUnwinding prevents it from ceasing to unwind partway through, which causes oscillations 
            {    //testt = true;
                if(isUnwinding == false)
                    {isUnwinding = true;}


                if (signOfMovement >= 1 && needToUnwind == true)
                        {while(currentError >= 0.0) 
                            {currentError -= 3600;}
                        testt=true;
                        }
               else if (signOfMovement <= -1)
                        {while(currentError <= 0.0) 
                            {currentError += 3600;}
                        }             
              //if it's gone too far. Too far and sign = 1 means it's gone too far clockwise, too far and sign = -1 means it's gone too far counterclockwise. So if the starting sign was 1, we want to always return a negative value if it's gone too far. If the starting sign was -1, we want to always return a positive value if it's gone too far.  With the current set up, sign of the error is the sign of velocity, so we'd always want a negative error if starting sign was 1. 
            }

		double requiredVelocity =  (*pidInstance).calculateDesiredVelocity(currentError, latest_dt);
        //if(testt){requiredVelocity = -1.0*abs(requiredVelocity);}
        return requiredVelocity;
	
	}
