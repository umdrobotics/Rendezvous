#include <iostream>

namespace HandleLanding
{
    double NON_LANDING_LIMIT_METERS = 0.5 ;// altitude below this means you land
    double MAXIMUM_NORTHING_DISPLACEMENT_FOR_LANDING_METERS = 0.5;
	double MAXIMUM_EASTING_DISPLACEMENT_FOR_LANDING_METERS = 0.5; 

	int TIMES_WITHIN_TOLERANCE = 0;
	int TIMES_IN_TOLERANCE_BEFORE_DESCENT = 3; 

	double getHeightAboveTargetForDescent(double distanceAboveTarget_meters)
	{
	//want to find out a reasonable unit to subtract from a height. 
	// I'm not using a PID controller because I want a smooth, reasonable descent with no risk of it overshooting (into the ground)
	// I understand there is some difference in behavior around the edge cases, but they will always place it in the next "zone" down 
	// this will ensure a smooth transition down


		if (distanceAboveTarget_meters <=2.0)
			{return 0.0 ;} //This is the signal to land
		else if ( 2.0 < distanceAboveTarget_meters && distanceAboveTarget_meters <= 5.0 )
				{return distanceAboveTarget_meters - 1.0;}
		else if (5.0 < distanceAboveTarget_meters && distanceAboveTarget_meters <= 10.0)
				{return distanceAboveTarget_meters - 2.0;}
		else if (10.0 < distanceAboveTarget_meters && distanceAboveTarget_meters <= 20.0)
				{return distanceAboveTarget_meters - 4.0;}
		else if (20.0 < distanceAboveTarget_meters && distanceAboveTarget_meters <= 130.0)
				{return distanceAboveTarget_meters - 10.0;}
		else if (130.0 < distanceAboveTarget_meters && distanceAboveTarget_meters <= 1200.0)
				{return distanceAboveTarget_meters - 100.0;}
		else	
			{
			std::cout <<"ALTITUDE ERROR: in lowerAltitue() in handleLanding.h : Input of : " << distanceAboveTarget_meters << " meters not in an acceptable range!\n" ;
			return distanceAboveTarget_meters;
			}
		
	}

	double getNewAltitudeForDescent(
						 double deltaNorth_meters, 
						 double deltaEast_meters,
						 double altitude_meters, 
						 double distanceAboveTarget_meters
		 
                         ,bool& shouldILand //THIS IS AN OUTPUT VARIABLE 		 
						 )
		{
		 shouldILand = false; //only change this later if you should land	
		 if (abs(deltaNorth_meters) <= abs(MAXIMUM_NORTHING_DISPLACEMENT_FOR_LANDING_METERS) && abs(deltaEast_meters) <= abs(MAXIMUM_EASTING_DISPLACEMENT_FOR_LANDING_METERS))
			{
			 TIMES_WITHIN_TOLERANCE ++;	
				
			}
		 else 
			{
				
			 TIMES_WITHIN_TOLERANCE = 0;	
			
			}
			
		 if (TIMES_WITHIN_TOLERANCE >= TIMES_IN_TOLERANCE_BEFORE_DESCENT)
				{ 
				  double targetHeight_meters =  altitude_meters - distanceAboveTarget_meters ;// this also accounts for elevation changes of the terrain. So if altitude is 10 meters, and we're only 8 meters above the target, the target is 2 meters high	
				  double newHeightAboveTarget_meters = getHeightAboveTargetForDescent(distanceAboveTarget_meters);
				  if(newHeightAboveTarget_meters < NON_LANDING_LIMIT_METERS)
				     {shouldILand = true;}
				 
				  double newAltitude_meters = newHeightAboveTarget_meters + targetHeight_meters; 
				  return newAltitidue_meters;
				}
		  
		 else
				{
				  return altitude_metersde; 			  	
				}
				
		}
		
	  
	 void pauseDescent()
		{TIMES_WITHIN_TOLERANCE = 0;}

		
}

