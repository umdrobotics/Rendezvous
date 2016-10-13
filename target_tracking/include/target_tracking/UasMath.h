

#ifndef _UAS_MATH_H
#define _UAS_MATH_H

/**  @file
     @brief Math library for Unmanned Aerial System

     @author Stan Baek
 */

#include <cmath>

#define RADIANS_PER_DEGREE (M_PI/180.0);
#define DEGREES_PER_RADIAN (180.0/M_PI);

class UasMath
{



public:

static inline double ConvertDeg2Rad(double angle_deg)
{
    return angle_deg * RADIANS_PER_DEGREE;
}

static inline double ConvertRad2Deg(double angle_rad)
{
    return angle_rad * DEGREES_PER_RADIAN;
}

};
#endif // _UTM_H
