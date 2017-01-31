

#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include "ros/ros.h" //note this must come before including 
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
 
 //modified from http://www.robot-home.it/blog/en/software/ball-tracker-con-filtro-di-kalman/

// Module "core"
//#include <opencv2/core/core.hpp>
 
// Module "highgui"
//#include <opencv2/highgui/highgui.hpp>
 
// Module "imgproc"
//#include <opencv2/imgproc/imgproc.hpp>
 
// Module "video"
//#include <opencv2/video/video.hpp>
 
// Output
//#include <cstdlib>
 
// Vector
//#include <vector>
 


#include <cmath>

class KalmanFilter
{

private: // members
	
    int m_nStateSize;
    int m_nMeasurementSize;
    int m_nInputSize;
    unsigned int m_nCVType;
    
    bool m_bHasFirstMeasurementProcessed;
    
    cv::KalmanFilter m_kf;
    cv::Mat m_matCurrentState;
    cv::Mat m_matMeasurement;
    
    
private: // methods
   
    void ConstructorHelper();

private: // NOT IMPLEMENTED. Do not implement until necessary
	KalmanFilter(const KalmanFilter&);  // copy constructor
	KalmanFilter& operator=(const KalmanFilter&); // assignment

public:

    // ctor
    KalmanFilter();
    
    KalmanFilter(int nStateSize, 
                 int nMeasurementSize,
                 int nInputSize,
                 unsigned int nCVType
                 );
    
    cv::Mat ProcessMeasurement(double dT, double xIn, double yIn);

    virtual ~KalmanFilter();
    
    std::ostream& GetString(std::ostream& os);
    
    std::ostream& GetCurrentState(std::ostream& os);
};

// non-member methods
std::ostream& operator<<(std::ostream& os, KalmanFilter& kf);


#endif

