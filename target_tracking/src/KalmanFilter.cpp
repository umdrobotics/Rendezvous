
#include "target_tracking/KalmanFilter.h"
// Module "core"

#include <sstream>  // stringstream, ...
#include <iostream> // std::cout, std::end, ...
#include <iomanip>  // std::setprecision
// #include <signal.h>


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
 
using namespace std;
 
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

KalmanFilter::KalmanFilter()
                            : m_nStateSize(4)
                            , m_nMeasurementSize(2)
                            , m_nInputSize(0)
                            , m_nCVType(CV_64F)
                            , m_bHasFirstMeasurementProcessed(false)
                            , m_kf(cv::KalmanFilter(4, 2, 0, CV_64F))
                            , m_matCurrentState(cv::Mat(4, 1, CV_64F))
                            , m_matMeasurement(cv::Mat(4, 1, CV_64F))
{
    ConstructorHelper();  
} 

 
KalmanFilter::KalmanFilter(int nStateSize, 
                            int nMeasurementSize,
                            int nInputSize,
                            unsigned int nCVType)
                            : m_nStateSize(nStateSize)
                            , m_nMeasurementSize(nMeasurementSize)
                            , m_nInputSize(nInputSize)
                            , m_nCVType(nCVType)
                            , m_bHasFirstMeasurementProcessed(false)
                            , m_kf(cv::KalmanFilter(nStateSize, nMeasurementSize, nInputSize, nCVType))
                            , m_matCurrentState(cv::Mat(m_nStateSize, 1, m_nCVType))
                            , m_matMeasurement(cv::Mat(m_nStateSize, 1, m_nCVType))
{
    ConstructorHelper();  
}


KalmanFilter::~KalmanFilter()
{
    ROS_INFO("Destructing KalmanFilter.");
}

ostream& KalmanFilter::GetString(ostream& os)
{
    return os << "State Size:" << m_nStateSize 
              << ", Measurement Size:" << m_nMeasurementSize
              << ", Input Size:" << m_nInputSize
              << ", CV Type:" << m_nCVType
              << ", Initialized?:" << m_bHasFirstMeasurementProcessed;
}

ostream& KalmanFilter::GetCurrentState(ostream& os)
{
    
    return os << m_matCurrentState;
    
}



cv::Mat KalmanFilter::ProcessMeasurement(double dT, double targetX, double targetY)
{
    
   // Transition State Matrix A
   // Note: set dT at each processing step!
   // [ 1 0 dT 0  ]
   // [ 0 1 0  dT ]
   // [ 0 0 1  0  ]
   // [ 0 0 0  1  ]
    m_kf.transitionMatrix.at<double>(2) = dT;
    m_kf.transitionMatrix.at<double>(7) = dT;
 
    // Kalman Update

    cv::Mat measurement(m_nMeasurementSize, 1, CV_64F); //(measSize, 1, type);
	measurement.at<double>(0) = targetX;
	measurement.at<double>(1) = targetY;

    ROS_INFO_STREAM("StatePre 1: " << m_kf.statePre);
	ROS_INFO_STREAM("StatePost 1: " << m_kf.statePost);


    if(!m_bHasFirstMeasurementProcessed){
        
        m_kf.statePre.at<double>(0) = targetX;
        m_kf.statePre.at<double>(1) = targetY;
        m_kf.statePre.at<double>(2) = 0.0;
        m_kf.statePre.at<double>(3) = 0.0;
        
        cv::setIdentity(m_kf.errorCovPre);
        cv::setIdentity(m_kf.errorCovPost, cv::Scalar::all(.1));
        m_bHasFirstMeasurementProcessed = true;
    }
    else
    { 
        m_kf.predict();
    }
 
 	cv::Mat state = m_kf.correct(measurement); 

    ROS_INFO_STREAM("StatePre 2: " << m_kf.statePre);
	ROS_INFO_STREAM("StatePost 2: " << m_kf.statePost);

	return state;
    // cv::Mat state(m_nStateSize, 1, m_nCVType); 
    // cv::Mat state = m_kf.predict();
    
    // if(!m_bHasFirstMeasurementProcessed)
    // {
    //     state.at<double>(0) = targetX;
    //     state.at<double>(1) = targetY;
    //     state.at<double>(2) = 0;
    //     state.at<double>(3) = 0;
    // }			
        
    //cout << "State post:" << endl << state << endl;   
    // return state ;
} 



cv::Mat loopStepWebWithoutMeasurement(cv::KalmanFilter kf, double dT , cv::Mat latestState)
{ 
     //for some reason, if we just assign kf.statePost = latestState directly, it crashes
    kf.statePost.at<double>(0) = latestState.at<double>(0); 
    kf.statePost.at<double>(1) = latestState.at<double>(1); 
    kf.statePost.at<double>(2) = latestState.at<double>(2); 
    kf.statePost.at<double>(3) = latestState.at<double>(3); 

    // [1 0 dt 0]
	// [0 1 0 dt]
	// [0 0 1 0]
	// [0 0 0 1]
	//now update dt
    
    kf.transitionMatrix.at<double>(2) = dT;
    kf.transitionMatrix.at<double>(7) = dT;
    
    cv::Mat state(4,1,CV_64F); //(statesize,1,type);
    state = kf.predict();
		
    //cout << "State post no measurement:" << endl << state << endl;   
    return state ;
} ///end function



// private methods 
    
void KalmanFilter::ConstructorHelper()
{    
   // Transition State Matrix F
   // Note: set dT at each processing step!
   // [ 1 0 dT 0  ]
   // [ 0 1 0  dT ]
   // [ 0 0 1  0  ]
   // [ 0 0 0  1  ]
   cv::setIdentity(m_kf.transitionMatrix);
 
   // Measure Matrix H
   // [ 1 0 0 0 ]
   // [ 0 1 0 0 ]
   m_kf.measurementMatrix = cv::Mat::zeros(m_nMeasurementSize, m_nStateSize, m_nCVType);
   m_kf.measurementMatrix.at<double>(0) = 1.0f;
   m_kf.measurementMatrix.at<double>(5) = 1.0f;
 
   // Process Noise Covariance Matrix Q
   // [ Ex 0  0    0    ]
   // [ 0  Ey 0    0    ]
   // [ 0  0  Ev_x 0    ]
   // [ 0  0  0    Ev_y ]
   cv::setIdentity(m_kf.processNoiseCov, cv::Scalar(2.5e-1));
 
   // Measures Noise Covariance Matrix R
   // [ Ex 0  ]
   // [ 0  Ey ]
   //cv::setIdentity(m_kf.measurementNoiseCov, cv::Scalar(1e-1));
 	cv::setIdentity(m_kf.measurementNoiseCov, cv::Scalar(4.0));
 
}



//nonmember methods
ostream& operator<<(ostream& os, KalmanFilter& kf)
{
    return kf.GetString(os);
}





//this has good insight into how to do found not found control, test it later
/*
double loopStep(cv::KalmanFilter kf, double dT, )
{ 

if (found)
      {  // [1 0 dt 0]
	// [0 1 0 dt]
	// [0 0 1 0]
	// [0 0 0 1]
	//now update dt
         // >>>> Matrix A
         kf.transitionMatrix.at<double>(2) = dT;
         kf.transitionMatrix.at<double>(7) = dT;
         // <<<< Matrix A


         cout << "dT:" << endl << dT << endl;
 
         state = kf.predict();
         cout << "State post:" << endl << state << endl;            
 
     // >>>>> Kalman Update
      if (balls.size() == 0)
      {
         notFoundCount++;
         cout << "notFoundCount:" << notFoundCount << endl;          if( notFoundCount >= 10 )
         {
            found = false;
         }
         else
            kf.statePost = state;
      }
      else
      {
         notFoundCount = 0;
 
         meas.at(0) = ballsBox[0].x + ballsBox[0].width / 2;
         meas.at(1) = ballsBox[0].y + ballsBox[0].height / 2;
         //meas.at(2) = (float)ballsBox[0].width;
         //meas.at(3) = (float)ballsBox[0].height;
 
         if (!found) // First detection!
         {
            // >>>> Initialization
            kf.errorCovPre.at(0) = 1; // px
            kf.errorCovPre.at(7) = 1; // px
            kf.errorCovPre.at(14) = 1;
            kf.errorCovPre.at(21) = 1;
            kf.errorCovPre.at(28) = 1; // px
            kf.errorCovPre.at(35) = 1; // px
 
            state.at(0) = meas.at(0);
            state.at(1) = meas.at(1);
            state.at(2) = 0;
            state.at(3) = 0;
            state.at(4) = meas.at(2);
            state.at(5) = meas.at(3);
            // <<<< Initialization
 
            found = true;
         }
         else
            {kf.correct(meas);} // Kalman Correction
 

} */



/*


void crudeTestWeb()
{
double actualX[]={2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06};
double actualY[]={803811,803811,803811,803811,803811,803811,803811,803811,803811,803811};

double ts[] = {0.5, 0.6, 0.7, 0.4, 0.5, 0.8, 1.4, 1.2, 0.6, 0.87};

cv::KalmanFilter mykf = initializeKalmanFilterWeb();
cv::Mat targetLocPrediction = loopStepWeb(mykf, ts[0], actualX[0], actualY[0], true);
cout <<"initial " <<targetLocPrediction<<"\n";
for (int i = 1; i< 10; i++)
{
 targetLocPrediction = loopStepWeb(mykf, ts[i], actualX[i], actualY[i], false);
cout <<"prediction number "<<i <<" : " <<targetLocPrediction<<"\n";
}

cout <<"now redo with x and y flipped \n";
double notY[]={2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06};
 double notX[]={803811,803811,803811,803811,803811,803811,803811,803811,803811,803811};



 mykf = initializeKalmanFilterWeb();
 targetLocPrediction = loopStepWeb(mykf, ts[0], notX[0], notY[0], true);
cout <<"initial " <<targetLocPrediction<<"\n";
for (int i = 1; i< 10; i++)
{
 targetLocPrediction = loopStepWeb(mykf, ts[i], notX[i], notY[i], false);
cout <<"prediction number "<<i <<" : " <<targetLocPrediction<<"\n";
}

//return void;
}

double expTestWeb()
{
double quads[]={1, 16, 81, 256, 625, 1296, 2401, 4096, 6561, 10000};
double sqrts[]={1, 1.4142, 1.7321, 2, 2.2361 ,2.4495 ,2.6458, 2.8284, 3, 3.1623};

double ts[] = {0.5, 0.6, 0.7, 0.4, 0.5, 0.8, 1.4, 1.2, 0.6, 0.87};

cv::KalmanFilter mykf = initializeKalmanFilterWeb();
cv::Mat targetLocPrediction = loopStepWeb(mykf, ts[0], quads[0], sqrts[0],true);
cout <<"initial " <<targetLocPrediction<<" ";
cout <<"actual x y " <<quads[0] <<" "<<sqrts[0]<<" \n";
for (int i = 1; i< 10; i++)
{
 targetLocPrediction = loopStepWeb(mykf, ts[i], quads[i], sqrts[i], false);
cout <<"prediction number "<<i <<" : " <<targetLocPrediction<<" actual x y " << quads[i] << " " << sqrts[i] << " \n";
}
}


double trigTestWeb()
{
//test trig functions for 0, pi/10, 2*pi/10, ...
double sin[]={0, .309, .5878, .8090, .9511, 1, .9511, .8090, .5878, .309};
double cos[]={1, .9511, .8090, .5787, .309 ,0 ,-.309, -.5787, -.8090, -.9511, -1};

double ts[] = {0.5, 0.6, 0.7, 0.4, 0.5, 0.8, 1.4, 1.2, 0.6, 0.87};

cv::KalmanFilter mykf = initializeKalmanFilterWeb();
cv::Mat targetLocPrediction = loopStepWeb(mykf, ts[0], sin[0], cos[0],true);
cout <<"initial " <<targetLocPrediction<<" ";
cout <<"actual x y " <<sin[0] <<" "<<cos[0]<<" \n";
for (int i = 1; i< 10; i++)
{
 targetLocPrediction = loopStepWeb(mykf, ts[i], sin[i], cos[i], false);
cout <<"prediction number "<<i <<" : " <<targetLocPrediction<<" actual x y " << sin[i] << " " << cos[i] << " \n";
}
}
*/


/*
void WriteToCSV(vector<Point> loc, vector<Point> vel, Vector<Point> predLoc)
{
    ofstream myFile;
    printf("about to open file for Kalman");

    myFile.open("/home/ubuntu/kalmanLog.csv");
    if (myFile.is_open()) {printf("success\n");} else {printf("failure\n");}

    myFile << "locX, locY, velX, velY, predLocX, predLocY, predVelX, predVelY";

    for (int i =0; i= predLoc.size(); i++)
    {
        myFile  << loc[i].x << "," 
                << loc[i].y << "," 
                << vel[i].x << "," 
                << vel[i].y << "," 
                << predLoc[i].x << "," 
                << predLoc[i].y << "," 
                << std:endl;
    } 

    myFile.close(); 

}

*/

