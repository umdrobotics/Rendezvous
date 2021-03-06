/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/
 //modified from http://www.robot-home.it/blog/en/software/ball-tracker-con-filtro-di-kalman/
// Module "core"
#include <opencv2/core/core.hpp>
 
// Module "highgui"
#include <opencv2/highgui/highgui.hpp>
 
// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>
 
// Module "video"
#include <opencv2/video/video.hpp>
 
// Output
#include <cstdlib>
 
// Vector
#include <vector>
 
using namespace std;
 



// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300

cv::KalmanFilter initializeKalmanFilterWeb()
{


 int stateSize = 4;//6;
   int measSize = 2;//4;
   int contrSize = 0;
 
   unsigned int type = CV_64F;//CV_32F;
   cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
 
   cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
   cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
   //cv::Mat procNoise(stateSize, 1, type)
   // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
 
   // Transition State Matrix A
   // Note: set dT at each processing step!
   // [ 1 0 dT 0  0 0 ]
   // [ 0 1 0  dT 0 0 ]
   // [ 0 0 1  0  0 0 ]
   // [ 0 0 0  1  0 0 ]
   // [ 0 0 0  0  1 0 ]
   // [ 0 0 0  0  0 1 ]
   cv::setIdentity(kf.transitionMatrix);
 
   // Measure Matrix H
   // [ 1 0 0 0 0 0 ]
   // [ 0 1 0 0 0 0 ]
   // [ 0 0 0 0 1 0 ]
   // [ 0 0 0 0 0 1 ]
   kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    cv::setIdentity(kf.measurementMatrix);
   //kf.measurementMatrix.at(0) = 1.0f;
   //kf.measurementMatrix.at(3) = 1.0f;
   //kf.measurementMatrix.at(16) = 1.0f;
   //kf.measurementMatrix.at(23) = 1.0f;
 
   // Process Noise Covariance Matrix Q
   // [ Ex 0  0    0 0    0 ]
   // [ 0  Ey 0    0 0    0 ]
   // [ 0  0  Ev_x 0 0    0 ]
   // [ 0  0  0    1 Ev_y 0 ]
   // [ 0  0  0    0 1    Ew ]
   // [ 0  0  0    0 0    Eh ]
   cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
   //kf.processNoiseCov.at(0) = 1e-2;
   //kf.processNoiseCov.at(7) = 1e-2;
   //kf.processNoiseCov.at(14) = 2.0f;
   //kf.processNoiseCov.at(21) = 1.0f;
   //kf.processNoiseCov.at(28) = 1e-2;
   //kf.processNoiseCov.at(35) = 1e-2;
 
   // Measures Noise Covariance Matrix R
   cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

return kf;

}


/*
double presetp()

{

  char ch = 0;      double ticks = 0;    bool found = false;      int notFoundCount = 0;            // >>>>> Main loop  
   while (ch != 'q' || ch != 'Q')
   {
      double precTick = ticks;
      ticks = (double) cv::getTickCount();
 
      double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

}*/

cv::Mat loopStepWeb(cv::KalmanFilter kf, double dT, double xIn, double yIn, bool firstDetection )
{ ////ayyyyyyyyyyyy


        // [1 0 dt 0]
	// [0 1 0 dt]
	// [0 0 1 0]
	// [0 0 0 1]
	//now update dt
         // >>>> Matrix A
         kf.transitionMatrix.at<double>(2) = dT;
         kf.transitionMatrix.at<double>(7) = dT;
         // <<<< Matrix A


         //cout << "dT:" << endl << dT << endl;
 //ill try moving this to the bottom
       //  state = kf.predict();
        // cout << "State post:" << endl << state << endl;            
 
     // >>>>> Kalman Update

   cv::Mat meas(2, 1, CV_64F); //(measSize, 1, type);
	meas.at<double>(0)=xIn;
	meas.at<double>(1)=yIn;    

      

     if(firstDetection){
            //this should happen on the first detection apparently
            // >>>> Initialization
	   cv::setIdentity(kf.errorCovPre);
	   
            /*kf.errorCovPre.at(0) = 1; // px
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
            state.at(5) = meas.at(3);*/
            // <<<< Initialization
       }

        
       else{ kf.correct(meas); }// Kalman Correction
 
            cv::Mat state(4,1,CV_64F); //(statesize,1,type);
          state = kf.predict();
        if(firstDetection){
            state.at<double>(0) = xIn;
            state.at<double>(1) = yIn;
            state.at<double>(2) = 0;
            state.at<double>(3) = 0;}			
         //cout << "State post:" << endl << state << endl;   
return state ;
} ///end function

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
         // >>>> Matrix A
         kf.transitionMatrix.at<double>(2) = dT;
         kf.transitionMatrix.at<double>(7) = dT;
         // <<<< Matrix A


        // cout << "dT:" << endl << dT << endl;
 //ill try moving this to the bottom
       //  state = kf.predict();
        // cout << "State post:" << endl << state << endl;            
 
     // >>>>> Kalman Update

   

      
 //we will probably need to modify the covariance if we lose the detection

           
            // >>>> Initialization
	   //cv::setIdentity(kf.errorCovPre);
	   
            /*kf.errorCovPre.at(0) = 1; // px
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
            state.at(5) = meas.at(3);*/
            // <<<< Initialization
       //

        

 
            cv::Mat state(4,1,CV_64F); //(statesize,1,type);
          state = kf.predict();
		
         //cout << "State post no measurement:" << endl << state << endl;   
return state ;
} ///end function


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

