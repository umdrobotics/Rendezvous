#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <fstream>

//#include <Windows.h>
 
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )
 
using namespace cv;
using namespace std;
  

void writeToCSV(Vector<Point> loc, Vector<Point> vel, Vector<Point> predLoc)
{
ofstream myFile;
printf("about to open file for Kalman");
myFile.open("/home/ubuntu/kalmanLog.csv");
if (myFile.is_open()) {printf("success\n");} else {printf("failure\n");}
myFile << "locX, locY, velX, velY, predLocX, predLocY, predVelX, predVelY";
for (int i =0; i= predLoc.size(); i++)
{
 myFile << loc[i].x << "," << loc[i].y << "," << vel[i].x << "," << vel[i].y << "," << predLoc[i].x << "," << predLoc[i].y << "," << /*predVel[i].x << "," <<  predVel[i].y <<*/ "\n" ;

 //cout << loc[i].x << "," << loc[i].y << "," << vel[i].x << "," << vel[i].y << "," << predLoc[i].x << "," << predLoc[i].y << "," << predVel[i].x << "," <<  predVel[i].y << "\n" ;
} 
myFile.close(); 

}



//Function should be called when you first find the target
KalmanFilter initializeKalmanFilter(double dt, double targetX, double targetY)
{
//dt = 0.1; //For debugging
KalmanFilter KF(4, 2, 0, CV_64F); //4-element state vector of object, 2-element measurement vector, 0-element control vector 

//initial state estimates based on first time we see the target
//IMPORTANT NOTE: The following sets element 0 to 8, sets element 1 to 7, overrides element 1 as 6, and then sets element 2 to 5. It somehow adds the columns and rows rather than doing a proper i,j access 
//KF.statePre.at<double>(0,0) = 8;
//KF.statePre.at<double>(0,1) = 7;
//KF.statePre.at<double>(1,0) = 6; 
//KF.statePre.at<double>(1,1) = 5; 
// The following approach ACTUALLY WORKS
//and this discussion is relevant to it http://stackoverflow.com/questions/8184053/accessing-elements-of-a-cvmat-with-atfloati-j-is-it-x-y-or-row-col
KF.statePre.at<double>(0) = targetX;
KF.statePre.at<double>(1) = targetY;
KF.statePre.at<double>(2) = 0; //x velocity
KF.statePre.at<double>(3) = 0; //y velocity

//set up measurement matrix
setIdentity(KF.measurementMatrix);

//set up noise measurements
setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
setIdentity(KF.measurementNoiseCov, Scalar::all(10));
setIdentity(KF.errorCovPost, Scalar::all(.1));

//set up the transition matrix
KF.transitionMatrix = *(Mat_<double>(4, 4) << 1,0,dt,0,   0,1,0,dt,  0,0,1,0,  0,0,0,1);

cout <<" initial KF: transitionMatrix: " << KF.transitionMatrix <<"\n";
cout <<" initial KF: measurementMatrix: " << KF.measurementMatrix <<"\n";
cout <<" initial KF: pre-state: " << KF.statePre <<"\n";
 return KF; //keep going calculations with this kalman filter
}



cv::Mat /*void*/ targetTrackStep(KalmanFilter KF, double dt, double xMeasured, double yMeasured)
{
//dt = 0.1; //For debugging


//try this based on this: http://www.robot-home.it/blog/en/software/ball-tracker-con-filtro-di-kalman/
// update dt, call predict,  measure, correct(measured)
//since we have measurements going into this, we'll go: correct, update dt, call predict

Mat_<double> measurement(2,1); 
measurement.setTo(Scalar(0)); //to make sure it's not filled with garbage values

measurement(0) = xMeasured;

measurement(1) = yMeasured;

KF.correct(measurement);

cout <<"measurement : " <<measurement <<"\n";

//now update dt
KF.transitionMatrix.at<double>(0,2) = dt;
KF.transitionMatrix.at<double>(1,3) = dt;

cout <<"updated transition matrix " <<  KF.transitionMatrix <<"\n";

Mat_<double> prediction(2,2);  
 prediction = KF.predict();

return prediction; 


}


cv::Mat /*void*/ targetEstimateWithoutMeasurement(KalmanFilter KF, double dt)
{


//now update dt
KF.transitionMatrix.at<double>(0,2) = dt;
KF.transitionMatrix.at<double>(1,3) = dt;

Mat_<double> prediction(2,2);  
 prediction = KF.predict();
return prediction; 


}
 

void crudeTest()
{
double actualX[]={2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06};
double actualY[]={803811,803811,803811,803811,803811,803811,803811,803811,803811,803811};

double ts[] = {0.5, 0.6, 0.7, 0.4, 0.5, 0.8, 1.4, 1.2, 0.6, 0.87};

KalmanFilter mykf = initializeKalmanFilter(ts[0], actualX[0], actualY[0]);
cv::Mat targetLocPrediction = targetTrackStep(mykf, ts[0], actualX[0], actualY[0]);
cout <<"initial " <<targetLocPrediction<<"\n";
for (int i = 1; i< 10; i++)
{
 targetLocPrediction = targetTrackStep(mykf, ts[i], actualX[i], actualY[i]);
cout <<"prediction number "<<i <<" : " <<targetLocPrediction<<"\n";
}

cout <<"now redo with x and y flipped \n";
double notY[]={2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06,2.49527e+06};
 double notX[]={803811,803811,803811,803811,803811,803811,803811,803811,803811,803811};



 mykf = initializeKalmanFilter(ts[0], notX[0], notY[0]);
 targetLocPrediction = targetTrackStep(mykf, ts[0], notX[0], notY[0]);
cout <<"initial " <<targetLocPrediction<<"\n";
for (int i = 1; i< 10; i++)
{
 targetLocPrediction = targetTrackStep(mykf, ts[i], notX[i], notY[i]);
cout <<"prediction number "<<i <<" : " <<targetLocPrediction<<"\n";
}

//return void;
}






int kalmanTest( )
{ 
printf("kalman test initated");
#define stateSize 4 // x, y, vx, vy
#define measureSize 2 //x, y
#define controlSize 0
 //expect quadcopter to have velocity and position for state (4 elements)
//expect acceleration as a control input (2 elements)
// will measure position AND velocity (4 elements)
KalmanFilter KF(stateSize, measureSize, controlSize); //4-element state vector of object, 2-element measurement vector, 0-element control vector 

cv::Point mousePos; //POINT mousePos;
//GetCursorPos(&mousePos);
mousePos.x =4;
mousePos.y=5; 
cv::Point vel; //model velocity as a 2d point for simplicity
vel.x =0; vel.y = 0; 

double samplingInterval = 0.1;
// intialization of KF...
KF.transitionMatrix = *(Mat_<double>(stateSize, stateSize) << 1,0,samplingInterval,0,   0,1,0,samplingInterval,  0,0,1,0,  0,0,0,1);
Mat_<double> measurement(measureSize,1); measurement.setTo(Scalar(0));
 
KF.statePre.at<double>(0) = mousePos.x;
KF.statePre.at<double>(1) = mousePos.y;
KF.statePre.at<double>(2) = vel.x;
KF.statePre.at<double>(3) = vel.y;
setIdentity(KF.measurementMatrix);
setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
setIdentity(KF.measurementNoiseCov, Scalar::all(10));
setIdentity(KF.errorCovPost, Scalar::all(.1));
// Image to show mouse tracking
Mat img(600, 800, CV_8UC3);
vector<Point> mousev,kalmanv;
mousev.clear();
kalmanv.clear();

//write to CSV file
vector<Point> actualPoints,predictPoints;
vector<Point> actualVels, predictVels; 
actualPoints.clear(); predictPoints.clear(); actualVels.clear(); predictVels.clear();

printf("inital pos x %d y %d", mousePos.x, mousePos.y);
int limit = 100;
int counter = 0; 
while(1 && (counter<limit) )
{ counter++;
 // First predict, to update the internal statePre variable
 Mat prediction = KF.predict();
 Point predictPt(prediction.at<double>(0,0),prediction.at<double>(0,1));

 //Point predictVel(prediction.at<float>(2), prediction.at<float>(3));
              
//update position since it's not windows anymore (simulates moving target)
printf("inital pos again x %d y %d", mousePos.x, mousePos.y);
mousePos.x = (((mousePos.x)+1) %100);
mousePos.y = (((mousePos.y)+1) %100);
//vel.x = (vel.x +(rand()%100) %10);
//vel.y = (vel.y +(rand()%100) %10);

actualPoints.push_back(mousePos);
actualVels.push_back(vel);
predictPoints.push_back(predictPt);
//predictVels.push_back(predictVel);
 // Get mouse point
 //GetCursorPos(&mousePos);
 double oldMeasureX = measurement(0);
double oldMeasureY = measurement(1);
 measurement(0) = mousePos.x;
 measurement(1) = mousePos.y; 
 //measurement(2) = (measurement(0) - oldMeasureX)/samplingInterval;
  //measurement(3) = (measurement(1) - oldMeasureY)/samplingInterval;
 
printf ("iteration %d predictions:  pos x %d y %d measurements: x %f y %f \n", counter,  predictPt.x, predictPt.y ,measurement(0), measurement(1));
printf ("iteration %d actuals: Vel x %d y %d pos x %d y %d \n", counter, vel.x, vel.y, mousePos.x, mousePos.y);  

 // The update phase 
 Mat estimated = KF.correct(measurement);
 cout << "\n estimation matrix" <<estimated <<"\n\n";
//these are drawing phases that don't matter
 /*Point statePt(estimated.at<float>(0),estimated.at<float>(1));
 Point measPt(measurement(0),measurement(1));
    // plot points
    imshow("mouse kalman", img);
    img = Scalar::all(0);
 
    mousev.push_back(measPt);
    kalmanv.push_back(statePt);
    drawCross( statePt, Scalar(255,255,255), 5 );
    drawCross( measPt, Scalar(0,0,255), 5 );
 
    for (int i = 0; i < mousev.size()-1; i++) 
     line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);
     
    for (int i = 0; i < kalmanv.size()-1; i++) 
     line(img, kalmanv[i], kalmanv[i+1], Scalar(0,155,255), 1);
     
 waitKey(10);  */
}
    writeToCSV(actualPoints, predictPoints, actualVels);                                       
    return 0;
}
