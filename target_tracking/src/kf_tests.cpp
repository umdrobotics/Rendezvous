#include "target_tracking/KalmanFilter.h"
#include <ros/ros.h>
#include <sstream>  // stringstream, ...
#include <iostream> // std::cout, std::end, ...
#include <iomanip>  // std::setprecision
#include <random>


double _target_pos_x = 0.0;
double _target_pos_y = 0.0;
double _noise_variance = 4.0;

cv::Mat GenerateLinearPath(double x_vel, double y_vel, double dt)
{

	cv::Mat path(2, 1, CV_64F);
	_target_pos_x =  _target_pos_x + x_vel*dt;
	_target_pos_y =  _target_pos_y + y_vel*dt;

	path.at<double>(0) = _target_pos_x;
	path.at<double>(1) = _target_pos_y;

	return path;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kf_tests");
    ROS_INFO("Kalman Filter Testing....");
    // ros::NodeHandle nh;

    int nStateSize = 4;
    int nMeasurementSize = 2;
    int nInputSize = 0;

    KalmanFilter kf(nStateSize, nMeasurementSize, nInputSize, CV_64F);
    
    ROS_INFO_STREAM("Kalman Filter:" << kf);


    double dt = 1;
    double x_vel = 5.0;
    double y_vel = 5.0;
    
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution(0.0, 2.0);

    int count = 0;
	cv::Mat path;
	double xIn;
	double yIn;

    while (count++ < 5)
	{		
		path = GenerateLinearPath(x_vel, y_vel, dt);

		ROS_INFO_STREAM("path: " << path);

		xIn = path.at<double>(0) + distribution(generator);
		yIn = path.at<double>(1) + distribution(generator);

		ROS_INFO_STREAM("xIn: " << xIn << ", yIn: " << yIn);

	    cv::Mat predState = kf.ProcessMeasurement(dt, xIn, yIn);

	}
    //std::cout << predState;


    //predState = kf.ProcessMeasurement(dT, xIn, yIn);


    
    return 0;    

}
