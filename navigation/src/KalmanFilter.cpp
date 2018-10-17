#include "navigation/KalmanFilter.h"
#include <math.h>
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

// constructor
KalmanFilter::KalmanFilter()
{
    sigma_ax_ = 0.1; // cause max acceleration = 1.5 m/s2
    sigma_ay_ = 0.1;
    sigma_GPSpx_ = 0.3;
    sigma_GPSpy_ = 0.3;
    sigma_GPSvx_ = 0.18; 
    sigma_GPSvy_ = 0.18;  
    sigma_Apriltagpx_ = 0.1; 
    sigma_Apriltagpy_ = 0.1;  

}

// destructor   
KalmanFilter::~KalmanFilter()
{

}

// 
void KalmanFilter::Initialize()
{
    double dt = 0.1;  // for GPS
    // double dpt = 0.025; // for prediction

    //~ // Initialize system matrixes
    //~ A_ <<  1, 0, dt, 0,
            //~ 0, 1, 0, dt,
            //~ 0, 0, 1, 0,
            //~ 0, 0, 0, 1;
            
    // Ap_ <<  1, 0, dpt, 0,
    //         0, 1, 0, dpt,
    //         0, 0, 1, 0,
    //         0, 0, 0, 1;    // for prediction
     
    // B_ = Eigen::MatrixXd(4,2);
    B_ = 0 ;

    Cg_ = MatrixXd::Identity(4, 4);
    Ca_ = Eigen::MatrixXd(2, 4);
    Ca_ <<  1, 0, 0, 0,
            0, 1, 0, 0;

    // Initialize uncertainty and noise
    Q_ <<   pow(dt,4.0)/4*pow(sigma_ax_,2.0), 0, pow(dt,3.0)/2*pow(sigma_ax_,2.0), 0,
            0, pow(dt,4.0)/4*pow(sigma_ay_,2.0), 0, pow(dt,3.0)/2*pow(sigma_ay_,2.0),
            pow(dt,3.0)/2*pow(sigma_ax_,2.0), 0, pow(dt,2.0)*pow(sigma_ax_,2.0), 0,
            0, pow(dt,3.0)/2*pow(sigma_ay_,2.0), 0, pow(dt,2.0)*pow(sigma_ay_,2.0);

    Rg_ <<  sigma_GPSpx_, 0, 0, 0,
            0, sigma_GPSpy_, 0, 0,
            0, 0, sigma_GPSvx_, 0,
            0, 0, 0, sigma_GPSvy_;    // noise, sigma = 0.5
            
    Ra_ <<  sigma_Apriltagpx_, 0,
            0, sigma_Apriltagpy_;

    nx_ = 4;

}

void KalmanFilter::SetPredHorizon(int nPred){
    
    nPred_ = nPred;
    XP_ = MatrixXd::Zero(nx_*nPred_,1);
    
}    

void KalmanFilter::SetXhatInitialPoint(Vector4d xk){
	if (!IsXhatInitialized_){

        xhat_ = xk;
        P_ = MatrixXd::Identity(4, 4);
        
        xEstWO_ = xhat_;
        
		// IsXhatInitialized_ = true;
	}
}

Matrix4d KalmanFilter::SetAMatrix(double dt){
	
	Matrix4d A;
    A <<    1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
            
    return A;
}


Vector4d KalmanFilter::UpdateWithGPSMeasurements(Vector4d output, double dt)
{
    
    Matrix4d A = SetAMatrix(dt);

    // Predict
    Vector4d xpred = A * xhat_ ;              // local var, prediction for time k
    Matrix4d ppred = A * P_ * A.transpose() + Q_; // local var, prediction for time k

    // Compute Kalman Gain
    Vector4d innovation = output - Cg_ * xpred;
    Matrix4d S = Cg_ * ppred * Cg_.transpose() + Rg_;
    MatrixXd K = ppred * Cg_.transpose() * S.inverse();

    // Update estimate
    xhat_ = xpred + K * innovation;
    P_ = ppred - K * Cg_ * ppred;
    
    
    xEstWO_ = xhat_;

    return xhat_;

}


Vector4d KalmanFilter::UpdateWithCameraMeasurements(Vector2d output, double dt)
{
    
    Matrix4d A = SetAMatrix(dt);

    // Predict
    Vector4d xpred = A * xhat_ ;              // local var, prediction for time k
    Matrix4d ppred = A * P_ * A.transpose() + Q_; // local var, prediction for time k

    // Compute Kalman Gain
    Vector2d innovation = output - Ca_ * xpred;
    Matrix2d S = Ca_ * ppred * Ca_.transpose() + Ra_;
    MatrixXd K = ppred * Ca_.transpose() * S.inverse();

    // Update estimate
    xhat_ = xpred + K * innovation;
    P_ = ppred - K * Ca_ * ppred;
    
    
    xEstWO_ = xhat_;

    return xhat_;

}


Vector4d KalmanFilter::PredictWOObservation(double dt){

	Matrix4d A = SetAMatrix(dt);
	xEstWO_ = A * xEstWO_;
	
	return xEstWO_;
}


VectorXd KalmanFilter::Predict(Vector4d xk)
{
	double dt = 0.025;
    Matrix4d A = SetAMatrix(dt);

	xPred_ = xk;
	XP_.segment(0, nx_) = xPred_;
	
    for(int i = 1; i < nPred_; i++){

        xPred_ = A * xPred_;
        // pPred_ = Ap_ * P_ * Ap_.transpose() + Q_;

        XP_.segment(i*nx_, nx_) = xPred_;

    }

    return XP_;

}




