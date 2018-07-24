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
    float sigma_ax = 0.5; // cause max acceleration = 1.5 m/s2
    float sigma_ay = 0.5;

    float dt = 0.1;
    float dpt = 0.025; // for prediction

    // Initialize system matrixes
    A_ <<   1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
            
    Ap_ <<  1, 0, dpt, 0,
            0, 1, 0, dpt,
            0, 0, 1, 0,
            0, 0, 0, 1;    // for prediction
     
    // B_ = Eigen::MatrixXd(4,2);
    B_ = 0 ;

    C_ = Eigen::MatrixXd(2,4);  
    C_ <<   1, 0, 0, 0,
            0, 1, 0, 0; 

    // Initialize uncertainty and noise
    Q_ <<   pow(dt,4.0)/4*pow(sigma_ax,2.0), 0, pow(dt,3.0)/2*pow(sigma_ax,2.0), 0,
            0, pow(dt,4.0)/4*pow(sigma_ay,2.0), 0, pow(dt,3.0)/2*pow(sigma_ay,2.0),
            pow(dt,3.0)/2*pow(sigma_ax,2.0), 0, pow(dt,2.0)*pow(sigma_ax,2.0), 0,
            0, pow(dt,3.0)/2*pow(sigma_ay,2.0), 0, pow(dt,2.0)*pow(sigma_ay,2.0);

    R_ <<   0.09, 0,
            0, 0.09;    // noise, sigma = 0.3


    nx_ = A_.cols();

}

// destructor   
KalmanFilter::~KalmanFilter()
{

}

// 
void KalmanFilter::Initialize()
{

}

void KalmanFilter::SetPredHorizon(int nPred){
    
    nPred_ = nPred;
    XP_ = MatrixXd::Zero(nx_*nPred_,1);
    
}    

void KalmanFilter::SetXhatInitialPoint(Vector4d xk){
	if (!IsXhatInitialized_){

        xhat_ = xk;
        P_ = MatrixXd::Identity(4, 4);
        
		IsXhatInitialized_ = true;
	}
}


Vector4d KalmanFilter::Update(Vector4d xk)
{
    
    Vector2d output = xk.head(2);

    // Predict
    Vector4d xpred = A_ * xhat_ ;              // local var, prediction for time k
    Matrix4d ppred = A_ * P_ * A_.transpose() + Q_; // local var, prediction for time k

    // Compute Kalman Gain
    Vector2d innovation = output - C_ * xpred;
    Matrix2d S = C_ * ppred * C_.transpose() + R_;
    MatrixXd K = ppred * C_.transpose() * S.inverse();

    // Update estimate
    xhat_ = xpred + K * innovation;
    P_ = ppred - K * C_ * ppred;

    return xhat_;

}



VectorXd KalmanFilter::Predict()
{
    for(int i = 0; i < nPred_; i++){

        xPred_ = Ap_ * xhat_;
        // pPred_ = Ap_ * P_ * Ap_.transpose() + Q_;

        XP_.segment(i*nx_, nx_) = xPred_;

    }

    return XP_;

}




