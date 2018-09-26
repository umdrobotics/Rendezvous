#include "navigation/ExtendedKalmanFilter.h"
#include <math.h>
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

// constructor
ExtendedKalmanFilter::ExtendedKalmanFilter()
{

    dt_ = 0.1;
    dpt_ = 0.05; // for prediction


    // Initialize uncertainty and noise
    Q_ = MatrixXd::Zero(5,5);
    Q_.bottomRightCorner(2,2) = MatrixXd::Identity(2,2);

    R_ <<   0.3, 0
            0, 0.3;   // noise, sigma = 0.5


    nx_ = 5;

}

// destructor   
ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

// Initializer
void ExtendedKalmanFilter::Initialize(){}

// Setter and getter
void ExtendedKalmanFilter::SetPredHorizon(int nPred){
    
    nPred_ = nPred;
    XP_ = MatrixXd::Zero(nx_*nPred_,1);
    
}    

void ExtendedKalmanFilter::SetXhatInitialPoint(VectorXd xk){
	if (!IsXhatInitialized_){

        xhat_ = MatrixXd::Zero(5, 1);
        xEstmWO_ = xhat_;
        P_ = MatrixXd::Identity(5, 5);
        
		IsXhatInitialized_ = true;
	}
}


VectorXd ExtendedKalmanFilter::Update(VectorXd xk){

    VectorXd output = ObservationModel(xk, dt_);

    // Predict
    VectorXd xpred = SystemModel(xhat_, dt_) ;              // local var, prediction for time 
    MatrixXd F = JacobianSystemModel(xhat_, dt_);           // state transition Jacobian
    MatrixXd ppred = F * P_ * F.transpose() + Q_;             // local var, prediction for time k

    // Compute Kalman Gain
    VectorXd innovation = output - ObservationModel(xpred, dt_);
    MatrixXd H = JacobianObservationModel(xpred, dt_);
    MatrixXd S = H * ppred * H.transpose() + R_;
    MatrixXd K = ppred * H.transpose() * S.inverse();

    // Update estimate
    xhat_ = xpred + K * innovation;
    P_ = ppred - K * H * ppred;
    
    
    xEstmWO_ = xhat_;

    return xhat_;

}


VectorXd ExtendedKalmanFilter::SystemModel(VectorXd xk, double dt){

    VectorXd xk1(5);

    double x = xk(0);
    double y = xk(1);
    double theta = xk(2);
    double v = xk(3);
    double omega = xk(4);

    double dx = x + dt*v*cos(theta);
    double dy = y + dt*v*sin(theta);
    double dtheta = theta + dt*omega;
    double dv = v;
    double domega = omega;

    xk1 << dx, dy, dtheta, dv, domega;
    return xk1;

}


VectorXd ExtendedKalmanFilter::ObservationModel(VectorXd xk, double dt){

    MatrixXd H(2,5) <<  1, 0, 0, 0, 0,
                        0, 1, 0, 0, 0;

    VectorXd output = H*xk;

    return output;
}


MatrixXd ExtendedKalmanFilter::JacobianSystemModel(VectorXd xk, double dt){

    // double x = xk(0);
    // double y = xk(1);
    double theta = xk(2);
    double v = xk(3);
    // double omega = xk(4);
    
    MatrixXd jacobian <<    1,  0, -dt*v*sin(theta), dt*cos(theta), 0,
                            0,  1,  dt*v*cos(theta), dt*sin(theta), 0,
                            0,  0,                1,             0, dt,
                            0,  0,                0,             1, 0,
                            0,  0,                0,             0, 1;

    return jacobian;
}

MatrixXd ExtendedKalmanFilter::JacobianObservationModel(VectorXd xk, double dt){

    MatrixXd jacobian(2,5) <<   1, 0, 0, 0, 0,
                                0, 1, 0, 0, 0;
    return jacobian;
    
}                            

VectorXd ExtendedKalmanFilter::PredictWOObservation(){
	
	xEstmWO_ = SystemModel(xEstmWO_, dt_);
	
	return xEstmWO_;
}


VectorXd ExtendedKalmanFilter::Predict(VectorXd xk)
{
	
	xPred_ = xk;
	XP_.segment(0, nx_) = xPred_;
	
    for(int i = 1; i < nPred_; i++){

        xPred_ = SystemModel(xPred_, dpt_);
        // MatrixXd F = JacobianSystemModel(xPred_, dpt_);           // state transition Jacobian
        // MatrixXd pPred_ = F * P_ * F.transpose() + Q_;             // local var, prediction for time k

        XP_.segment(i*nx_, nx_) = xPred_;

    }

    return XP_;

}




