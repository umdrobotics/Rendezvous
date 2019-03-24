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

    //~ dt_ = 0.1;
    //~ dpt_ = 0.025; // for prediction
    
    sigma_a_ = 0.1;
    sigma_w_ = 0.001;
    


    // Initialize uncertainty and noise
    Q_ = MatrixXd::Zero(4,4);
    //~ Q_.block(3,3,2,2) = MatrixXd::Identity(2,2);


    Rg_ <<  0.3*0.3, 0, 0, 0,
            0, 0.3*0.3, 0, 0,
            0, 0, 0.18*0.18*2, 0,
            0, 0, 0, 0.001*0.001;   // noise, sigma = 0.5
            
    Ra_ <<  0.1*0.1, 0,
            0, 0.1*0.1;


    nx_ = 4;  
    //~ nxdrone_ = 4;// for drone state

}

// destructor   
ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
	
}

// Initializer
void ExtendedKalmanFilter::Initialize(){
	
}

// Setter and getter
void ExtendedKalmanFilter::SetPredHorizon(int nPred){
    
    nPred_ = nPred;
    XP_ = MatrixXd::Zero(nx_*nPred_,1);
    
}    

void ExtendedKalmanFilter::SetXhatInitialPoint(VectorXd xk){
	if (!IsXhatInitialized_){

        xhat_ = xk;
        xEstmWO_ = xhat_;
        P_ = MatrixXd::Identity(nx_, nx_);
        
		IsXhatInitialized_ = true;
	}
}


Vector4d ExtendedKalmanFilter::UpdateWithGPSMeasurements(Vector4d xk, double dt){

    Vector4d output = ObservationModelForGPS(xk, dt);
    double theta = xk(3);
    
    Q_ <<   pow(dt,4.0)/4*pow(sigma_a_,2.0)*cos(theta)*cos(theta), pow(dt,4.0)/4*pow(sigma_a_,2.0)*cos(theta)*sin(theta), pow(dt,3.0)/2*pow(sigma_a_,2.0)*cos(theta), 0,
            pow(dt,4.0)/4*pow(sigma_a_,2.0)*cos(theta)*sin(theta), pow(dt,4.0)/4*pow(sigma_a_,2.0)*sin(theta)*sin(theta), pow(dt,3.0)/2*pow(sigma_a_,2.0)*sin(theta), 0,
            pow(dt,3.0)/2*pow(sigma_a_,2.0)*cos(theta), 		   pow(dt,3.0)/2*pow(sigma_a_,2.0)*sin(theta),            pow(dt,2.0)*pow(sigma_a_,2.0),              0,
            0, 													   0,                                                     0,                                          pow(dt,2.0)*pow(sigma_w_,2.0);

    // Predict
    VectorXd xpred = SystemModel(xhat_, dt) ;              // local var, prediction for time 
    MatrixXd F = JacobianSystemModel(xhat_, dt);           // state transition Jacobian
    MatrixXd ppred = F * P_ * F.transpose() + Q_;             // local var, prediction for time k
    
    //~ std::cout << Q_.transpose() << std::endl;

    // Compute Kalman Gain
    VectorXd innovation = output - ObservationModelForGPS(xpred, dt);
    MatrixXd H = JacobianObservationModelForGPS(xpred, dt);
    MatrixXd S = H * ppred * H.transpose() + Rg_;
    MatrixXd K = ppred * H.transpose() * S.inverse();

    // Update estimate
    xhat_ = xpred + K * innovation;
    P_ = ppred - K * H * ppred;
    
    
    xEstmWO_ = xhat_;

    return xhat_;

}


Vector4d ExtendedKalmanFilter::UpdateWithCameraMeasurements(Vector4d xk, double dt){

    Vector2d output = ObservationModelForCamera(xk, dt);
    double theta = xk(3);
    
    Q_ <<   pow(dt,4.0)/4*pow(sigma_a_,2.0)*cos(theta)*cos(theta), pow(dt,4.0)/4*pow(sigma_a_,2.0)*cos(theta)*sin(theta), pow(dt,3.0)/2*pow(sigma_a_,2.0)*cos(theta), 0,
            pow(dt,4.0)/4*pow(sigma_a_,2.0)*cos(theta)*sin(theta), pow(dt,4.0)/4*pow(sigma_a_,2.0)*sin(theta)*sin(theta), pow(dt,3.0)/2*pow(sigma_a_,2.0)*sin(theta), 0,
            pow(dt,3.0)/2*pow(sigma_a_,2.0)*cos(theta), 		   pow(dt,3.0)/2*pow(sigma_a_,2.0)*sin(theta),            pow(dt,2.0)*pow(sigma_a_,2.0),              0,
            0, 													   0,                                                     0,                                          pow(dt,2.0)*pow(sigma_w_,2.0);

    // Predict
    VectorXd xpred = SystemModel(xhat_, dt) ;              // local var, prediction for time 
    MatrixXd F = JacobianSystemModel(xhat_, dt);           // state transition Jacobian
    MatrixXd ppred = F * P_ * F.transpose() + Q_;             // local var, prediction for time k

    // Compute Kalman Gain
    VectorXd innovation = output - ObservationModelForCamera(xpred, dt);
    MatrixXd H = JacobianObservationModelForCamera(xpred, dt);
    MatrixXd S = H * ppred * H.transpose() + Ra_;
    MatrixXd K = ppred * H.transpose() * S.inverse();

    // Update estimate
    xhat_ = xpred + K * innovation;
    P_ = ppred - K * H * ppred;
    
    
    xEstmWO_ = xhat_;

    return xhat_;

}


Vector4d ExtendedKalmanFilter::SystemModel(Vector4d xk, double dt){

    VectorXd xk1(4);

    double x = xk(0);
    double y = xk(1);
    double v = xk(2);
    double theta = xk(3);
    
    // double omega = xk(4);

    double dx = x + dt*v*cos(theta);
    double dy = y + dt*v*sin(theta);
    double dv = v;
    double dtheta = theta;  // rads
    
    // double domega = omega;				// rads/s

    xk1 << dx, dy,dv, dtheta ;//, domega;
    return xk1;

}


Vector4d ExtendedKalmanFilter::ObservationModelForGPS(Vector4d xk, double dt){

    MatrixXd H(4,4);
    H <<  1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Vector4d output = H*xk;

    return output;
}


Vector2d ExtendedKalmanFilter::ObservationModelForCamera(Vector4d xk, double dt){

    MatrixXd H(4,2);
    H <<  1, 0, 0, 0,
          0, 1, 0, 0;

    Vector2d output = H*xk;

    return output;
}


Matrix4d ExtendedKalmanFilter::JacobianSystemModel(Vector4d xk, double dt){

    // double x = xk(0);
    // double y = xk(1);
    double v = xk(2);
    double theta = xk(3);
    // double omega = xk(4);
    
    MatrixXd jacobian(4,4);
    jacobian <<             1,  0, dt*cos(theta), -dt*v*sin(theta),
                            0,  1, dt*sin(theta), dt*v*cos(theta),
                            0,  0,                1,             0,
                            0,  0,                0,             1;

    return jacobian;
}

MatrixXd ExtendedKalmanFilter::JacobianObservationModelForGPS(Vector4d xk, double dt){

    MatrixXd jacobian(4,4);
    jacobian <<     1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    return jacobian;
    
}


MatrixXd ExtendedKalmanFilter::JacobianObservationModelForCamera(Vector4d xk, double dt){

    MatrixXd jacobian(4,2);
    jacobian <<     1, 0, 0, 0,
                    0, 1, 0, 0;
    return jacobian;
    
} 
                          

VectorXd ExtendedKalmanFilter::PredictWOObservation(double dt){
	
	xEstmWO_ = SystemModel(xEstmWO_, dt);
	
	return xEstmWO_;
}


VectorXd ExtendedKalmanFilter::Predict(Vector4d xk)
{
	double dt = 0.025;
	xPred_ = xk;
	XP_.segment(0, nx_) = StateTransformer(xPred_);
	
    for(int i = 1; i < nPred_; i++){

        xPred_ = SystemModel(xPred_, dt);
        // MatrixXd F = JacobianSystemModel(xPred_, dpt_);           // state transition Jacobian
        // MatrixXd pPred_ = F * P_ * F.transpose() + Q_;             // local var, prediction for time k

        XP_.segment(i*nx_, nx_) = StateTransformer(xPred_);

    }

    return XP_;

}


Vector4d ExtendedKalmanFilter::StateTransformer(Vector4d xk){
	
	Vector4d xk1;

    double x = xk(0);
    double y = xk(1);
    double v = xk(2);
    double theta = xk(3);
    // double omega = xk(4);

    double px = x;
    double py = y;
    double vx = v*cos(theta);
    double vy = v*sin(theta);


    xk1 << px, py, vx, vy;
    
    return xk1;

}
