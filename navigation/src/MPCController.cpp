#include "navigation/MPCController.h"
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;


// constructor
// MPCController::MPCController(MatrixXd Ain, MatrixXd Bin, MatrixXd Qin, MatrixXd Rin, int Pin, int Min)
// {
//     A_ = Ain;
//     B_ = Bin;
//     Q_ = Qin;
//     R_ = Rin;
//     M_ = Min;
//     P_ = Pin;

// }

MPCController::MPCController()
{

    double m = 3.1;
    double g = 9.80665;
    double kd = 0.0705;

    double q = 0.9;


    A_ <<   1, 0, 0.02, 0,
            0, 1, 0, 0.02,
            0, 0, 0.9995, 0,
            0, 0, 0, 0.9995;
	 
	B_ = Eigen::MatrixXd(4,2);
    B_ <<   -0.002, 0,
            0, -0.002,
            -0.1961, 0,
            0, -0.1961;

	P_ = 20;

    M_ = 3;

    
    //~ 
    nx_ = B_.rows();
    nu_ = B_.cols();

    Q_ = q*MatrixXd::Identity(nx_*P_, nx_*P_);
    R_ = (1-q)*MatrixXd::Identity(nu_*M_, nu_*M_);

    
    // Add more penalty on velocity
    //~ for(int i = 0; i<P_/2+2; i++){
		//~ Q_(4*i+2, 4*i+2) = 3;
		//~ Q_(4*i+3, 4*i+3) = 3;
	//~ } 
	//~ for(int i = 0; i<P_; i++){
		//~ Q_(4*i, 4*i) = 1;
		//~ Q_(4*i+1, 4*i+1) = 1;
	//~ }



}

// destructor   
MPCController::~MPCController()
{

}




// 
void MPCController::Initialize()
{
    // Build Ap
    Ap_ = MatrixXd::Zero(nx_*P_, nx_);
    MatrixXd Ap0 = MatrixXd::Identity(nx_, nx_);
    for(int i = 0; i < P_; i++)
    {
        Ap0 = Ap0 * A_;
        Ap_.block(i*nx_,0,nx_,nx_) = Ap0;
    }
    //~ std::cout << Ap_ << std::endl;

    // Build Bp
    Bp_ = MatrixXd::Zero(nx_*P_, nu_*M_);
    for(int i = 0; i < P_; i++)
    {
        MatrixXd Bpj = MatrixXd::Zero(nx_, nu_*M_);
        MatrixXd Bpi = MatrixXd::Zero(nx_, nu_);
        if(i < M_-1){
            Bpi = B_;
        }
        else{
            Bpi = A_.pow(i-M_+1)*B_;
        }
        
        
        for(int j = min(M_-1,i); j > -1; j--){
            Bpj.block(0,j*nu_,nx_,nu_) = Bpi;
            Bpi = A_*Bpi;
            
        }
        Bp_.block(i*nx_,0,nx_,nu_*M_) = Bpj;
        
    }
    
    //~ std::cout << Bp_ << std::endl;

    // cout <<
    Um_ = MatrixXd::Zero(nu_*M_,1);
    Xp_ = MatrixXd::Zero(nx_*P_,1);
    
    K_ = (Bp_.transpose()*Q_*Bp_ + R_).inverse()*Bp_.transpose()*Q_;
    //~ std::cout << K_ << std::endl;

}


VectorXd MPCController::Predict(Vector4d xk)
{
	//~ Xp_.block(0,0,nx_*(P_-1),1) = Xp_.block(nx_,0,nx_*(P_-1),1);
    //~ Xp_.block(nx_*(P_-1),0,nx_,1) = MatrixXd::Zero(nx_, 1);
    
    VectorXd Xpd = Ap_*xk + Bp_*Um_;
    //~ MatrixXd Xpd = Ap_*xk;
    //~ std::cout << xk.transpose() << ", "<< Xpd.block(76,0,4,1).transpose()  << std::endl;

    //~ std::cout << xk.transpose() << ", " ;
    //~ Xp_ = Xp_ + Xpd;
    return Xpd;
}


Vector2d MPCController::ComputeOptimalInput(VectorXd StateError)
{
	
    VectorXd Umd = -K_*StateError;
    Um_ += Umd;
    
    //~ std::cout << Um_.transpose() << std::endl;
    
    Vector2d uk = Um_.segment(0,nu_);
    Um_.head(nu_*(M_-1)) = Um_.tail(nu_*(M_-1));
    Um_.tail(nu_) = MatrixXd::Zero(nu_, 1);
    
    
    return uk;
}


MatrixXd MPCController::CorrectPrediction()
{

}


