#include "navigation/MPCController.h"
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;


// constructor
MPCController::MPCController()
{

    double m = 3.1;
    double g = 9.80665;
    double kd = 0.0705;

    double q = 0.9;


    A_ <<   1, 0, 0.025, 0,
            0, 1, 0, 0.025,
            0, 0, 0.9989, 0,
            0, 0, 0, 0.9989;
	 
	B_ = Eigen::MatrixXd(4,2);
    B_ <<   -0.0031, 0,
            0, -0.0031,
            -0.2451, 0,
            0, -0.2451;

	P_ = 15;

    M_ = 5;

    
    //~ 
    nx_ = B_.rows();
    nu_ = B_.cols();

    Hp_ = MatrixXd::Zero(nx_,1);
    LastXp_ = MatrixXd::Zero(nx_*P_,1);


}

// destructor   
MPCController::~MPCController()
{

}




// 
void MPCController::Initialize(float q, float ki)
{

    q_ = q >= 0 ? q : 0.7;
    ki_ = ki >= 0 ? ki : 0;


    Q_ = q_*MatrixXd::Identity(nx_*P_, nx_*P_);
    R_ = (1-q_)*MatrixXd::Identity(nu_*M_, nu_*M_);

    // Add more penalty on velocity
    //~ for(int i = 0; i<P_/2+2; i++){
        //~ Q_(4*i+2, 4*i+2) = 3;
        //~ Q_(4*i+3, 4*i+3) = 3;
    //~ } 
    //~ for(int i = 0; i<P_; i++){
        //~ Q_(4*i, 4*i) = 1;
        //~ Q_(4*i+1, 4*i+1) = 1;
    //~ }


    // Build Ap
    Ap_ = MatrixXd::Zero(nx_*P_, nx_);
    MatrixXd Ap0 = MatrixXd::Identity(nx_, nx_);
    for(int i = 0; i < P_; i++)
    {
        Ap0 = Ap0 * A_;
        Ap_.block(i*nx_,0,nx_,nx_) = Ap0;
    }

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
    

    // cout <<
    Um_ = MatrixXd::Zero(nu_*M_,1);
    Xp_ = MatrixXd::Zero(nx_*P_,1);
    
    K_ = (Bp_.transpose()*Q_*Bp_ + R_).inverse()*Bp_.transpose()*Q_;
    //~ std::cout << K_ << std::endl;

}


VectorXd MPCController::Predict(Vector4d xk)
{
    
    Xp_ = Ap_*xk + Bp_*Um_;

    return Xp_;
}


Vector4d MPCController::CorrectPrediction(Vector4d output)
{
    
    Hp_ = Hp_ + (output - LastXp_.head(4))/40*0.1;
    Hp_.segment(2,2) = MatrixXd::Zero(2, 1);
    
    std::cout << "error, Hp_: " << (output - LastXp_.head(4)).transpose() << ", " << Hp_.transpose() << std::endl;
    
    return Hp_;
}


Vector2d MPCController::ComputeOptimalInput(VectorXd StateError)
{
	
    VectorXd Umd = K_*StateError;
    Um_ += Umd;
    
    //~ std::cout << Um_.transpose() << std::endl;
    
    Vector2d uk = Um_.segment(0,nu_);
    Um_.head(nu_*(M_-1)) = Um_.tail(nu_*(M_-1));
    Um_.tail(nu_) = MatrixXd::Zero(nu_, 1);

    LastXp_ = Xp_;
    
    
    return uk;
}





