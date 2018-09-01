#include "navigation/MPCController.h"
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <math.h>
#include <nlopt.hpp>
#include <vector>


using namespace std;
using namespace Eigen;
//~ using namespace nlopt;

// constructor
MPCController::MPCController()
{

    double m = 2.883;
    double g = 9.80665;
    double kd = 0.0705;

    double q = 0.9;


    A_ <<   1, 0, 0.025, 0,
            0, 1, 0, 0.025,
            0, 0, 0.9994, 0,
            0, 0, 0, 0.9994;
	 
	B_ = Eigen::MatrixXd(4,2);
    B_ <<   -0.0031, 0,
            0, -0.0031,
            -0.2451, 0,
            0, -0.2451;

	P_ = 3; //12

    M_ = 2; //5

    
    //~ 
    nx_ = B_.rows();
    nu_ = B_.cols();

    Hp_ = MatrixXd::Zero(nx_,1);

    
    IsXpInitialized_  = false;
    
	Um_ = MatrixXd::Zero(nu_*M_,1);
	
	Dp_ = MatrixXd::Zero(nx_,1);
    //~ Xp_ = MatrixXd::Zero(nx_*P_,1);
    
    //~ rp_ = MatrixXd::Zero(nx_*P_,1);
    //~ xk_ = MatrixXd::Zero(nx_,1);


}

// destructor   
MPCController::~MPCController()
{

}




// 
void MPCController::Initialize(float q, float kiPos, float kiVec)
{
 
    q_ = q >= 0 ? q : 0.7;
    kiPos_ = kiPos >= 0 ? kiPos : 0;
	kiVec_ = kiVec >= 0 ? kiVec : 0;

    Q_ = q_*MatrixXd::Identity(nx_*P_, nx_*P_);
    R_ = 0.35*(1-q_)*MatrixXd::Identity(nu_*M_, nu_*M_); 

    //~ // Add more penalty on velocity
    int k = 1; //7
    //~ for(int i = k; i<P_-k; i++){
        //~ Q_(4*i+2, 4*i+2) = 1.1;
        //~ Q_(4*i+3, 4*i+3) = 1.1;
    //~ } 
    for(int i = 0; i<k; i++){
        Q_(4*i, 4*i) = 10;
        Q_(4*i+1, 4*i+1) = 10;
    }
	for(int i = P_-k; i<P_; i++){
        Q_(4*i, 4*i) = 1.15;
        Q_(4*i+1, 4*i+1) = 1.15;
    } 
    //~ for(int i = k; i<P_-k; i++){
        //~ Q_(4*i+2, 4*i+2) = 1;
        //~ Q_(4*i+3, 4*i+3) = 1;
    //~ } 
    //~ for(int i = 0; i<k; i++){
        //~ Q_(4*i, 4*i) = 3;
        //~ Q_(4*i+1, 4*i+1) = 3;
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

    // Unconstrainted MPC gain
    K_ = (Bp_.transpose()*Q_*Bp_ + R_).inverse()*Bp_.transpose()*Q_; 
    
    // Constrainted MPC parms 
    H_ = Bp_.transpose()*Q_*Bp_ + R_;
    F_ = Bp_.transpose()*Q_;
    
    Cv_ = MatrixXd::Zero(2*P_, nx_*P_);
    for(int i = 0; i<P_; i++){
        Cv_(2*i, 4*i+2) = 1;
        Cv_(2*i+1, 4*i+3) = 1;
    } 
    
    G_ = MatrixXd::Zero(4*P_+4*M_, nu_*M_);
    G_.block(0,0,2*M_,2*M_) = MatrixXd::Ones(2*M_, 2*M_);
    G_.block(2*M_,0,2*M_,2*M_) = -MatrixXd::Ones(2*M_, 2*M_);
    G_.block(4*M_,0,2*P_,2*M_) = Cv_*Bp_;
    G_.block(2*P_+4*M_,0,2*P_,2*M_) = -Cv_*Bp_;
    
    S_ = MatrixXd::Zero(4*P_+4*M_, 4*P_);
    S_.block(0,0,2*M_,4*P_) = MatrixXd::Zero(2*M_, nx_*P_);
    S_.block(2*M_,0,2*M_,4*P_) = MatrixXd::Zero(2*M_, nx_*P_); 
    S_.block(4*M_,0,2*P_,4*P_) = -Cv_;
    S_.block(2*P_+4*M_,0,2*P_,4*P_) = Cv_; 
    
    Md_ = G_*H_.inverse()*G_.transpose();
    D_ = G_*H_.inverse()*F_ + S_;
    L_ = Md_.norm();
    
    //~ std::cout << "Cv_: " << Cv_ << std::endl;
    //~ std::cout << "G_: " << Bp_ << std::endl;
    //~ std::cout << "S_: " << S_ << std::endl;
    //~ std::cout << "Md_: " << Md_ << std::endl;
    //~ std::cout << "D_: " << D_ << std::endl;
    //~ std::cout << "L_: " << L_ << std::endl;
    

}

void MPCController::SetXpInitialPoint(Vector4d xk){
	if (!IsXpInitialized_){
		Xp_ = xk.colwise().replicate(P_);
		LastXp_ = Xp_;
        LastXk_ = xk;
        
		IsXpInitialized_ = true;
	}
}

VectorXd MPCController::CorrectPrediction(Vector4d output)
{
    // Find model prediction mismatch error, Hp
    // Then 
    
    //~ std::cout << "xk, xp: " << output.transpose() << std::endl << ", " << Xp_.segment(0,16).transpose() << std::endl;
    Hp_ = output - Xp_.segment(0,4);
    //~ Dp_.head(4) += (output - Xp_.segment(52,4)).head(4)*0.006;              /// 76: /20*0.045;
	//~ Dp_.head(2) += Hp_.head(2)*2;       //

	//~ std::cout << "error, Dp: " << (output - Xp_.segment(76,4)).head(2).transpose() << ", " << Dp_.head(2).transpose() << std::endl;
    Xp_ += (Hp_.colwise().replicate(P_) + Dp_.colwise().replicate(P_));
    //~ std::cout << "xk, xp: " << output.transpose() << std::endl << ", " << Xp_.segment(0,4).transpose() << std::endl;

    
    // Update Xp
    Xp_.head(nx_*(P_-1)) = Xp_.tail(nx_*(P_-1));
    //~ std::cout << Hp_ << std::endl;
    //~ std::cout << "xk, xp: " << output.transpose() << ", " << Xp_.segment(4,4).transpose() << std::endl;
    
    
    
    return Xp_;
}


Vector2d MPCController::ComputeOptimalInput(VectorXd StateError)
{
	
    Umd_ = -K_*StateError;
    //~ uk_ += Umd_.head(2)/40;
   
    Um_ += Umd_;
    //~ Um_ = Umd_;
    
    //~ std::cout << "Umd, uk: " << Umd_.transpose() << ", "<< uk_.transpose() << std::endl;
    
    uk_ = Um_.segment(0,nu_);
    Um_.head(nu_*(M_-1)) = Um_.tail(nu_*(M_-1));
    Um_.tail(nu_) = MatrixXd::Zero(nu_, 1);

    //~ LastXp_ = Xp_;
    
    
    return uk_;
}


Vector2d MPCController::ComputeOptimalInput2(Vector4d xk, VectorXd rp)
{
	//~ ArrayXd ub = 20*MatrixXd::Ones(4*P_, 1);
	ArrayXd lb = MatrixXd::Zero(4*P_+4*M_, 1);
	
	VectorXd W = MatrixXd::Zero(4*P_+4*M_, 1);
	W.head(2*M_) = 20*MatrixXd::Ones(2*M_, 1);
	W.segment(2*M_, 2*M_) = 20*MatrixXd::Ones(2*M_, 1);
    W.segment(4*M_, 2*P_) = 17*MatrixXd::Ones(2*P_, 1) - Cv_*rp;
    W.tail(2*P_) = 17*MatrixXd::Ones(2*P_, 1) + Cv_*rp;
    
  
    // Parms of iterator
    MatrixXd xp = Ap_*xk - rp;
    MatrixXd K1 = MatrixXd::Ones(4*P_+4*M_, 4*P_+4*M_) - Md_/L_;
    MatrixXd K2 = (D_*xp + W)/L_;
    
    
    
    // Start iterating
    //~ VectorXd yk = MatrixXd::Zero(4*P_+4*M_, 1);
    //~ VectorXd iterator = (K1*yk - K2).min(ub).max(lb);
    //~ while((iterator - yk).norm() > 0.1){
		//~ yk = iterator;
		//~ iterator = (K1*yk - K2).min(ub).max(lb);
	//~ }
	//~ yk = iterator;
	VectorXd iterator;
    for(int i = 0; i < 100; i++){
		//~ iterator = (K1*yk - K2).array().max(lb);
		iterator = K1*yk - K2;
		std::cout << yk.head(nu_).transpose() << "," << iterator.head(nu_).transpose() << std::endl;
		
		if ((iterator - yk).norm() < 0.1) { 
			yk = iterator; 
			break; 
		}
		
		
		yk = iterator;
	}
	
	//~ std::cout << yk << std::endl;

	Um_ = -H_.inverse()*(G_.transpose()*yk + F_*xp);
	uk_ = Um_.head(nu_);

   
    return uk_;
}


VectorXd MPCController::Predict(Vector4d xk)
{
    
    //~ VectorXd Xpd = Ap_*(xk-LastXk_) + Bp_*Umd_;    //20s
    //~ Xp_ += Xpd;
    //~ VectorXd Xpd = Ap_*(xk) + Bp_*Umd_;                //16s
    VectorXd Xpd = Ap_*(xk) + Bp_*Um_;
    Xp_ = Xpd;
    
    LastXk_ = xk;
    
    //~ std::cout << "Xpd: " << Xpd.head(8).transpose() << std::endl;
    //~ std::cout << "Xp: " << Xp_.head(8).transpose() << std::endl;
    
    //~ Xp_.tail(nx_*(P_-1)) = Xp_.head(nx_*(P_-1));
    //~ Xp_.head(nx_) = xk;

    return Xp_;
}




