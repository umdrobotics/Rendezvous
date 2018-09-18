#include "navigation/MPCController.h"
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>


using namespace std;
using namespace Eigen;

// constructor
MPCController::MPCController()
{

    //~ A_ <<   1, 0, 0.025, 0,
            //~ 0, 1, 0, 0.025,
            //~ 0, 0, 0.9994, 0,
            //~ 0, 0, 0, 0.9994;
	 //~ 
	//~ B_ = Eigen::MatrixXd(4,2);
    //~ B_ <<   -0.0031, 0,
            //~ 0, -0.0031,
            //~ -0.2451, 0,
            //~ 0, -0.2451;
            
    A_ <<   1, 0, 0.0498, 0.0001,
            0, 1, -0.0002, 0.0495,
            0.0009, 0.0008, 0.9895, 0.0038,
            -0.0011, -0.0009, -0.0064, 0.9792;
	 
	B_ = Eigen::MatrixXd(4,2);
    B_ <<   0.0003, -0.0006,
            -0.0001, 0.0004,
            -0.0035, -0.0051,
            0.0132, -0.0049;
    
    C_ = MatrixXd::Identity(4,4);

	P_ = 12; //12
    M_ = 5; //5
    
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
    int k = 7; //7
    //~ for(int i = k; i<P_-k; i++){
        //~ Q_(4*i+2, 4*i+2) = 1.1;
        //~ Q_(4*i+3, 4*i+3) = 1.1;
    //~ } 
    for(int i = 0; i<k; i++){
        Q_(4*i, 4*i) = 10;
        Q_(4*i+1, 4*i+1) = 10;
    }
	for(int i = P_-k; i<P_; i++){
        Q_(4*i, 4*i) = 1.15;    //1.15
        Q_(4*i+1, 4*i+1) = 1.15;   //1.15
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

// Not used now. Since the method to find the solution has changed.
Vector2d MPCController::ComputeOptimalInput(VectorXd StateError)
{
	
    Umd_ = -K_*StateError;
    //~ uk_ += Umd_.head(2)/40;
   
    //~ Um_ += Umd_;
    Um_ = Umd_;
    
    //~ std::cout << "Umd, uk: " << Umd_.transpose() << ", "<< uk_.transpose() << std::endl;
    
    uk_ = Um_.segment(0,nu_);
    Um_.head(nu_*(M_-1)) = Um_.tail(nu_*(M_-1));
    Um_.tail(nu_) = MatrixXd::Zero(nu_, 1);

    //~ LastXp_ = Xp_;
    
    
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




