#include "Navigation/MPCController.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


// constructor
// MPCController::MPCController(MatrixXd Ain, MatrixXd Bin, MatrixXd Qin, MatrixXd Rin, int Pin, int Min)
// {
//     A = Ain;
//     B = Bin;
//     Q = Qin;
//     R = Rin;
//     M = Min;
//     P = Pin;

// }

MPCController::MPCController()
{

    double m = 3.1;
    double g = 9.80665;
    double kd = 0.0705;

    A <<    0, 0, 1, 0,
            0, 0, 0, 1,
            0, 0, -kd/m, 0,
            0, 0, 0, -kd/m;

    B <<    0, 0,
            0, 0, 
            -g, 0,
            0, -g;

    M = 3;
    P = 20;

    Q = MatrixXd::Identity(nx*P, nx*P);
    R = MatrixXd::Identity(nu*M, nu*M);


}

// destructor   
MPCController::~MPCController()
{

}

// 
void MPCController::Initialize()
{
    nx = B.rows();
    nu = B.cols();

    // Build Ap
    Ap = MatrixXd::Zero(nx*P, nx);
    MatrixXd Ap0 = MatrixXd::Identity(nx, nx);
    for(int i = 0; i < P; i++)
    {
        Ap0 = Ap0 * A;
        Ap.block<nx,nx>(i*nx,0) = Ap0;
    }

    // Build Bp
    Bp = MatrixXd::Zero(nx*P, nu*M);
    for(int i = 0; i < P; i++)
    {
        MatrixXd Bpj = MatrixXd::Zero(nx, nu*M);
        if(i < M){
            MatrixXd Bpi = B;
        }
        else{
            Bpi = A.pow(i-M)*B;
        }
        
        
        for(int j = min(M,i); j < 1; j--){
            Bpj.block<nx,nu>(0,(j-1)*nu) = Bpi;
            Bpi = A*Bpi;
        }
        Bp.block<nx,nu*M>(i*nx,0) = Bpj;
    }

    // cout <<
    Um = MatrixXd::Zero(nu*M,1);

}


MatrixXd MPCController::Predict(MatrixXd xk)
{
    MatrixXd Xp = Ap*xk + Bp*Um;
    return Xp;
}


MatrixXd MPCController::ComputeOptimalInput(MatrixXd StateError);
{
    MatrixXd Umd = -K*StateError;
    MatrixXd Um += Umd;
    MatrixXd uk = Um.block<1,2>(0,0).transpose();
    return uk;
}


MatrixXd MPCController::CorrectPrediction()
{

}


