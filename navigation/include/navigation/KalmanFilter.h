#ifndef KALMAN_FILTER_CLASS_H_
#define KALMAN_FILTER_CLASS_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

class KalmanFilter {

    public: // methods

        int nx_;
        int nPred_;
    
        bool IsXhatInitialized_;

        Vector4d xhat_;

        Vector4d xPred_;
        Matrix4d pPred_;

        VectorXd XP_;


        
        // constructor
        KalmanFilter();
        
        // destructor   
        virtual ~KalmanFilter();
        
        // Setters and getters
        void SetXhatInitialPoint(Vector4d xk);
        void SetPredHorizon(int nPred){

        // Core functions
        void Initialize();
        Vector4d Update(Vector4d xk);
        VectorXd Predict();




    private: // members

        Matrix4d A_;
        Matrix4d Ap_;
        int B_;
        MatrixXd C_;

        Matrix4d Q_;
        Matrix2d R_;
        Matrix4d P_;




        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif