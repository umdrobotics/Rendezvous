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
        bool IsNoObservation_;

        Vector4d xhat_;

        Vector4d xPred_;
        Matrix4d pPred_;

        VectorXd XP_;
        Vector4d xEstmWO_;


        
        // constructor
        KalmanFilter();
        
        // destructor   
        virtual ~KalmanFilter();
        
        // Setters and getters
        void SetXhatInitialPoint(Vector4d xk);
        void SetPredHorizon(int nPred);

        // Core functions
        void Initialize();
        Vector4d Update(Vector4d xk);
        Vector4d PredictWOObservation();
        VectorXd Predict(Vector4d xk);




    private: // members

        Matrix4d A_;
        Matrix4d Ap_;
        int B_;
        Matrix4d C_;

        Matrix4d Q_;
        Matrix4d R_;
        Matrix4d P_;




        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif
