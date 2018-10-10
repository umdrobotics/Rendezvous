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
        Vector4d xEstWO_;


        
        // constructor
        KalmanFilter();
        
        // destructor   
        virtual ~KalmanFilter();
        
        // Setters and getters
        void SetXhatInitialPoint(Vector4d xk);
        void SetPredHorizon(int nPred);
        Matrix4d SetAMatrix(double dt);

        // Core functions
        void Initialize();
        Vector4d UpdateWithGPSMeasurements(Vector4d output, double dt);
        Vector4d UpdateWithCameraMeasurements(Vector2d output, double dt);
        Vector4d PredictWOObservation(double dt);
        VectorXd Predict(Vector4d xk);




    private: // members

        //~ Matrix4d A_;
        // Matrix4d Ap_;
        int B_;
        Matrix4d Cg_;
        MatrixXd Ca_;

        Matrix4d Q_;
        Matrix4d Rg_;
        Matrix2d Ra_;
        Matrix4d P_;




        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif
