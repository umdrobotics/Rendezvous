#ifndef EXTENDED_KALMAN_FILTER_CLASS_H_
#define EXTENDED_KALMAN_FILTER_CLASS_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

class ExtendedKalmanFilter {

    public: // methods

        int nx_;
        //~ int nxdrone_;
        int nPred_;
        
        //~ float dt_;
        //~ float dpt_;
    
        bool IsXhatInitialized_;
        
        double sigma_a_;
        double sigma_w_;


        VectorXd xhat_;

        VectorXd xPred_;
        MatrixXd pPred_;

        VectorXd XP_;
        VectorXd xEstmWO_;


        
        // constructor
        ExtendedKalmanFilter();
        
        // destructor   
        virtual ~ExtendedKalmanFilter();
        
        // Setters and getters
        void Initialize();
        void SetXhatInitialPoint(VectorXd xk);
        void SetPredHorizon(int nPred);

        // Core functions
        Vector4d SystemModel(Vector4d xk, double dt);
        Vector4d ObservationModelForGPS(Vector4d xk, double dt);
        Vector2d ObservationModelForCamera(Vector4d xk, double dt);
        Matrix4d JacobianSystemModel(Vector4d xk, double dt);
        MatrixXd JacobianObservationModelForGPS(Vector4d xk, double dt);
        MatrixXd JacobianObservationModelForCamera(Vector4d xk, double dt);
        
        Vector4d UpdateWithGPSMeasurements(Vector4d xk, double dt);
        Vector4d UpdateWithCameraMeasurements(Vector4d xk, double dt);
        VectorXd PredictWOObservation(double dt);
        VectorXd Predict(Vector4d xk);
        Vector4d StateTransformer(Vector4d xk);





    private: // members

        MatrixXd Q_;
        Matrix4d Rg_;
        Matrix2d Ra_;
        MatrixXd P_;




        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif
