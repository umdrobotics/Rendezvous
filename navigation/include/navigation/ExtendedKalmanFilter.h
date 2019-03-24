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
        int stepsAhead_;
        
        //~ float dt_;
        //~ float dpt_;
    
        bool IsXhatInitialized_;
        
        double sigma_a_;
        double sigma_w_;
        double sigma_GPSpx_;
        double sigma_GPSpy_;
        double sigma_GPSvx_; 
        double sigma_GPSvy_;  
        double sigma_Apriltagpx_; 
        double sigma_Apriltagpy_;     


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
        
        Vector4d UpdateWithGPSMeasurements(Vector4d output, double dt);
        Vector4d UpdateWithCameraMeasurements(Vector2d output, double dt);
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
