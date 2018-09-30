#ifndef EXTENDED_KALMAN_FILTER_CLASS_H_
#define EXTENDED_KALMAN_FILTER_CLASS_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

class ExtendedKalmanFilter {

    public: // methods

        int nx_;
        int nxdrone_;
        int nPred_;
        
        float dt_;
        float dpt_;
    
        bool IsXhatInitialized_;


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
        VectorXd SystemModel(VectorXd xk, double dt);
        VectorXd ObservationModel(VectorXd xk, double dt);
        MatrixXd JacobianSystemModel(VectorXd xk, double dt);
        MatrixXd JacobianObservationModel(VectorXd xk, double dt);
        
        
        VectorXd Update(VectorXd xk);
        VectorXd PredictWOObservation();
        VectorXd Predict(VectorXd xk);
        Vector4d StateTransformer(VectorXd xk);





    private: // members

        MatrixXd Q_;
        Matrix4d R_;
        MatrixXd P_;




        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif
