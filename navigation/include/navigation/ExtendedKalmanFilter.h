#ifndef KALMAN_FILTER_CLASS_H_
#define KALMAN_FILTER_CLASS_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

class ExtendedKalmanFilter {

    public: // methods

        int nx_;
        int nPred_;
    
        bool IsXhatInitialized_;
        bool IsNoObservation_;

        VectorXd xhat_;

        VectorXd xPred_;
        MatrixXd pPred_;

        VectorXd XP_;
        VectorXd xEstmWO_;

        float dt_;
        float dpt_;
        
        // constructor
        ExtendedKalmanFilter();
        
        // destructor   
        virtual ~ExtendedKalmanFilter();
        
        // Setters and getters
        void Initialize();
        void SetXhatInitialPoint(VectorXd xk);
        void SetPredHorizon(int nPred);

        // Core functions
        VectorXd Update(VectorXd xk);
        VectorXd PredictWOObservation();
        VectorXd Predict(VectorXd xk);




    private: // members

        MatrixXd Q_;
        MatrixXd R_;
        MatrixXd P_;




        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif
