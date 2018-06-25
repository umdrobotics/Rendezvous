#ifndef MPC_CONTROLLER_CLASS_H_
#define MPC_CONTROLLER_CLASS_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

class MPCController {

    public: // methods
    
        int P_, M_;
        int nx_, nu_;

        // constructor
        MPCController();
        
        // destructor   
        virtual ~MPCController();
        
        // Setters and getters
        
        // Core functions
        void Initialize();
        VectorXd Predict(Vector4d xk);
        Vector2d ComputeOptimalInput(MatrixXd StateError);
        MatrixXd CorrectPrediction();



    private: // members

        Matrix4d A_;
        MatrixXd B_;

        MatrixXd Q_;
        MatrixXd R_;

        MatrixXd Ap_; 
        MatrixXd Bp_;
            
        MatrixXd K_;
        VectorXd Um_;
        
        VectorXd Xp_;

        
    private: // methods
        
        
        
    private: // NOT IMPLEMENTED


};


#endif
