#ifndef MPC_CONTROLLER_CLASS_H_
#define MPC_CONTROLLER_CLASS_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

class MPCController {

    public: // methods
        Matrix4d A_;
        MatrixXd B_;
        Matrix4d C_;

        MatrixXd Q_;
        MatrixXd R_;

        MatrixXd Ap_; 
        MatrixXd Bp_;
        
        MatrixXd K_;
        
    
        double P_, M_;
        int nx_, nu_;
        double q_, kiPos_, kiVec_;
        double Qk_, Qf_, Qb_;
        
        bool IsXpInitialized_;
                

        VectorXd Um_;
        VectorXd Xp_;
        VectorXd LastXp_;
        Vector4d LastXk_; 
		Vector4d Hp_;
		
		//~ Vector4d xk_;
		//~ VectorXd rp_;
		
		Vector4d Dp_;
		
		Vector2d uk_;
		VectorXd Umd_;
        
        // constructor
        MPCController();
        
        // destructor   
        virtual ~MPCController();
        
        // Setters and getters
        void SetXpInitialPoint(Vector4d xk);
        
        // Core functions
        void Initialize(float q, float kiPos, float kiVec);
        VectorXd Predict(Vector4d xk);
        Vector2d ComputeOptimalInput(VectorXd StateError);
        VectorXd CorrectPrediction(Vector4d output);



    private: // members

        //~ Matrix4d A_;
        //~ MatrixXd B_;
//~ 
        //~ MatrixXd Q_;
        //~ MatrixXd R_;
//~ 
        //~ MatrixXd Ap_; 
        //~ MatrixXd Bp_;
            //~ 
        //~ MatrixXd K_;
        //~ MatrixXd K1_;
        //~ MatrixXd K2_;
        //~ MatrixXd K3_;


        
    private: // methods
        
  
    private: // NOT IMPLEMENTED


};


#endif
