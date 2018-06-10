#include <Eigen/Dense>

using namespace Eigen;

class MPCController {

    public: // methods

        // dconstructor
        MPCController();
        
        // destructor   
        virtual ~MPCController();
        
        // 
        float Initialize();
        float Predict();
        float ComputeOptimalInput();
        float CorrectPrediction();



    private: // members

        MatrixXd A(4,4);
        MatrixXd B(4,2);

        int P;
        int M;

        MatrixXd Q;
        MatrixXd R;

        MatrixXd Ap; 
        MatrixXd Bp;
            
        MatrixXd K;
        MatrixXd Um(2,1);

        int nx, nu;
        
        
        
        
    private: // methods
        
        
        
    private: // NOT IMPLEMENTED


};


