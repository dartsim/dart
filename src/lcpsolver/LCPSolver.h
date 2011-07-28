#ifndef _LCP_SOLVER
#define _LCP_SOLVER

#include "Eigen/Dense"
using namespace Eigen;

#include <vector>
using namespace std;

namespace lcpsolver {
    class LCPSolver
    {
    public:
	    LCPSolver();
	    ~LCPSolver();
    	
	    bool Solve(const MatrixXd& _A, const VectorXd& _b, VectorXd& _x);
    private:
	    typedef vector<int> varIndexList;
	    MatrixXd mColumns;
	    varIndexList mBasicVarIndices;
	    varIndexList mNonBasicVarIndices;
	    int mDim;
	    VectorXd mRatios;
	    int mEnteringVariableId;
	    void initialize(const MatrixXd& _A, const VectorXd& _b);
	    bool initStep();
	    bool pivotStep();
	    void exchangeBasis(int _i, int _j);
	    void pivotVariable(int _i);
	    void formColumns(const MatrixXd& _A, const VectorXd& _b);
	    void getSolution(VectorXd& _x, VectorXd& _z);
	    int determineDroppingVariable();
	    void calculateRatio();
	    bool checkIfSolution();

    };
} //namespace lcpsolver
#endif