#include "LCPSolver.h"
#include "Lemke.h"

namespace lcpsolver {

    LCPSolver::LCPSolver()
    {

    }
    LCPSolver::~LCPSolver()
    {

    }
    void LCPSolver::initialize(const MatrixXd& _A, const VectorXd& _b)
    {
	    mDim = _b.size();

	    mBasicVarIndices.clear();
	    mNonBasicVarIndices.clear();
	    formColumns(_A, _b);
	    for (int i = 0; i < mDim; ++i)
	    {
		    mBasicVarIndices.push_back(i);
		    mNonBasicVarIndices.push_back(mDim + i);
	    }
	    mNonBasicVarIndices.push_back(2 * mDim);
	    mRatios.resize(mDim);
    }
    bool LCPSolver::Solve(const MatrixXd& A, const VectorXd& b, VectorXd& _x)
    {
    #if 0
	    initialize(A, b);
	    bool bSolutionFound = false;

	    bSolutionFound = initStep();
	    if (!bSolutionFound)
	    {
		    const int MAX_PIVOT_STEP = 1000;
		    for (int ithStep = 0; ithStep < MAX_PIVOT_STEP; ++ithStep)
		    {
			    if (pivotStep())
			    {
				    bSolutionFound = true;
				    break;
			    }
		    }
	    }
	    VectorXd z;
	    if (bSolutionFound)
	    {
		    getSolution(x, z);
	    }
	    return bSolutionFound;
    #else
	    int err = Lemke(A, b, _x);
	    return (err == 0);
    #endif
    }

    bool LCPSolver::initStep()
    {
	    mNonBasicVarIndices.push_back(2 * mDim);
	    mRatios = mColumns.col(2 * mDim + 1);
	    int dropIdInBasicVariables;
	    double minValue = mRatios.minCoeff(&dropIdInBasicVariables);
	    if (minValue >= 0)
		    return true;
	    int realDropId = mBasicVarIndices[dropIdInBasicVariables];
	    exchangeBasis(realDropId, 2 * mDim);
	    pivotVariable(2 * mDim);
	    mEnteringVariableId = (realDropId + mDim) % (2 * mDim);
	    return false;
    }

    bool LCPSolver::pivotStep()
    {
	    int realDropId = determineDroppingVariable();

	    exchangeBasis(realDropId, mEnteringVariableId);
	    pivotVariable(mEnteringVariableId);
	    mEnteringVariableId = (realDropId + mDim) % (2 * mDim);
	    return checkIfSolution();
    }

    int LCPSolver::determineDroppingVariable()
    {
	    calculateRatio();
	    int dropIdInBasicVariables;
	    double minValue = mRatios.minCoeff(&dropIdInBasicVariables);
	    CHECK(minValue >= 0) << "I think ratio vector should be non negative.";
	    int realDropId = mBasicVarIndices[dropIdInBasicVariables];
	    return realDropId;
    }

    void LCPSolver::calculateRatio()
    {
	    const VectorXd& b = mColumns.col(2 * mDim + 1);
	    int minId = 0;
	    CHECK(b.minCoeff(&minId) >= 0) << "I think b vector should be non negative after pivoting step.";
	    const double inf = 1e30;
	    const VectorXd& enteringColumn = mColumns.col(mEnteringVariableId);
	    for (int i = 0; i < mDim; ++i)
	    {
		    if (enteringColumn[i] < 0)
			    mRatios[i] = inf;
		    else
			    mRatios[i] = b[i] / enteringColumn[i];
	    }
    }

    bool LCPSolver::checkIfSolution()
    {
	    varIndexList::const_iterator it = find(mBasicVarIndices.begin(), mBasicVarIndices.end(), 2 * mDim);
	    if (it == mBasicVarIndices.end())
		    return true;
	    else
	    {
		    const VectorXd& x0 = mColumns.col(2 * mDim);
		    for (int i = 0; i < mDim; ++i)
		    {
			    if (abs(x0[i]) > 1e-10)
				    return false;
		    }
		    return true;
	    }
	    return true;
    }


    void LCPSolver::exchangeBasis(int _inBasic, int _inNonBasic)
    {
	    varIndexList::iterator it1 = find(mBasicVarIndices.begin(), mBasicVarIndices.end(), _inBasic);
	    varIndexList::iterator it2 = find(mNonBasicVarIndices.begin(), mNonBasicVarIndices.end(), _inNonBasic);

	    CHECK(it1 != mBasicVarIndices.end()) << "exchangeBasis cannnot find " << *it1 << " in basic variable indices.";
	    CHECK(it2 != mNonBasicVarIndices.end()) << "exchangeBasis cannnot find " << *it2 << " in nonbasic variable indices.";

	    int it1Value = *it1;
	    *it1 = *it2;
	    *it2 = it1Value;
    }

    void LCPSolver::pivotVariable(int _i)
    {
	    VectorXd ithColumn = mColumns.col(_i);
	    varIndexList::iterator it = find(mBasicVarIndices.begin(), mBasicVarIndices.end(), _i);
	    CHECK(it != mBasicVarIndices.end()) << _i << " is not found in basic variables. Failed to pivotVariable.";
	    int order = it - mBasicVarIndices.begin();
	    if (abs(ithColumn(order)) < 1e-6)
	    {
		    printf("hitted");
	    }
	    CHECK(abs(ithColumn(order)) > 1e-6) << "Leading zero encountered! Is this problem degenerate?";
	    mColumns.row(order) /= ithColumn(order);
	    for (int ithRow = 0; ithRow < mDim; ++ithRow)
	    {
		    if (ithRow == order) continue;
		    mColumns.row(ithRow) -= mColumns(ithRow, _i) * mColumns.row(order);

		    CHECK_NEAR(mColumns(ithRow, _i), 0, 1e-10);
		    mColumns(ithRow, _i) = 0;
	    }

    }

    void LCPSolver::formColumns(const MatrixXd& _A, const VectorXd& _b)
    {
	    mColumns.resize(mDim, 2 * mDim + 2);
	    mColumns.block(0, 0, mDim, mDim) = MatrixXd::Identity(mDim, mDim);

	    mColumns.block(0, mDim, mDim, mDim) = -_A;
	    for (int i = 0; i < _b.size(); ++i)
	    {
		    for (int j = 0; j < _b.size(); ++j)
		    {
			    if (abs(mColumns(i, j + mDim)) < 1e-10)
				    mColumns(i, j + mDim) = 0;
		    }
	    }
	    mColumns.block(0, 2 * mDim, mDim, 1) = VectorXd::Constant(mDim, -1);
	    mColumns.block(0, 2 * mDim + 1, mDim, 1) = _b;
    }

    void LCPSolver::getSolution(VectorXd& _x, VectorXd& _z)
    {
	    VectorXd result = VectorXd::Zero(2 * mDim);
	    for (varIndexList::const_iterator it1 = mBasicVarIndices.begin(); it1 != mBasicVarIndices.end(); ++it1)
	    {
		    int idx = *it1;
		    varIndexList::iterator it = find(mBasicVarIndices.begin(), mBasicVarIndices.end(), idx);
		    CHECK(it != mBasicVarIndices.end()) << idx << " is not found in basic variables. Failed to pivotVariable.";
		    int order = it - mBasicVarIndices.begin();

		    result[idx] = mColumns(order, 2 * mDim + 1);
	    }

	    _z = result.segment(0, mDim);
	    _x = result.segment(mDim, mDim);
    }
} //namespace lcpsolver