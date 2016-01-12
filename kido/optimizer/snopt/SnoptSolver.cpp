/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/
#include "SnoptSolver.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

#include "optimizer/Problem.h"
#include "optimizer/Var.h"
#include "optimizer/Constraint.h"
#include "optimizer/ConstraintBox.h"
#include "optimizer/ObjectiveBox.h"

namespace dart {
namespace optimizer {
namespace snopt {


SnoptSolver::SnoptSolver(Problem *problem)
    : Solver(problem) {
    mSnopt = NULL;

    mNoDisplay = false;
    mSolverIter = 10;

    // mTotalDofs = 0;
    mTotalDofs = problem->getNumVariables();

    mOptCount = 0;
    mPrint = true;
    mUnit = 4;
}

SnoptSolver::~SnoptSolver() {
    delete mSnopt;
}

bool SnoptSolver::solve() {
    resetSolver();

    int nVar = mTotalDofs;

    // RESET CONSTRAINTS AND OBJECTIVES
    //	mConstrBox->SetNumDofs(nVar);
    //	mObjBox->SetNumDofs(nVar);

    // RESET PROBLEM DIMENSION IN SNOPT
    int nConstr = conBox()->getNumTotalRows();

    mSnopt->resizeJacobian(nVar, nVar, nConstr, nConstr);

    // FILL IN BOUNDARIES
    double* coef_vals = new double[nVar + mSnopt->mNumConstr];
    double* lower_bounds = new double[nVar];
    double* upper_bounds = new double[nVar];


    std::vector<Variable *>& vars(mProb->vars());
    for(int i = 0; i < nVar; i++){
        upper_bounds[i] = vars[i]->mUpper;
        lower_bounds[i] = vars[i]->mLower;
        coef_vals[i] = vars[i]->mVal;
    }

    int count = 0;
    // ASSIGN SLACK FOR EACH CONSTRAINT
    for(int i = 0; i < conBox()->getNumConstraints(); i++){
        Constraint* c = conBox()->getConstraint(i);
        for(int j = 0; j < c->mNumRows; j++)
            coef_vals[nVar + count + j] = c->mConstTerm[j];
        count += c->mNumRows;
    }

    // ASSIGN CONSTRINAT TYPE
    int counter = 0;
    // switch(constr_eqn)
    // case 0: [-constTerm, constTerm]
    // case 1: [constTerm, constTerm + 1e7] C>=0
    // case 2: [constTerm - 1e7, constTerm] C<=0
    // case 3:
    // case 4: [constTerm - 1e-2, constTerm + 1e-2]
    // case 5: [constTerm - 1e3, constTerm + 1e3]
    for(int i = 0; i < conBox()->getNumConstraints(); i++){
        Constraint* c = conBox()->getConstraint(i);
        for(int j=0; j< c->mNumRows; j++){
            if(c->mEquality == 1)
                mSnopt->mConstrEqns[counter++] = 1;
            else if(c->mEquality == -1)
                mSnopt->mConstrEqns[counter++] = 2;
            else if(c->mSlack) // allow slip
                mSnopt->mConstrEqns[counter++] = 5;
            else
                mSnopt->mConstrEqns[counter++] = 0;
        }
    }

    mOptCount++;
    // comment the following two lines for better performance
    mSnopt->mOutput = 9;
    // mSnopt->mSum = 6;

    SnoptInterface::Return ret = mSnopt->solve(coef_vals, lower_bounds, upper_bounds,mUnit);

    std::cout << "[VLOG(1)]" << "SnoptSolver " << mOptCount << " : ";
    std::cout << "[VLOG(1)]" << "obj = " << mSnopt->mReturnedObj << std::endl;

    delete[] coef_vals;
    delete[] lower_bounds;
    delete[] upper_bounds;

    if(ret == SnoptInterface::Solution) {
        return true;
    } else {
        return false;
    }
}

int SnoptSolver::iterUpdate(long mask, int compute_gradients, double *coefs, void *update_data) {
    if(!mask)
        return 0;


    SnoptSolver *m = (SnoptSolver*)update_data;


    // CLEAR SNOPT
    m->mSnopt->clear(mask, compute_gradients);

    // UPDATE MODELS

    std::vector<Variable *>& vars(m->mProb->vars());
    for(int i = 0; i < m->mTotalDofs; i++)
        vars[i]->mVal = coefs[i];

    // Update Problem
    m->mProb->update(coefs);

    // UPDATE CONSTRAINTS, OBJECTIVES, AND THEIR GRADIENTS
    if(mask & SnoptInterface::Obj){
        //	m->mObjBox->EvalActivation(0);
        if(compute_gradients){
            //FILL OUT dObj_dCoef
            m->objBox()->evalObjGrad();
        }

        if(compute_gradients != 1){
            //FILL OUT obj
            m->objBox()->evalObj();
        }
    }

    if(mask & SnoptInterface::Constr){
        //	m->mConstrBox->EvalActivation(0);
        if(compute_gradients){
            // FILL OUT JACOBIAN
            m->conBox()->evalJac();
        }
        if(compute_gradients != 1){
            //FILL OUT CONSTR
            m->conBox()->evalCon();
        }
    }

    return 1;
}

void SnoptSolver::resetSolver() {
    delete mSnopt;

    mSnopt = new SnoptInterface(1, &conBox()->mJac, &conBox()->mJacMap, &conBox()->mCon, &objBox()->mObj, &objBox()->mObjGrad, SnoptSolver::iterUpdate, this);

}


ConstraintSet* SnoptSolver::conBox() {
    return mProb->conBox();
}
ObjectiveSet* SnoptSolver::objBox() {
    return mProb->objBox();
}

} // namespace snopt
} // namespace optimizer
} // namespace dart
