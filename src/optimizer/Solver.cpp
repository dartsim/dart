/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "Solver.h"
#include "Var.h"
#include "Problem.h"
using namespace Eigen;

namespace optimizer {

    Solver::Solver()
        : mProb(NULL) {
    }

    Solver::Solver(Problem *prob)
        : mProb(prob) {
    }

    Solver::~Solver() {
    }

    Eigen::VectorXd Solver::getState() {
        int nVar = mProb->getNumVariables();
        std::vector<Var *>& vars(mProb->vars());

        VectorXd x(nVar);
        for (int i = 0; i < nVar; i++) {
            x(i) = vars[i]->mVal;
        }
        return x;
    }

    void Solver::setState(const Eigen::VectorXd& x) {
        int nVar = mProb->getNumVariables();
        std::vector<Var *>& vars(mProb->vars());

        for (int i = 0; i < nVar; i++) {
            vars[i]->mVal = x(i);
        }
    }

    void Solver::setProblem(Problem * prob) {
        this->mProb = prob;
    }

    Problem* Solver::getProblem() const {
        return mProb;
    }

} // namespace optimizer
