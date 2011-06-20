/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "Solver.h"

namespace optimizer {

    Solver::Solver()
        : mProb(NULL) {
    }

    Solver::Solver(Problem *prob)
        : mProb(prob) {
    }

    Solver::~Solver() {
    }

    void Solver::setProblem(Problem * prob) {
        this->mProb = prob;
    }

    Problem* Solver::getProblem() const {
        return mProb;
    }

} // namespace optimizer
