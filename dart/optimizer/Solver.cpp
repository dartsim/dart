/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Solver.h"
#include "Var.h"
#include "Problem.h"

namespace dart {
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

    Eigen::VectorXd x(nVar);
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
} // namespace dart
