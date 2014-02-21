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

#ifndef DART_OPTIMIZER_CONSTRAINT_H
#define DART_OPTIMIZER_CONSTRAINT_H

#include <vector>
#include <Eigen/Dense>
#include "OptimizerArrayTypes.h"

namespace dart {
namespace optimizer {

class Var;

class Constraint {
public:
    Constraint(std::vector<Var *>& var);
    virtual ~Constraint() {}

public:
    virtual Eigen::VectorXd evalCon() = 0;
    virtual double evalObj();

    virtual void fillJac(VVD, int index) {}
    virtual void fillJac(VVD, VVB, int index) {}
    virtual void fillObjGrad(std::vector<double>& dG){}
    virtual void fillObjHess(Eigen::MatrixXd& ddG, int index) {}

    virtual void allocateMem() {}
    virtual void updateParams() {}

    std::vector<Var *>& mVariables;
    int mIndex; // index of the variable this constraint is concerning
    int mNumRows; // number of rows of this constraint
    bool mSlack; // [-1e3, 1e3]
    Eigen::VectorXd mConstTerm; // constraint value, usually zero for equality constraint
    bool mActive; // is this constraint active
    Eigen::VectorXd mWeight;
    std::vector<int> mConfigIndecies;
    int mEquality; // 1: >=0; -1: <=0; 0: =0
    Eigen::VectorXd mCompletion;
};

} // namespace optimizer
} // namespace dart

#endif // #ifndef DART_OPTIMIZER_CONSTRAINT_H

