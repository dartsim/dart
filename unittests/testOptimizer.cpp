/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
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

// For problem
#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "dart/optimizer/Var.h"
#include "dart/optimizer/Constraint.h"
#include "dart/optimizer/Problem.h"
#include "dart/optimizer/ObjectiveBox.h"
#include "dart/optimizer/snopt/SnoptSolver.h"

/* ********************************************************************************************* *
class SampleConstraint : public optimizer::Constraint {
public:
    SampleConstraint(std::vector<optimizer::Var *>& var, int index, double target)
        : optimizer::Constraint(var), mIndex(index), mTarget(target) {
        mNumRows = 1;

        mWeight = Eigen::VectorXd::Ones(1);
        mConstTerm = Eigen::VectorXd::Zero(1);
        mCompletion = Eigen::VectorXd::Zero(1);
    }
    
    virtual Eigen::VectorXd evalCon() {
        std::vector<optimizer::Var *>& vars = mVariables;
        Eigen::VectorXd x(1);
        x(0) = vars[mIndex]->mVal - mTarget;
        return x;
    }

    virtual void fillJac(optimizer::VVD, int index) {}
    virtual void fillJac(optimizer::VVD, optimizer::VVB, int index) {}
    virtual void fillObjGrad(std::vector<double>& dG) {
        VectorXd dP = evalCon();

        for (unsigned int i = 0; i < mVariables.size(); i++){
            const optimizer::Var* var = mVariables[i];
            VectorXd J(1);
            if (i == mIndex) {
                J(0) = 1.0;
            } else {
                J(1) = 0.0;
            }

            J /= var->mWeight;
            dG.at(i) += dP.dot(J);
        }
    }

private:
    int mIndex;
    double mTarget;
};

/* ********************************************************************************************* *
TEST(SIMPLE_SNOPT, OPTIMIZER) {
    using namespace optimizer;
    
    Problem prob;
    prob.addVariable(0.0, -10.0, 10.0);
    prob.createBoxes();

    SampleConstraint* c = new SampleConstraint(
        prob.vars(), 0, 3.0);
    prob.objBox()->add(c);

    snopt::SnoptSolver solver(&prob);
    solver.solve();

    Eigen::VectorXd sol = solver.getState();
    const double TOLERANCE = 0.000001;
    EXPECT_EQ(sol.size(), 1);
    EXPECT_NEAR(sol(0), 3.0, TOLERANCE);
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
