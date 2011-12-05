/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 * Date: 12/01/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "JointLimitDynamics.h"
#include "lcpsolver/LCPSolver.h"
#include "SkeletonDynamics.h"
#include "kinematics/Dof.h"

using namespace Eigen;

namespace dynamics {
    JointLimitDynamics::JointLimitDynamics(SkeletonDynamics *_skel, double _dt): mSkel(_skel), mDt(_dt) {
        mConstrForce = VectorXd::Zero(mSkel->getNumDofs());
        mTauStar = VectorXd::Zero(mSkel->getNumDofs());
    }

    void JointLimitDynamics::computeTauStar() {
        VectorXd tau = mSkel->getExternalForces();
        mTauStar = (mSkel->getMassMatrix() * mSkel->getQDotVector()) - (mDt * (mSkel->getCombinedVector() - tau));
    }

    void JointLimitDynamics::applyJointLimitTorques() {
        int nDof = mSkel->getNumDofs();
        mLimitingDofIndex.clear();
        for (int i = 0; i < nDof; i++) {
            double val = mSkel->getDof(i)->getValue();
            double ub = mSkel->getDof(i)->getMax();
            double lb = mSkel->getDof(i)->getMin();
            if (val >= ub)
                mLimitingDofIndex.push_back(i);
            if (val <= lb)
                mLimitingDofIndex.push_back(-i);
        }
        if (mLimitingDofIndex.size() == 0)
            return;

        cout << "joint limit hit" << endl;
        fillMatrices();
        bool succ = solve();
        if (!succ)
            cout << "lcp not solved" << endl;
        applySolution();

    }

    void JointLimitDynamics::fillMatrices() {
        int dimA = mLimitingDofIndex.size();      
        MatrixXd Minv = mSkel->getMassMatrix().inverse();
        mA = MatrixXd::Zero(dimA, dimA);
        computeTauStar();

        // Construct A
        for (int i = 0; i < dimA; i++)
            for (int j = 0; j < dimA; j++) 
                mA(i, j) = Minv(abs(mLimitingDofIndex[i]), abs(mLimitingDofIndex[j]));

        mA *= mDt;

        // Construct Q
        MatrixXd JMinv(dimA, Minv.cols());
        for (int i = 0; i < dimA; i++) {
            if (mLimitingDofIndex[i] > 0)  // hitting upper bound
                JMinv.row(i) = -Minv.row(mLimitingDofIndex[i]);
            else
                JMinv.row(i) = Minv.row(abs(mLimitingDofIndex[i]));
        }
        mQBar = JMinv * mTauStar;
    }

    bool JointLimitDynamics::solve() {
        lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
        //cout << mA.rows() << " " << mA.cols() << " " << mQBar.rows() << " " << mX.rows() << endl;
        bool b = solver.Solve(mA, mQBar, mX);
        return b;
    }
    
    void JointLimitDynamics::applySolution() {
        mConstrForce.setZero();
        for (unsigned int i = 0; i < mLimitingDofIndex.size(); i++) {
            if (mLimitingDofIndex[i] > 0) // hitting upper bound
                mConstrForce[mLimitingDofIndex[i]] -= mX[i];
            else
                mConstrForce[abs(mLimitingDofIndex[i])] += mX[i];
            cout << "mConstrForce[" << abs(mLimitingDofIndex[i]) << "] = " << mConstrForce[abs(mLimitingDofIndex[i])] << endl;
        }
    }
    
} // namespace dynamics
