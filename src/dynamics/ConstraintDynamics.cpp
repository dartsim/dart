/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 * Georgia Tech Graphics Lab
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
 *   * This code incorporates portions of Open Dynamics Engine 
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights 
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow 
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "ConstraintDynamics.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"

using namespace Eigen;
using namespace utils;

namespace dynamics {
    ConstraintDynamics::ConstraintDynamics(SkeletonDynamics *_skel) {
        mSkel = _skel;
    }

    ConstraintDynamics::~ConstraintDynamics() {
    }

    void ConstraintDynamics::addConstraint(Constraint *_constr) {
        mConstraints.push_back(_constr);
        int nConstr = mConstraints.size();
        mJGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mJDotGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mCGlobal = VectorXd(3 * nConstr);
        mCDotGlobal = VectorXd(3 * nConstr);
    }

    void ConstraintDynamics::deleteConstraint(int _index) {
        delete mConstraints[_index];
        mConstraints.erase(mConstraints.begin() + _index);
        int nConstr = mConstraints.size();
        mJGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mJDotGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mCGlobal = VectorXd(3 * nConstr);
        mCDotGlobal = VectorXd(3 * nConstr);
    }            

    void ConstraintDynamics::applyConstraintForces(VectorXd& _qddot) {
        double ks = 100;
        double kd = 10;
        int nConstrs = mConstraints.size();
        int nDofs = mSkel->getNumDofs();
        VectorXd qDot = mSkel->getQDotVector();
        /*            MatrixXd J(MatrixXd::Zero(3 * nConstrs, nDofs));
                      MatrixXd JDot(MatrixXd::Zero(3 * nConstrs, nDofs));
                      VectorXd C(3 * nConstrs);
                      VectorXd CDot(3 * nConstrs);
        */
        for (int i = 0; i < nConstrs; i++) {
            mConstraints[i]->updateDynamics();
            mJGlobal.block(i * 3, 0, 3, nDofs) = mConstraints[i]->getJ();
            mJDotGlobal.block(i * 3, 0, 3, nDofs) = mConstraints[i]->getJDot();
            mCGlobal.segment(i * 3, 3) = mConstraints[i]->getC();
            mCDotGlobal.segment(i * 3, 3) = mConstraints[i]->getCDot();
        }
            
        MatrixXd A = mJGlobal * mSkel->getInvMassMatrix() * mJGlobal.transpose();
        VectorXd b = -mJDotGlobal * qDot - mJGlobal * _qddot - mCGlobal * ks - mCDotGlobal * kd;
        VectorXd lambda = A.inverse() * b;
        mConstrForce = mJGlobal.transpose() * lambda;
    }
} // namespace dynamics

