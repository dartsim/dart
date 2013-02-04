/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 * Date: 02/01/2013
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

    void ConstraintDynamics::applyConstraintForces(VectorXd& _qddot) {
        if (mBodies.size() == 0)
            return;

        double ks = 100;
        double kd = 10;
        int nConstrs = mBodies.size();
        int nDofs = mSkel->getNumDofs();
        VectorXd qDot = mSkel->getQDotVector();
        MatrixXd J(MatrixXd::Zero(3 * nConstrs, nDofs));
        MatrixXd JDot(MatrixXd::Zero(3 * nConstrs, nDofs));
        VectorXd C(3 * nConstrs);
        VectorXd CDot(3 * nConstrs);

        for (int i = 0; i < nConstrs; i++) {
            J.block(i * 3, 0, 3, nDofs) = getJacobian(mBodies[i], mOffsets[i]);
            JDot.block(i * 3, 0, 3, nDofs) = getJacobianDot(mBodies[i], mOffsets[i]);
            Vector3d worldP = xformHom(mBodies[i]->getWorldTransform(), mOffsets[i]);
            C.segment(i * 3, 3) = worldP - mTargets[i];
            CDot.segment(i * 3, 3) = J.block(i * 3, 0, 3, nDofs) * qDot;

        }
            
        mA = J * mSkel->getInvMassMatrix() * J.transpose();
        mB = -JDot * qDot - J * _qddot - C * ks - CDot * kd;
        VectorXd lambda = mA.inverse() * mB;
        mConstrForce = J.transpose() * lambda;
    }

    void ConstraintDynamics::addConstraint(BodyNodeDynamics *_body, Vector3d _offset) {
        mBodies.push_back(_body);
        mOffsets.push_back(_offset);
        Vector3d worldP = xformHom(_body->getWorldTransform(), _offset);
        mTargets.push_back(worldP);
    }
    void ConstraintDynamics::reset() {
        mBodies.clear();
        mOffsets.clear();
    }

    MatrixXd ConstraintDynamics::getJacobian(BodyNodeDynamics* _node, const Vector3d& _localP) {
        int nDof = mSkel->getNumDofs();
        MatrixXd J( MatrixXd::Zero(3, nDof));
 
        for(int i = 0; i < _node->getNumDependentDofs(); i++) {
            int dofIndex = _node->getDependentDof(i);
            VectorXd Jcol = xformHom(_node->getDerivWorldTransform(i), _localP);
            J.col(dofIndex) = Jcol;
        }
        return J;
    }

    MatrixXd ConstraintDynamics::getJacobianDot(BodyNodeDynamics* _node, const Vector3d& _localP) {
        int nDof = mSkel->getNumDofs();
        int nLocalDof = _node->getNumDependentDofs();
        VectorXd qDot = mSkel->getQDotVector();
        MatrixXd JDot(MatrixXd::Zero(3, nDof));
        MatrixXd sum(MatrixXd::Zero(3, nLocalDof));
        _node->updateSecondDerivatives(_localP);
        for (int i = 0; i < nLocalDof; i++) {
            int dofIndex = _node->getDependentDof(i);
            sum += _node->getJvDeriv(i) * qDot[dofIndex];
        }
        for (int i = 0; i < nLocalDof; i++) {
            int dofIndex = _node->getDependentDof(i);
            JDot.col(dofIndex) = sum.col(i);
        }
        return JDot;
    }


} // namespace dynamics

