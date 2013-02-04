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

#ifndef DYNAMICS_CONSTRAINT_DYNAMICS_H
#define DYNAMICS_CONSTRAINT_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

/*
  // Sample Usage
  dynamics::ConstraintDynamics constraint();
  constraint.addConstraint(body, offset);
  constraint.applyConstraintForces(qddot); // call this function after qddot is computed
 */
namespace dynamics {
    class BodyNodeDynamics;
    class SkeletonDynamics;
    class ConstraintDynamics {
    public:
        ConstraintDynamics(SkeletonDynamics *_skel);
        virtual ~ConstraintDynamics();
        void applyConstraintForces(Eigen::VectorXd& _qddot);
        void addConstraint(BodyNodeDynamics *_body, Eigen::Vector3d _offset);
        void reset();
        inline Eigen::VectorXd getConstraintForce() const { return mConstrForce; }

    private:
        Eigen::MatrixXd getJacobian(BodyNodeDynamics* _node, const Eigen::Vector3d& _localP);
        Eigen::MatrixXd getJacobianDot(BodyNodeDynamics* _node, const Eigen::Vector3d& _localP);

        SkeletonDynamics *mSkel;
        std::vector<BodyNodeDynamics*> mBodies;
        std::vector<Eigen::Vector3d> mOffsets;
        std::vector<Eigen::Vector3d> mTargets;

        // Matrices to pass to solver
        Eigen::MatrixXd mA;
        Eigen::VectorXd mB;
        Eigen::VectorXd mConstrForce; // solved constraint force in generalized coordinates;
    };
} // namespace dynamics

#endif // #ifndef DYNAMICS_CONSTRAINT_DYNAMICS_H

