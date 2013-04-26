/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu
 * Date:
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
#include "Constraint.h"

#include <vector>
#include <Eigen/Dense>
#include <dynamics/Constraint.h>
#include <collision/CollisionSkeleton.h>

namespace kinematics {
    class BodyNode;
} // namespace kinematics

namespace lcpsolver {
    class LCPSolver;
} //namespace lcpsolver


namespace dynamics {
    class SkeletonDynamics;
    class BodyNodeDynamics;

    class ConstraintDynamics {
    public:
        ConstraintDynamics(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu = 1.0, int _d = 4);
        virtual ~ConstraintDynamics();
        void computeConstraintForces();            
        inline Eigen::VectorXd getTotalConstraintForce(int _skelIndex) const { 
            return mTotalConstrForces[_skelIndex]; 
        }
        inline Eigen::VectorXd getContactForce(int _skelIndex) const { 
            return mContactForces[_skelIndex]; 
        }
        inline collision_checking::SkeletonCollision* getCollisionChecker() const {
            return mCollisionChecker; 
        }
        inline int getNumContacts() const { 
            return mCollisionChecker->getNumContact();
        }

        inline Constraint* getConstraint(int _index) const { return mConstraints[_index]; }; 
        void addConstraint(Constraint *_constr);
        void deleteConstraint(int _index);

    private:
        void initialize();

        void updateMassMat();
        void updateTauStar();

        void fillMatrices();
        bool solve();
        void applySolution();

        inline int getTotalNumDofs() const { return mIndices[mIndices.size() - 1]; }

        Eigen::MatrixXd getJacobian(kinematics::BodyNode* node, const Eigen::Vector3d& p);
        void updateNBMatrices();
        Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& p, const Eigen::Vector3d& n) ; // gets a matrix of tangent dirs.
        Eigen::MatrixXd getContactMatrix() const; // E matrix
        Eigen::MatrixXd getMuMatrix() const; // mu matrix
        void updateConstraintTerms();
        void computeConstraintWithoutContact();

            
        std::vector<SkeletonDynamics*> mSkels;
        std::vector<int> mBodyIndexToSkelIndex;
        std::vector<int> mIndices;
        collision_checking::SkeletonCollision* mCollisionChecker;
        double mDt; // timestep
        double mMu; // friction coeff.
        int mNumDir; // number of basis directions            

        // Cached (aggregated) mass/tau matrices
        Eigen::MatrixXd mMInv;
        Eigen::VectorXd mTauStar;
        Eigen::MatrixXd mN;
        Eigen::MatrixXd mB;
            
        // Matrices to pass to solver
        Eigen::MatrixXd mA;
        Eigen::VectorXd mQBar;
        Eigen::VectorXd mX;

        std::vector<Eigen::VectorXd> mContactForces; 
        std::vector<Eigen::VectorXd> mTotalConstrForces; // solved constraint force in generalized coordinates; mConstrForces[i] is the constraint force for the ith skeleton
        // constraints
        std::vector<Constraint*> mConstraints;
        int mTotalRows;

        Eigen::MatrixXd mZ; // N x N
        Eigen::VectorXd mTauHat; // M x 1
        Eigen::MatrixXd mGInv; // M x M
        std::vector<Eigen::MatrixXd> mJMInv; // M x N
        std::vector<Eigen::MatrixXd> mJ; // M x N
        std::vector<Eigen::MatrixXd> mPreJ; // M x N
        Eigen::VectorXd mC; // M * 1
        Eigen::VectorXd mCDot; // M * 1
    };
} // namespace dynamics

#endif // #ifndef DYNAMICS_CONSTRAINT_DYNAMICS_H

