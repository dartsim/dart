/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 10/21/2011
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

#ifndef DYNAMICS_CONTACT_DYNAMICS_H
#define DYNAMICS_CONTACT_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

namespace kinematics {
    class BodyNode;
} // namespace kinematics

namespace collision_checking {
    class SkeletonCollision;
} // namespace collision

namespace lcpsolver {
    class LCPSolver;
} //namespace lcpsolver


/*
  // Sample Usage
  dynamics::ContactDynamics contacts(skels, dt);
  contacts.applyContactForces();
 */
namespace dynamics {
    class SkeletonDynamics;
    
    class ContactDynamics {
    public:
        ContactDynamics(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu = 0.8, int _d = 4);
        virtual ~ContactDynamics();
        void applyContactForces();
        void reset();
        inline Eigen::VectorXd getConstraintForce(int _skelIndex) const { return mConstrForces[_skelIndex]; }
        inline collision_checking::SkeletonCollision* getCollisionChecker() const {return mCollision; }
        
    private:
        void initialize();
        void destroy();
       
        void updateMassMat();
        void updateTauStar();
       
        void fillMatrices();
        bool solve();
        void applySolution();

        inline int getNumSkels() const { return mSkels.size(); }
        inline int getNumTotalDofs() const { return mIndices[mIndices.size() - 1]; }
        inline int getNumContactDirections() const { return mNumDir; }
        int getNumContacts() const;

        Eigen::MatrixXd getJacobian(kinematics::BodyNode* node, const Eigen::Vector3d& p);

        // Helper functions to compute all of the matrices
        // Notation is similar to that used in derivation:
        // Mqddot + Cqdot + kq = tau + (J^T)(f_n)N + (J^T)D(f_d) -> Mqdot = tau* + N(f_n) + B(f_d)
        //        inline Eigen::MatrixXd getMassMatrix() const { return mM; } // M matrix
        //        inline Eigen::VectorXd getTauStarVector() const { return mTauStar; } // T* vector (not T)
        void updateNormalMatrix(); // N matrix
        void updateBasisMatrix() ; // B matrix 
        Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& p, const Eigen::Vector3d& n) ; // gets a matrix of tangent dirs.
        Eigen::MatrixXd getContactMatrix() const; // E matrix
        Eigen::MatrixXd getMuMatrix() const; // mu matrix

        void cleanupContact();
        void penaltyMethod();
        
        std::vector<SkeletonDynamics*> mSkels;
        std::vector<int> mBodyIndexToSkelIndex;
        std::vector<int> mIndices;
        collision_checking::SkeletonCollision* mCollision;
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
        std::vector<Eigen::VectorXd> mConstrForces; // solved constraint force in generalized coordinates; mConstrForces[i] is the constraint force for the ith skeleton
    }; 
} // namespace dynamics

#endif // #ifndef DYNAMICS_CONTACT_DYNAMICS_H

