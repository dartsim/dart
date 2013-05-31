/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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

#ifndef DART_DYNAMICS_SKELETONDYNAMICS_H
#define DART_DYNAMICS_SKELETONDYNAMICS_H

#include <vector>
#include <Eigen/Dense>

#include "kinematics/Skeleton.h"

namespace dynamics
{

/// @brief
class SkeletonDynamics : public kinematics::Skeleton
{
public:
    /// @brief
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    SkeletonDynamics();

    /// @brief
    virtual ~SkeletonDynamics();

    /// @brief Creates a derived class of BodyNode that calculates the dynamics
    /// quantities
    virtual kinematics::BodyNode* createBodyNode(
            const char* const _name = NULL);

    /// @brief
    void initDynamics();

    /// @brief Inverse dynamics computation.
    /// Runs recursive inverse dynamics algorithm and returns the generalized
    /// forces; if qdd is NULL, it is treated as zero; also computes Jacobian Jv
    /// and Jw in iterative manner if the flag is true i.e. replaces
    /// updateFirstDerivatives of non-recursive dynamics; when
    ///_withExternalForces is true, external forces will be accounted for in the
    /// returned generalized forces; when _withExternalForces is false, only the
    /// sum of Corolis force and gravity is returned.
    Eigen::VectorXd computeInverseDynamicsLinear(
            const Eigen::Vector3d &_gravity,
            const Eigen::VectorXd *_qdot,
            const Eigen::VectorXd *_qdotdot=NULL,
            bool _computeJacobians=true,
            bool _withExternalForces=false);

    /// @brief Compute equations of motion matrices/vectors: M, C/Cvec, g in
    /// M*qdd + C*qd + g; if _useInvDynamics==true, uses
    ///computeInverseDynamicsLinear to compute C*qd+g term directly; else uses
    /// expensive generic computation of non-recursive dynamics. Note that
    ///different quantities are computed using different algorithms. At the end,
    /// mass matrix M, Coriolis force plus gravity Cg, and the external force
    /// Fext will be ready to use. The generalized force of gravity g is also
    ///updated if nonrecursive formula is used.
    void computeDynamics(const Eigen::Vector3d &_gravity,
                         const Eigen::VectorXd &_qdot,
                         bool _useInvDynamics = true,
                         bool _calcMInv = true);

    /// @brief Evaluate external forces to generalized torques.
    /// Similarly to the inverse dynamics computation, when _useRecursive is
    /// true, a recursive algorithm is used; else the jacobian is used to do the
    /// conversion: tau = J^{T}F. Highly recommand to use this function after
    /// the respective (recursive or nonrecursive) dynamics computation because
    /// the necessary Jacobians will be ready. Extra care is needed to make sure
    /// the required quantities are up-to-date when using this function alone.
    void evalExternalForces( bool _useRecursive );

    /// @brief Clear all the contacts of external forces.
    /// Automatically called after each (forward/inverse) dynamics computation,
    /// which marks the end of a cycle.
    void clearExternalForces();

    /// @brief Clamp joint rotations to the range of [-pi, pi].
    /// It's particularly useful for exponential map because the system will
    /// become unstable if the exponential map rotaion is outside this range.
    /// For euler angles, the dof values can directly add or subtract 2*pi.
    /// For exponential map, once the rotation magnitude is changed, the
    /// velocity needs to change accordingly to represent the same angular
    /// velocity. This function requires the updated transformations.
    void clampRotation( Eigen::VectorXd& _q, Eigen::VectorXd& _qdot);


    Eigen::MatrixXd getMassMatrix() const { return mM; }
    Eigen::MatrixXd getInvMassMatrix() const { return mMInv; }
    Eigen::MatrixXd getCoriolisMatrix() const { return mC; }
    Eigen::VectorXd getCoriolisVector() const { return mCvec; }
    Eigen::VectorXd getGravityVector() const { return mG; }
    Eigen::VectorXd getCombinedVector() const { return mCg; }
    Eigen::VectorXd getExternalForces() const { return mFext; }
    Eigen::VectorXd getInternalForces() const { return get_tau(); }
    Eigen::VectorXd getDampingForces() const;

    bool getImmobileState() const { return mImmobile; }
    void setImmobileState(bool _s) { mImmobile = _s; }
    bool getJointLimitState() const { return mJointLimit; }
    void setJointLimitState(bool _s) { mJointLimit = _s; }
    void setInternalForces(const Eigen::VectorXd& _forces) { set_tau(_forces); }
    void setMinInternalForces(Eigen::VectorXd _minForces) { set_tauMin(_minForces); }
    Eigen::VectorXd getMinInternalForces() const { return get_tauMin(); }
    void setMaxInternalForces(Eigen::VectorXd _maxForces) { set_tauMax(_maxForces); }
    Eigen::VectorXd getMaxInternalForces() const { return get_tauMax(); }

protected:
    Eigen::MatrixXd mM;    ///< Mass matrix for the skeleton
    Eigen::MatrixXd mMInv; ///< Inverse of mass matrix for the skeleton
    Eigen::MatrixXd mC;    ///< Coriolis matrix for the skeleton; not being used currently
    Eigen::VectorXd mCvec; ///< Coriolis vector for the skeleton == mC*qdot
    Eigen::VectorXd mG;    ///< Gravity vector for the skeleton; computed in nonrecursive dynamics only
    Eigen::VectorXd mCg;   ///< combined coriolis and gravity term == mC*qdot + g
    Eigen::VectorXd mFext; ///< external forces vector for the skeleton

    bool mImmobile;   ///< If the skeleton is immobile, its dynamic effect is equivalent to having infinite mass; if the DOFs of an immobile skeleton are manually changed, the collision results might not be correct
    bool mJointLimit; ///<True if the joint limits are enforced in dynamic simulation

};

} // namespace dynamics

#endif // #ifndef DART_DYNAMICS_SKELETONDYNAMICS_H
