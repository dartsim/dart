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

#ifndef DART_DYNAMICS_BODYNODE_DYNAMICS_H
#define DART_DYNAMICS_BODYNODE_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>
#include "kinematics/BodyNode.h"
#include "math/EigenHelper.h"
#include "math/UtilsMath.h"

namespace dynamics
{

/// @brief BodyNodeDynamics class represents a single node of the skeleton for
/// dynamics.
class BodyNodeDynamics : public kinematics::BodyNode
{
public:
    // We need this aligned allocator because we have Matrix4d as members in
    // this class.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief Default constructor. The name can be up to 128.
    BodyNodeDynamics(const char* const _name = NULL);

    /// @brief Default destructor
    virtual ~BodyNodeDynamics();

    // Following functions called automatically by the skeleton: computeInverseDynamicsLinear and computeDynamics respectively
    /// @brief Initialize data structures for linear inverse dynamics computation.
    void initInverseDynamics();

    /// @brief initialize data structures for non-recursive dynamics computation
    void initDynamics();

    /// @brief Set gravity mode.
    /// @param[in] _gravityMode
    /// TODO: Not implemented yet!
    void setGravityMode(bool _gravityMode) { mGravityMode = _gravityMode; }

    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// @return
    /// TODO: Not implemented yet!
    bool getGravityMode() const { return mGravityMode; }

    //--------------------------------------------------------------------------
    // Inverse Dynamics
    //--------------------------------------------------------------------------
    Eigen::Vector3d mVelBody;      ///< linear velocity expressed in the *local frame* of the body
    Eigen::Vector3d mVelDotBody;   ///< linear acceleration expressed in the *local frame* of the body
    Eigen::Vector3d mOmegaBody;    ///< angular velocity expressed in the *local frame* of the body
    Eigen::Vector3d mOmegaDotBody; ///< angular acceleration expressed in the *local frame* of the body

    Eigen::MatrixXd mJwJoint;         ///< Jacobian matrix for the parent joint
    Eigen::MatrixXd mJwDotJoint;      ///< Time derivative of the Jacobian matrix for the parent joint
    Eigen::Vector3d mForceJointBody;  ///< the constraint joint force in Cartesian coordinates, expressed in the local frame of the body instead of the joint
    Eigen::Vector3d mTorqueJointBody; ///< the torque in Cartesian coordinates for the joint expressed in the local frame of the body instead of the joint
    Eigen::Vector3d mExtForceBody;    ///< the external Cartesian force applied to the body; usually computed from mContacts
    Eigen::Vector3d mExtTorqueBody;   ///< the external Cartesian torque applied to the body; usually directly supplied from outside; contribution of the linear force will be considered later in the computation

    /// @brief Computes the velocities in the first pass of the algorithm.
    /// Also computes Transform W etc using updateTransform.
    /// Computes Jacobians Jv and Jw if the flag is true.
    /// Replaces updateFirstDerivatives of non-recursive dynamics.
    void computeInvDynVelocities(const Eigen::Vector3d &_gravity,
                                 const Eigen::VectorXd *_qdot,
                                 const Eigen::VectorXd *_qdotdot,
                                 bool _computeJacobians = true );

    // TODO: NOT IMPLEMENTED.
    void computeInvDynVelocities_2(const Eigen::Vector3d &_gravity,
                                   const Eigen::VectorXd *_qdot,
                                   const Eigen::VectorXd *_qdotdot,
                                   bool _computeJacobians = true );

    /// @brief Computes the forces in the second pass of the algorithm.
    void computeInvDynForces(const Eigen::Vector3d &_gravity,
                             const Eigen::VectorXd *_qdot,
                             const Eigen::VectorXd *_qdotdot,
                             bool _withExternalForces );

    // non-recursive Dynamics formulation - M*qdd + C*qdot + g = 0
    void updateSecondDerivatives();  ///< Update the second derivatives of the transformations
    void updateSecondDerivatives(Eigen::Vector3d _offset);  ///< Update the second derivatives of the transformations

    Eigen::MatrixXd mM;    ///< Mass matrix of dimension numDependentDofs x numDependentDofs; to be added carefully to the skeleton mass matrix
    Eigen::MatrixXd mC;    ///< Coriolis matrix of dimension numDependentDofs x numDependentDofs; to be added carefully to the skeleton Coriolis matrix
    Eigen::VectorXd mCvec; ///< Coriolis vector of dimension numDependentDofs x 1; mCvec = mC*qdot
    Eigen::VectorXd mG;    ///< Gravity vector or generalized gravity forces; dimension numDependentDofs x 1
    Eigen::VectorXd mFext; ///< generalized external forces this node contributes: J^TF; dimension numDependentDofs x 1

    void evalMassMatrix();  ///< evaluates the mass matrix mM
    void evalCoriolisMatrix(const Eigen::VectorXd &_qDotSkel); ///< evaluates the Coriolis matrix mC
    void evalCoriolisVector(const Eigen::VectorXd &_qDotSkel); ///< evaluates the Coriolis vector mCvec directy: i.e. shortcut for mC*qdot
    void evalGravityVector(const Eigen::Vector3d &_gravity);   ///< evaluates the gravity vector mG in the generalized coordinates
    void evalExternalForces(Eigen::VectorXd& _extForce); ///< evaluates the external forces mFext in the generalized coordinates as J^TF
    void evalExternalForcesRecursive(Eigen::VectorXd& _extForce ); ///< evaluates the external forces mFext in the generalized coordinates recursively

    void jointCartesianToGeneralized(const Eigen::Vector3d& _cForce,
                                     Eigen::VectorXd& _gForce,
                                     bool _isTorque = true); ///< convert cartesian forces in joint frame to generalized forces
    void bodyCartesianToGeneralized(const Eigen::Vector3d& _cForce,
                                    Eigen::VectorXd& _gForce,
                                    bool _isTorque = true);  ///< convert cartesian forces in body com frame to generalized forces
    void getGeneralized(Eigen::VectorXd& _gForce); ///< convert coriolis forces in cartesian space to generalized forces

    // add functions to add to the existing *full* matrices typically for the entire skeleton
    void aggregateMass(Eigen::MatrixXd &_M);
    void aggregateCoriolis(Eigen::MatrixXd &_C);
    void aggregateCoriolisVec(Eigen::VectorXd &_Cvec);
    void aggregateGravity(Eigen::VectorXd &_G);

    // add and remove contact points where forces are applied
    void addExtForce( const Eigen::Vector3d& _offset,
                      const Eigen::Vector3d& _force,
                      bool _isOffsetLocal = true,
                      bool _isForceLocal = false ); ///< apply linear Cartesian forces to this node. A force is defined by a point of application and a force vector. The last two parameters specify frames of the first two parameters. Coordinate transformations are applied when needed. The point of application and the force in local coordinates are stored in mContacts. When conversion is needed, make sure the transformations are avaialble
    void addExtTorque( const Eigen::Vector3d& _torque, bool _isLocal); ///< apply Cartesian torque to the node. The torque in local coordinates is accumulated in mExtTorqueBody
    void clearExternalForces(); ///< clean up structures that store external forces: mContacts, mFext, mExtForceBody and mExtTorqueBody; called from @SkeletonDynamics::clearExternalForces

    Eigen::Vector3d evalLinMomentum();
    Eigen::Vector3d evalAngMomentum(Eigen::Vector3d _pivot);

    inline Eigen::MatrixXd getJvDeriv(int _qIndex) const { return mJvq[_qIndex]; }

protected:
    bool mInitializedInvDyn;            ///< true if linear inverse dynamics is initialized; init functions initialize only if false
    bool mInitializedNonRecursiveDyn;   ///< true if non recursive dynamics is initialized; init function initialize only if false

    // non-recursive Dynamics formulation - second derivatives
    EIGEN_VV_MAT4D mTqq;  ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
    EIGEN_VV_MAT4D mWqq;  ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
    std::vector<Eigen::MatrixXd> mJvq; ///< Linear Jacobian derivative wrt to all the dependent dofs
    std::vector<Eigen::MatrixXd> mJwq; ///< Angular Jacobian derivative wrt to all the dependent dofs
    Eigen::MatrixXd mJvDot; ///< Time derivative of the Linear velocity Jacobian
    Eigen::MatrixXd mJwDot; ///< Time derivative of the Angular velocity Jacobian

    /// @brief List of contact points where external forces are applied.
    /// contact points are a pair of (local point offset, Cartesian force in
    /// local coordinates).
    std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > mContacts;

    Eigen::Matrix4d getLocalSecondDeriv(const kinematics::Dof *_q1,
                                        const kinematics::Dof *_q2) const;

    void evalJacDerivLin(int _qi, Eigen::Vector3d _offset); ///< Evaluate the first derivatives of the linear Jacobian wrt to the dependent dofs
    void evalJacDerivAng(int _qi);                          ///< Evaluate the first derivatives of the angular Jacobian wrt to the dependent dofs
    void evalJacDotLin(const Eigen::VectorXd &_qDotSkel);   ///< Evaluate time derivative of the linear Jacobian of this body node (num cols == num dependent dofs)
    void evalJacDotAng(const Eigen::VectorXd &_qDotSkel);   ///< Evaluate time derivative of the angular Jacobian of this body node (num cols == num dependent dofs)

    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// TODO: Not implemented yet!
    bool mGravityMode;
};

} // namespace dynamics

#endif // #ifndef DART_DYNAMICS_BODYNODE_DYNAMICS_H
