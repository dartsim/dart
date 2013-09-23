/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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

#ifndef DART_DYNAMICS_SKELETON_H
#define DART_DYNAMICS_SKELETON_H

#include <vector>
#include <Eigen/Dense>
#include "math/Geometry.h"
#include "dynamics/GenCoordSystem.h"

namespace dart {
namespace renderer { class RenderInterface; }
namespace dynamics {

class BodyNode;
class Joint;
class Marker;

/// @brief
class Skeleton : public GenCoordSystem
{
public:
    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// @brief Constructor
    Skeleton(const std::string& _name = "Skeleton");

    /// @brief Destructor
    virtual ~Skeleton();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setName(const std::string& _name);

    /// @brief
    const std::string& getName() const;

    /// @brief
    void setSelfCollidable(bool _selfCollidable);

    /// @brief
    bool getSelfCollidable() const;

    /// @brief
    void setImmobileState(bool _immobile);

    /// @brief
    bool getImmobileState() const;

    /// @brief
    double getMass() const;

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    void addBodyNode(BodyNode* _body);

    /// @brief
    int getNumBodyNodes() const;

    /// @brief
    BodyNode* getRootBodyNode() const;

    /// @brief
    BodyNode* getBodyNode(int _idx) const;

    /// @brief
    BodyNode* getBodyNode(const std::string& _name) const;

    /// @brief
    Joint* getJoint(int _idx) const;

    /// @brief
    Joint* getJoint(const std::string& _name) const;

    /// @brief
    Marker* getMarker(int _i);

    /// @brief
    Marker* getMarker(const std::string& _name) const;

    //--------------------------------------------------------------------------
    // Properties updated by dynamics
    //--------------------------------------------------------------------------
    /// @brief
    void setConfig(const std::vector<int>& _id, Eigen::VectorXd _vals,
                   bool _calcTrans = true, bool _calcDeriv = true);

    /// @brief
    void setConfig(const Eigen::VectorXd& _pose,
                   bool bCalcTrans = true, bool bCalcDeriv = true);

    /// @brief Get the configuration of this skeleton described in generalized
    /// coordinates. The returned order of configuration is determined by _id.
    /// If you just want the configuration in original order then use
    /// GenCoordSystem::get_q().
    Eigen::VectorXd getConfig(const std::vector<int>& _id) const;

    /// @brief
    Eigen::MatrixXd getMassMatrix() const;

    /// @brief
    Eigen::MatrixXd getInvMassMatrix() const;

    /// @brief
    Eigen::MatrixXd getCoriolisMatrix() const;

    /// @brief
    Eigen::VectorXd getCoriolisVector() const;

    /// @brief
    Eigen::VectorXd getGravityVector() const;

    /// @brief
    Eigen::VectorXd getCombinedVector() const;

    /// @brief
    Eigen::VectorXd getExternalForces() const;

    /// @brief
    Eigen::VectorXd getInternalForces() const;

    /// @brief
    Eigen::VectorXd getDampingForces() const;

    /// @brief
    Eigen::VectorXd getConstraintForces() const;

    /// @brief
    void setInternalForces(const Eigen::VectorXd& _forces);

    /// @brief
    void setMinInternalForces(Eigen::VectorXd _minForces);

    /// @brief
    Eigen::VectorXd getMinInternalForces() const;

    /// @brief
    void setMaxInternalForces(Eigen::VectorXd _maxForces);

    /// @brief
    Eigen::VectorXd getMaxInternalForces() const;

    /// @brief
    void clearInternalForces();

    /// @brief
    void setConstraintForces(const Eigen::VectorXd& _Fc);

    /// @brief
    double getKineticEnergy() const;

    // TODO: Not implemented.
    /// @brief
    double getPotentialEnergy() const;

    /// @brief
    Eigen::Vector3d getWorldCOM();

    //--------------------------------------------------------------------------
    // Recursive dynamics algorithms
    //--------------------------------------------------------------------------
    /// @brief
    void init();

    /// @brief Update joint and body kinematics.
    void updateForwardKinematics(bool _firstDerivative = true,
                                 bool _secondDerivative = true);

    /// @brief (q, dq, ddq) --> (tau)
    void computeInverseDynamicsLinear(const Eigen::Vector3d& _gravity,
                                bool _computeJacobian = true,
                                bool _computeJacobianDeriv = true,
                                bool _withExternalForces = false,
                                bool _withDampingForces = false);

    /// @brief Inverse dynamics computation.
    /// Runs recursive inverse dynamics algorithm and returns the generalized
    /// forces; if qdd is NULL, it is treated as zero; also computes Jacobian Jv
    /// and Jw in iterative manner if the flag is true i.e. replaces
    /// updateFirstDerivatives of non-recursive dynamics; when
    ///_withExternalForces is true, external forces will be accounted for in the
    /// returned generalized forces; when _withExternalForces is false, only the
    /// sum of Corolis force and gravity is returned.
    Eigen::VectorXd computeInverseDynamicsLinear(
            const Eigen::Vector3d& _gravity,
            const Eigen::VectorXd* _qdot,
            const Eigen::VectorXd* _qdotdot = NULL,
            bool _computeJacobians = true,
            bool _withExternalForces = false,
            bool _withDampingForces = false);

    /// @brief Evaluate external forces to generalized torques.
    /// Similarly to the inverse dynamics computation, when _useRecursive is
    /// true, a recursive algorithm is used; else the jacobian is used to do the
    /// conversion: tau = J^{T}F. Highly recommand to use this function after
    /// the respective (recursive or nonrecursive) dynamics computation because
    /// the necessary Jacobians will be ready. Extra care is needed to make sure
    /// the required quantities are up-to-date when using this function alone.
    void updateExternalForces();

    /// @brief
    void updateDampingForces();

    /// @brief Clear all the contacts of external forces.
    /// Automatically called after each (forward/inverse) dynamics computation,
    /// which marks the end of a cycle.
    void clearExternalForces();

    /// @brief (q, dq) --> M, C, G
    void computeEquationsOfMotionID(const Eigen::Vector3d& _gravity);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamicsID(const Eigen::Vector3d& _gravity,
                                  bool _equationsOfMotion = true);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamicsFS(const Eigen::Vector3d& _gravity,
                                  bool _equationsOfMotion = true);

    //--------------------------------------------------------------------------
    // Rendering
    //--------------------------------------------------------------------------
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true) const;

    /// @brief
    void drawMarkers(renderer::RenderInterface* _ri = NULL,
                     const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                     bool _useDefaultColor = true ) const;

protected:
    /// @brief
    std::string mName;

    /// @brief
    bool mSelfCollidable;

    //--------------------------------------------------------------------------
    // Structual Properties
    //--------------------------------------------------------------------------
    /// @brief
    std::vector<BodyNode*> mBodyNodes;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief If the skeleton is immobile, its dynamic effect is equivalent to
    /// having infinite mass. If the DOFs of an immobile skeleton are manually
    /// changed, the collision results might not be correct.
    bool mImmobile;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    double mTotalMass;

    /// @brief Mass matrix for the skeleton.
    Eigen::MatrixXd mM;

    /// @brief Inverse of mass matrix for the skeleton.
    Eigen::MatrixXd mMInv;

    /// @brief Coriolis matrix for the skeleton; not being used currently
    Eigen::MatrixXd mC;

    /// @brief Coriolis vector for the skeleton == mC*qdot.
    Eigen::VectorXd mCvec;

    /// @brief Gravity vector for the skeleton; computed in nonrecursive
    /// dynamics only.
    Eigen::VectorXd mG;

    /// @brief Combined coriolis and gravity term == mC*qdot + g.
    Eigen::VectorXd mCg;

    /// @brief External forces vector for the skeleton.
    Eigen::VectorXd mFext;

    /// @brief
    Eigen::VectorXd mFc;

    /// @brief
    Eigen::VectorXd mDampingForce;

public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_SKELETON_H

