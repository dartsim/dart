/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_SKELETON_H_
#define DART_DYNAMICS_SKELETON_H_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/math/Geometry.h"
#include "dart/dynamics/GenCoordSystem.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class BodyNode;
class Joint;
class Marker;

/// \brief
class Skeleton : public GenCoordSystem {
public:
    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// \brief Constructor
    explicit Skeleton(const std::string& _name = "Skeleton");

    /// \brief Destructor
    virtual ~Skeleton();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// \brief Set name.
    void setName(const std::string& _name);

    /// \brief Get name.
    const std::string& getName() const;

    /// \brief Set whether this skeleton allows self collisions between body
    ///        nodes in this skeleton.
    /// \param[in] _isSelfCollidable True if self collision is allowed.
    void setSelfCollidable(bool _isSelfCollidable);

    /// \brief Get whether this skeleton allows self collisions between body
    ///        nodes in this skeleton.
    /// \return True if self collision is allowed.
    bool isSelfCollidable() const;

    /// \brief Set whether this skeleton will be updated by forward dynamics.
    /// \param[in] _isMobile True if this skeleton is mobile.
    /// \warning This function should be called before this skeleton is added to
    ///          the world. If not, the constraint dynamics algorithm will not
    ///          work. If the user want to change the immobile state after this
    ///          skeleton is added to the world, the user should remove this
    ///          skeleton from the world and add it to the world again.
    void setMobile(bool _isMobile);

    /// \brief Get whether this skeleton will be updated by forward dynamics.
    /// \return True if this skeleton is mobile.
    bool isMobile() const;

    /// \brief Set time step. This timestep is used for implicit joint damping
    ///        force.
    void setTimeStep(double _timeStep);

    /// \brief Get time step.
    double getTimeStep() const;

    /// \brief Set 3-dim gravitational acceleration. The gravity is used for
    ///        calculating gravity force vector of the skeleton.
    void setGravity(const Eigen::Vector3d& _gravity);

    /// \brief Get 3-dim gravitational acceleration.
    const Eigen::Vector3d& getGravity() const;

    /// \brief Get total mass of the skeleton. The total mass is calculated at
    ///        init().
    double getMass() const;

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// \brief
    void addBodyNode(BodyNode* _body);

    /// \brief
    int getNumBodyNodes() const;

    /// \brief
    BodyNode* getRootBodyNode() const;

    /// \brief
    BodyNode* getBodyNode(int _idx) const;

    /// \brief
    BodyNode* getBodyNode(const std::string& _name) const;

    /// \brief
    Joint* getJoint(int _idx) const;

    /// \brief
    Joint* getJoint(const std::string& _name) const;

    /// \brief
    Marker* getMarker(int _i);

    /// \brief
    Marker* getMarker(const std::string& _name) const;

    //--------------------------------------------------------------------------
    // Properties updated by dynamics
    //--------------------------------------------------------------------------
    /// \brief Set the configuration of this skeleton described in generalized
    ///        coordinates. The order of input configuration is determined by
    ///        _id.
    void setConfig(const std::vector<int>& _id, const Eigen::VectorXd& _config);

    /// \brief Set the configuration of this skeleton described in generalized
    ///        coordinates.
    void setConfig(const Eigen::VectorXd& _config);

    /// \brief Get the configuration of this skeleton described in generalized
    ///        coordinates. The returned order of configuration is determined by
    ///        _id.
    Eigen::VectorXd getConfig(const std::vector<int>& _id) const;

    /// \brief Get the configuration of this skeleton described in generalized
    ///        coordinates.
    Eigen::VectorXd getConfig() const;

    /// \brief Set the state of this skeleton described in generalized
    ///        coordinates.
    void setState(const Eigen::VectorXd& _state);

    /// \brief Get the state of this skeleton described in generalized
    ///        coordinates.
    Eigen::VectorXd getState();

    /// \brief Get mass matrix of the skeleton.
    const Eigen::MatrixXd& getMassMatrix();

    /// \brief Get augmented mass matrix of the skeleton. This matrix is used
    ///        in ConstraintDynamics to compute constraint forces. The matrix is
    ///        M + h*D + h*h*K where D is diagonal joint damping coefficient
    ///        matrix, K is diagonal joint stiffness matrix, and h is simulation
    ///        time step.
    const Eigen::MatrixXd& getAugMassMatrix();

    /// \brief Get inverse of mass matrix of the skeleton.
    const Eigen::MatrixXd& getInvMassMatrix();

    /// \brief Get inverse of augmented mass matrix of the skeleton.
    const Eigen::MatrixXd& getInvAugMassMatrix();

    /// \brief Get Coriolis force vector of the skeleton.
    const Eigen::VectorXd& getCoriolisForceVector();

    /// \brief Get gravity force vector of the skeleton.
    const Eigen::VectorXd& getGravityForceVector();

    /// \brief Get combined vector of Coriolis force and gravity force of the
    ///        skeleton.
    const Eigen::VectorXd& getCombinedVector();

    /// \brief Get external force vector of the skeleton.
    const Eigen::VectorXd& getExternalForceVector();

    /// \brief Get internal force vector of the skeleton.
    Eigen::VectorXd getInternalForceVector() const;

    /// \brief Get damping force of the skeleton.
    const Eigen::VectorXd& getDampingForceVector();

    /// \brief Get constraint force vector.
    const Eigen::VectorXd& getConstraintForceVector();

    /// \brief Set internal force vector.
    void setInternalForceVector(const Eigen::VectorXd& _forces);

    /// \brief Set upper limit of the internal force vector.
    void setMinInternalForceVector(const Eigen::VectorXd& _minForces);

    /// \brief Get lower limit of the internal force vector.
    Eigen::VectorXd getMinInternalForces() const;

    /// \brief Set upper limit of the internal force vector.
    void setMaxInternalForceVector(const Eigen::VectorXd& _maxForces);

    /// \brief Get upper limit of the internal force vector.
    Eigen::VectorXd getMaxInternalForceVector() const;

    /// \brief Clear internal force vector.
    void clearInternalForceVector();

    /// \brief Clear all the contacts of external force vector.
    ///        Automatically called after each (forward/inverse) dynamics
    ///        computation, which marks the end of a cycle.
    void clearExternalForceVector();

    /// \brief Set constraint force vector.
    void setConstraintForceVector(const Eigen::VectorXd& _Fc);

    /// \brief Get skeleton's COM w.r.t. world frame.
    Eigen::Vector3d getWorldCOM();

    /// \brief Get skeleton's COM velocity w.r.t. world frame.
    Eigen::Vector3d getWorldCOMVelocity();

    /// \brief Get skeleton's COM acceleration w.r.t. world frame.
    Eigen::Vector3d getWorldCOMAcceleration();

    /// \brief Get skeleton's COM Jacobian w.r.t. world frame.
    Eigen::MatrixXd getWorldCOMJacobian();

    /// \brief Get skeleton's COM Jacobian time derivative w.r.t. world frame.
    Eigen::MatrixXd getWorldCOMJacobianTimeDeriv();

    /// \brief Get kinetic energy of this skeleton.
    virtual double getKineticEnergy() const;

    /// \brief Get potential energy of this skeleton.
    virtual double getPotentialEnergy() const;

    //--------------------------------------------------------------------------
    // Recursive dynamics algorithms
    //--------------------------------------------------------------------------
    /// \brief
    void init(double _timeStep = 0.001, const Eigen::Vector3d& _gravity =
            Eigen::Vector3d(0.0, 0.0, -9.81));

    /// \brief (q, dq, ddq) --> (tau)
    void computeInverseDynamicsLinear(bool _computeJacobian = true,
                                      bool _computeJacobianDeriv = true,
                                      bool _withExternalForces = false,
                                      bool _withDampingForces = false);

    /// \brief (q, dq, tau) --> (ddq)
    void computeForwardDynamics();

    //--------------------------------------------------------------------------
    // Rendering
    //--------------------------------------------------------------------------
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true) const;

    /// \brief
    void drawMarkers(renderer::RenderInterface* _ri = NULL,
                     const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                     bool _useDefaultColor = true) const;

protected:
    /// \brief Name
    std::string mName;

    /// \brief
    bool mIsSelfCollidable;

    /// \brief List of body nodes in the skeleton.
    std::vector<BodyNode*> mBodyNodes;

    /// \brief If the skeleton is not mobile, its dynamic effect is equivalent
    ///        to having infinite mass. If the configuration of an immobile
    ///        skeleton are manually changed, the collision results might not be
    ///        correct.
    bool mIsMobile;

    /// \brief Time step for implicit joint damping force.
    double mTimeStep;

    /// \brief Gravity vector.
    Eigen::Vector3d mGravity;

    /// \brief Total mass.
    double mTotalMass;

    /// \brief Mass matrix for the skeleton.
    Eigen::MatrixXd mM;

    /// \brief Dirty flag for the mass matrix.
    bool mIsMassMatrixDirty;

    /// \brief Mass matrix for the skeleton.
    Eigen::MatrixXd mAugM;

    /// \brief Dirty flag for the mass matrix.
    bool mIsAugMassMatrixDirty;

    /// \brief Inverse of mass matrix for the skeleton.
    Eigen::MatrixXd mInvM;

    /// \brief Dirty flag for the inverse of mass matrix.
    bool mIsInvMassMatrixDirty;

    /// \brief Inverse of augmented mass matrix for the skeleton.
    Eigen::MatrixXd mInvAugM;

    /// \brief Dirty flag for the inverse of augmented mass matrix.
    bool mIsInvAugMassMatrixDirty;

    /// \brief Coriolis vector for the skeleton which is C(q,dq)*dq.
    Eigen::VectorXd mCvec;

    /// \brief Dirty flag for the Coriolis force vector.
    bool mIsCoriolisVectorDirty;

    /// \brief Gravity vector for the skeleton; computed in nonrecursive
    ///        dynamics only.
    Eigen::VectorXd mG;

    /// \brief Dirty flag for the gravity force vector.
    bool mIsGravityForceVectorDirty;

    /// \brief Combined coriolis and gravity vector which is C(q, dq)*dq + g(q).
    Eigen::VectorXd mCg;

    /// \brief Dirty flag for the combined vector of Coriolis and gravity.
    bool mIsCombinedVectorDirty;

    /// \brief External force vector for the skeleton.
    Eigen::VectorXd mFext;

    /// \brief Dirty flag for the external force vector.
    bool mIsExternalForceVectorDirty;

    /// \brief Constraint force vector.
    Eigen::VectorXd mFc;

    /// \brief Damping force vector.
    Eigen::VectorXd mFd;

    /// \brief Dirty flag for the damping force vector.
    bool mIsDampingForceVectorDirty;

    /// \brief Update mass matrix of the skeleton.
    virtual void updateMassMatrix();

    /// \brief Update augmented mass matrix of the skeleton.
    virtual void updateAugMassMatrix();

    /// \brief Update inverse of mass matrix of the skeleton.
    virtual void updateInvMassMatrix();

    /// \brief Update inverse of augmented mass matrix of the skeleton.
    virtual void updateInvAugMassMatrix();

    /// \brief Update Coriolis force vector of the skeleton.
    virtual void updateCoriolisForceVector();

    /// \brief Update gravity force vector of the skeleton.
    virtual void updateGravityForceVector();

    /// \brief Update combined vector of the skeletong.
    virtual void updateCombinedVector();

    /// \brief update external force vector to generalized torques.
    virtual void updateExternalForceVector();

    /// \brief Update damping force vector.
    virtual void updateDampingForceVector();

public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SKELETON_H_
