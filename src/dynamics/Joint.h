/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/21/2013
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

#ifndef DART_DYNAMICS_JOINT_H
#define DART_DYNAMICS_JOINT_H

#include "math/Geometry.h"
#include "dynamics/GenCoordSystem.h"

namespace dart {
namespace renderer { class RenderInterface; }
namespace dynamics {

class BodyNode;

/// @brief
class Joint : public GenCoordSystem
{
public:
    //--------------------------------------------------------------------------
    // Types
    //--------------------------------------------------------------------------
    /// @brief
    enum JointType
    {
        UNKNOWN,
        WELD,          // 0-dof
        REVOLUTE,      // 1-dof
        PRISMATIC,     // 1-dof
        SCREW,         // 1-dof
        UNIVERSAL,     // 2-dof
        PLANAR,        // 2-dof
        TRANSLATIONAL, // 3-dof
        BALL,          // 3-dof
        EULER,         // 3-dof
        FREE           // 6-dof
    };

    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// @brief
    Joint(const std::string& _name = "");

    /// @brief
    virtual ~Joint();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setName(const std::string& _name);

    /// @brief
    const std::string& getName() const;

    //--------------------------------------------------------------------------
    // Kinematical Properties
    //--------------------------------------------------------------------------
    /// @brief
    JointType getJointType() const;

    /// @brief
    const Eigen::Isometry3d& getLocalTransform() const;

    /// @brief
    const math::Jacobian& getLocalJacobian() const;

    /// @brief
    const Eigen::Vector6d& getLocalVelocity() const;

    /// @brief
    const math::Jacobian& getLocalJacobianFirstDerivative() const;

    /// @brief
    const Eigen::Vector6d& getLocalAcceleration() const;

    /// @brief true if d is present in the dof list for the joint.
    bool isPresent(const GenCoord* _q) const;

    /// @brief Get local index of the dof at this joint; if the dof is not
    /// presented at this joint, return -1.
    int getGenCoordLocalIndex(int _dofSkelIndex) const;

    //--------------------------------------------------------------------------
    // Dynamics Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setPositionLimited(bool _positionLimit);

    /// @brief
    bool isPositionLimited() const;

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setSkeletonIndex(int _idx);

    /// @brief
    int getSkeletonIndex() const;

    /// @brief
    void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T);

    /// @brief
    void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T);

    /// @brief
    const Eigen::Isometry3d& getTransformFromParentBodyNode() const;

    /// @brief
    const Eigen::Isometry3d& getTransformFromChildBodyNode() const;

    // TODO: Not implemented.
    /// @brief
    virtual double getPotentialEnergy() const = 0;

    //--------------------------------------------------------------------------
    // Recursive Kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    /// q --> T(q)
    virtual void updateTransform() = 0;

    /// @brief
    /// q, dq --> S(q), V(q, dq)
    /// V(q, dq) = S(q) * dq
    virtual void updateVelocity() = 0;

    /// @brief
    /// dq, ddq, S(q) --> dS(q), dV(q, dq, ddq)
    /// dV(q, dq, ddq) = dS(q) * dq + S(q) * ddq
    virtual void updateAcceleration() = 0;

    /// @brief
    void setDampingCoefficient(int _idx, double _d);

    /// @brief
    double getDampingCoefficient(int _idx) const;

    /// @brief
    Eigen::VectorXd getDampingForces() const;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    void applyGLTransform(renderer::RenderInterface* _ri);

protected:
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    std::string mName;

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief Unique dof id in skeleton
    int mSkelIndex;

    /// @brief Type of joint e.g. ball, hinge etc.
    JointType mJointType;

    /// @brief
    Eigen::Isometry3d mT_ParentBodyToJoint;

    /// @brief
    Eigen::Isometry3d mT_ChildBodyToJoint;

    //--------------------------------------------------------------------------
    // Kinematics variables
    //--------------------------------------------------------------------------
    /// @brief Local transformation.
    Eigen::Isometry3d mT;

    /// @brief Local generalized body velocity.
    Eigen::Vector6d mV;

    /// @brief Local Jacobian.
    math::Jacobian mS;

    /// @brief Local generalized body acceleration.
    Eigen::Vector6d mdV;

    /// @brief Time derivative of local Jacobian.
    math::Jacobian mdS;

    //--------------------------------------------------------------------------
    // Dynamics variables
    //--------------------------------------------------------------------------
    /// @brief True if the joint limits are enforced in dynamic simulation.
    bool mIsPositionLimited;

    /// @brief
    std::vector<double> mDampingCoefficient;

    /// @brief
    std::vector<double> mSpringStiffness;
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_JOINT_H

