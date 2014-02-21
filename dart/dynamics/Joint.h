/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef DART_DYNAMICS_JOINT_H_
#define DART_DYNAMICS_JOINT_H_

#include <string>
#include <vector>

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

/// \brief
class Joint : public GenCoordSystem {
public:
  friend class BodyNode;

  //--------------------------------------------------------------------------
  // Types
  //--------------------------------------------------------------------------
  /// \brief
  enum JointType {
    WELD,           // 0-dof
    REVOLUTE,       // 1-dof
    PRISMATIC,      // 1-dof
    SCREW,          // 1-dof
    UNIVERSAL,      // 2-dof
    TRANSLATIONAL,  // 3-dof
    BALL,           // 3-dof
    PLANAR,         // 3-dof
    EULER,          // 3-dof
    FREE            // 6-dof
  };

  //--------------------------------------------------------------------------
  // Constructor and Destructor
  //--------------------------------------------------------------------------
  /// \brief
  Joint(JointType _type, const std::string& _name = "Noname Joint");

  /// \brief
  virtual ~Joint();

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  /// \brief
  void setName(const std::string& _name);

  /// \brief
  const std::string& getName() const;

  //--------------------------------------------------------------------------
  // Kinematical Properties
  //--------------------------------------------------------------------------
  /// \brief
  JointType getJointType() const;

  /// \brief
  const Eigen::Isometry3d& getLocalTransform() const;

  /// \brief
  const math::Jacobian& getLocalJacobian() const;

  /// \brief
  const math::Jacobian& getLocalJacobianTimeDeriv() const;

  /// \brief Get whether this joint contains _genCoord.
  /// \param[in] Generalized coordinate to see.
  /// \return True if this joint contains _genCoord.
  bool contains(const GenCoord* _genCoord) const;

  /// \brief Get local index of the dof at this joint; if the dof is not
  /// presented at this joint, return -1.
  int getGenCoordLocalIndex(int _dofSkelIndex) const;

  //--------------------------------------------------------------------------
  // Dynamics Properties
  //--------------------------------------------------------------------------
  /// \brief
  void setPositionLimited(bool _isPositionLimited);

  /// \brief
  bool isPositionLimited() const;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------
  /// \brief
  int getSkeletonIndex() const;

  /// \brief
  void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T);

  /// \brief
  void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T);

  /// \brief
  const Eigen::Isometry3d& getTransformFromParentBodyNode() const;

  /// \brief
  const Eigen::Isometry3d& getTransformFromChildBodyNode() const;

  /// \brief Set damping coefficient for viscous force.
  /// \param[in] _idx Index of joint axis.
  /// \param[in] _d Damping coefficient.
  void setDampingCoefficient(int _idx, double _d);

  /// \brief Get damping coefficient for viscous force.
  /// \param[in] _idx Index of joint axis.
  double getDampingCoefficient(int _idx) const;

  /// \brief Get damping force.
  ///
  /// We apply the damping force in implicit manner. The damping force is
  /// F = -(dampingCoefficient * dq(k+1)), where dq(k+1) is approximated as
  /// dq(k) + h * ddq(k). Since, in the recursive forward dynamics algorithm,
  /// ddq(k) is unknown variable that we want to obtain as the result, the
  /// damping force here is just F = -(dampingCoefficient * dq(k)) and
  /// -dampingCoefficient * h * ddq(k) term is rearranged at the recursive
  /// forward dynamics algorithm, and it affects on the articulated inertia.
  /// \sa BodyNode::updateArticulatedInertia(double).
  Eigen::VectorXd getDampingForces() const;

  /// \brief Set spring stiffness for spring force.
  /// \param[in] _idx Index of joint axis.
  /// \param[in] _k Spring stiffness.
  void setSpringStiffness(int _idx, double _k);

  /// \brief Get spring stiffnes for spring force.
  /// \param[in] _idx Index of joint axis.
  double getSpringStiffness(int _idx) const;

  /// \brief Set rest position for spring force.
  /// \param[in] _idx Index of joint axis.
  /// \param[in] _q0 Rest position.
  void setRestPosition(int _idx, double _q0);

  /// \brief Get rest position for spring force.
  /// \param[in] _idx Index of joint axis.
  /// \return Rest position.
  double getRestPosition(int _idx) const;

  /// \brief Get spring force.
  ///
  /// We apply spring force in implicit manner. The spring force is
  /// F = -(springStiffness * q(k+1)), where q(k+1) is approximated as
  /// q(k) + h * dq(k) * h^2 * ddq(k). Since, in the recursive forward dynamics
  /// algorithm, ddq(k) is unknown variable that we want to obtain as the
  /// result, the spring force here is just
  /// F = -springStiffness * (q(k) + h * dq(k)) and
  /// -springStiffness * h^2 * ddq(k) term is rearranged at the recursive
  /// forward dynamics algorithm, and it affects on the articulated inertia.
  /// \sa BodyNode::updateArticulatedInertia(double).
  ///
  /// \param[in] _timeStep Time step used for approximating q(k+1).
  Eigen::VectorXd getSpringForces(double _timeStep) const;

  /// \brief Get potential energy.
  double getPotentialEnergy() const;

  /// \brief
  void applyGLTransform(renderer::RenderInterface* _ri);

protected:
  /// \brief
  /// q --> T(q)
  virtual void updateTransform() = 0;

  /// @brief TODO(JS): This is workaround for Issue #122.
  virtual void updateTransform_Issue122(double _timeStep) {}

  /// \brief
  /// q, dq --> S(q), V(q, dq)
  /// V(q, dq) = S(q) * dq
  virtual void updateJacobian() = 0;

  /// @brief TODO(JS): This is workaround for Issue #122.
  virtual void updateJacobian_Issue122() {}

  /// \brief
  /// dq, ddq, S(q) --> dS(q), dV(q, dq, ddq)
  /// dV(q, dq, ddq) = dS(q) * dq + S(q) * ddq
  virtual void updateJacobianTimeDeriv() = 0;

  /// @brief TODO(JS): This is workaround for Issue #122.
  virtual void updateJacobianTimeDeriv_Issue122() {}

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  /// \brief
  std::string mName;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------
  /// \brief Unique dof id in skeleton
  int mSkelIndex;

  /// \brief
  Eigen::Isometry3d mT_ParentBodyToJoint;

  /// \brief
  Eigen::Isometry3d mT_ChildBodyToJoint;

  //--------------------------------------------------------------------------
  // Kinematics variables
  //--------------------------------------------------------------------------
  /// \brief Local transformation.
  Eigen::Isometry3d mT;

  /// \brief Local Jacobian.
  math::Jacobian mS;

  /// \brief Time derivative of local Jacobian.
  math::Jacobian mdS;

  //--------------------------------------------------------------------------
  // Dynamics variables
  //--------------------------------------------------------------------------
  /// \brief True if the joint limits are enforced in dynamic simulation.
  bool mIsPositionLimited;

  /// \brief
  std::vector<double> mDampingCoefficient;

  /// \brief
  std::vector<double> mSpringStiffness;

  /// \brief
  std::vector<double> mRestPosition;

private:
  /// \brief Type of joint e.g. ball, hinge etc.
  JointType mJointType;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_JOINT_H_
