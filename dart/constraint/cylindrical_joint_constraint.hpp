/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_CONSTRAINT_CYLINDRICALJOINTCONSTRAINT_HPP_
#define DART_CONSTRAINT_CYLINDRICALJOINTCONSTRAINT_HPP_

#include <dart/constraint/dynamic_joint_constraint.hpp>

#include <dart/math/math_types.hpp>

#include <dart/export.hpp>

#include <Eigen/Dense>

#include <string_view>

namespace dart {
namespace constraint {

/// CylindricalJointConstraint represents a runtime cylindrical joint constraint
/// between a body and the world or between two bodies. It leaves translation
/// along the specified axis and rotation about that axis unconstrained.
class DART_API CylindricalJointConstraint : public DynamicJointConstraint
{
public:
  /// Constructor that takes one body, an axis point, and an axis in world
  /// frame.
  /// \param[in] body Constrained body.
  /// \param[in] jointPos Point on the cylinder axis expressed in world frame.
  /// \param[in] axis Cylinder axis expressed in world frame.
  CylindricalJointConstraint(
      dynamics::BodyNode* body,
      const Eigen::Vector3d& jointPos,
      const Eigen::Vector3d& axis);

  /// Constructor that takes two bodies, an axis point, and body axes in world
  /// frame.
  /// \param[in] body1 First constrained body.
  /// \param[in] body2 Second constrained body.
  /// \param[in] jointPos Point on the cylinder axis expressed in world frame.
  /// \param[in] axis1 Cylinder axis for body1 expressed in world frame.
  /// \param[in] axis2 Cylinder axis for body2 expressed in world frame.
  CylindricalJointConstraint(
      dynamics::BodyNode* body1,
      dynamics::BodyNode* body2,
      const Eigen::Vector3d& jointPos,
      const Eigen::Vector3d& axis1,
      const Eigen::Vector3d& axis2);

  /// Destructor
  ~CylindricalJointConstraint() override;

  // Documentation inherited
  std::string_view getType() const override;

  /// Returns constraint type for this class.
  static std::string_view getStaticType();

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* lcp) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t index) override;

  // Documentation inherited
  void getVelocityChange(double* delVel, bool withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(double* lambda) override;

  // Documentation inherited
  bool isActive() const override;

  // Documentation inherited
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  void uniteSkeletons() override;

private:
  /// Build an orthonormal basis around the current cylinder axis.
  void updatePerpendicularBasis();

  /// Offset from body frame 1 origin to the axis point, expressed in body 1.
  Eigen::Vector3d mOffset1;

  /// Offset from body frame 2 origin to the axis point, expressed in body 2.
  Eigen::Vector3d mOffset2;

  /// Cylinder axis expressed in body frame 1.
  Eigen::Vector3d mAxis1;

  /// Cylinder axis expressed in body frame 2, or world frame for one-body use.
  Eigen::Vector3d mAxis2;

  /// Cached cylinder axis for body 1 expressed in world frame.
  Eigen::Vector3d mWorldAxis1;

  /// Cached cylinder axis for body 2 or the world expressed in world frame.
  Eigen::Vector3d mWorldAxis2;

  /// Two orthonormal vectors perpendicular to mWorldAxis1.
  Eigen::Matrix<double, 2, 3> mPerpBasis;

  /// Position/orientation constraint violation (2 translational + 2 angular).
  Eigen::Matrix<double, 4, 1> mViolation;

  /// Linear map between constraint space and Cartesian space for body1.
  Eigen::Matrix<double, 4, 6> mJacobian1;

  /// Linear map between constraint space and Cartesian space for body2.
  Eigen::Matrix<double, 4, 6> mJacobian2;

  ///
  double mOldX[4];

  /// Index of applied impulse.
  std::size_t mAppliedImpulseIndex;

public:
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_CYLINDRICALJOINTCONSTRAINT_HPP_
