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

#ifndef DART_CONSTRAINT_REVOLUTEJOINTCONSTRAINT_HPP_
#define DART_CONSTRAINT_REVOLUTEJOINTCONSTRAINT_HPP_

#include <dart/constraint/dynamic_joint_constraint.hpp>

#include <dart/math/math_types.hpp>

#include <dart/export.hpp>

#include <Eigen/Dense>

namespace dart {
namespace constraint {

/// RevoluteJointConstraint enforces a 1-DOF hinge relationship between two
/// bodies (or a body and the world). Three translational constraints keep the
/// anchor positions coincident and two angular constraints align the hinge
/// axes, leaving rotation about the axis unconstrained.
class DART_API RevoluteJointConstraint : public DynamicJointConstraint
{
public:
  /// Constructor that creates a hinge between a body and the world.
  /// \param[in] body The constrained body.
  /// \param[in] jointPos Joint anchor position expressed in world frame.
  /// \param[in] axis Hinge axis expressed in world frame.
  RevoluteJointConstraint(
      dynamics::BodyNode* body,
      const Eigen::Vector3d& jointPos,
      const Eigen::Vector3d& axis);

  /// Constructor that creates a hinge between two bodies.
  /// \param[in] body1 First constrained body.
  /// \param[in] body2 Second constrained body.
  /// \param[in] jointPos Joint anchor position expressed in world frame.
  /// \param[in] axis1 Hinge axis for body1 expressed in world frame.
  /// \param[in] axis2 Hinge axis for body2 expressed in world frame.
  RevoluteJointConstraint(
      dynamics::BodyNode* body1,
      dynamics::BodyNode* body2,
      const Eigen::Vector3d& jointPos,
      const Eigen::Vector3d& axis1,
      const Eigen::Vector3d& axis2);

  /// Destructor
  ~RevoluteJointConstraint() override;

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
  /// Returns true if this constraint is between two different skeletons.
  bool isCollidingTwoDifferentSkeletons() const;

  /// Build an orthonormal basis around the current hinge axis.
  void updateAngularBasis();

  /// Offsets from the origins of body frames to the joint position expressed
  /// in each body frame.
  Eigen::Vector3d mOffset1;
  Eigen::Vector3d mOffset2;

  /// Hinge axes expressed in each body frame (axis2 is unused when constraining
  /// against the world).
  Eigen::Vector3d mAxis1;
  Eigen::Vector3d mAxis2;

  /// Cached world-frame hinge axes (axis2 is the reference axis when
  /// constraining against the world).
  Eigen::Vector3d mWorldAxis1;
  Eigen::Vector3d mWorldAxis2;

  /// Two orthonormal vectors perpendicular to mWorldAxis1.
  Eigen::Matrix<double, 2, 3> mPerpBasis;

  /// Position/orientation constraint violation (3 translational + 2 angular).
  Eigen::Matrix<double, 5, 1> mViolation;

  /// Jacobians mapping body spatial velocity to constraint velocity.
  Eigen::Matrix<double, 5, 6> mJacobian1;
  Eigen::Matrix<double, 5, 6> mJacobian2;

  ///
  double mOldX[5];

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_REVOLUTEJOINTCONSTRAINT_HPP_
