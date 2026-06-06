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

#include <dart/constraint/DynamicJointConstraint.hpp>

#include <dart/math/MathTypes.hpp>

#include <Eigen/Dense>

namespace dart {
namespace constraint {

/// CylindricalJointConstraint represents a runtime cylindrical joint constraint
/// between a body and the world or between two bodies. It leaves translation
/// along the specified axis and rotation about that axis unconstrained.
class CylindricalJointConstraint : public DynamicJointConstraint
{
public:
  /// Constructor that takes one body, an axis point, and an axis in world
  /// frame.
  /// \param[in] _body Constrained body.
  /// \param[in] _jointPos Point on the cylinder axis expressed in world frame.
  /// \param[in] _axis Cylinder axis expressed in world frame.
  CylindricalJointConstraint(
      dynamics::BodyNode* _body,
      const Eigen::Vector3d& _jointPos,
      const Eigen::Vector3d& _axis);

  /// Constructor that takes two bodies, an axis point, and body axes in world
  /// frame.
  /// \param[in] _body1 First constrained body.
  /// \param[in] _body2 Second constrained body.
  /// \param[in] _jointPos Point on the cylinder axis expressed in world frame.
  /// \param[in] _axis1 Cylinder axis for _body1 expressed in world frame.
  /// \param[in] _axis2 Cylinder axis for _body2 expressed in world frame.
  CylindricalJointConstraint(
      dynamics::BodyNode* _body1,
      dynamics::BodyNode* _body2,
      const Eigen::Vector3d& _jointPos,
      const Eigen::Vector3d& _axis1,
      const Eigen::Vector3d& _axis2);

  /// Destructor
  virtual ~CylindricalJointConstraint();

  // Documentation inherited
  const std::string& getType() const override;

  /// Returns constraint type for this class.
  static const std::string& getStaticType();

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* _lcp) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t _index) override;

  // Documentation inherited
  void getVelocityChange(double* _vel, bool _withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(double* _lambda) override;

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
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_CYLINDRICALJOINTCONSTRAINT_HPP_
