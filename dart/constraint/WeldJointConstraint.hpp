/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_CONSTRAINT_WELDJOINTCONSTRAINT_HPP_
#define DART_CONSTRAINT_WELDJOINTCONSTRAINT_HPP_

#include <dart/constraint/DynamicJointConstraint.hpp>

#include <dart/math/MathTypes.hpp>

#include <Eigen/Dense>

namespace dart {
namespace constraint {

/// WeldJointConstraint represents weld joint constraint between a body and the
/// world or between two bodies
class WeldJointConstraint : public DynamicJointConstraint
{
public:
  /// Constructor that takes one body
  explicit WeldJointConstraint(dynamics::BodyNode* _body);

  /// Constructor that takes two bodies
  WeldJointConstraint(dynamics::BodyNode* _body1, dynamics::BodyNode* _body2);

  /// Set the relative transform that this WeldJointConstraint will enforce
  void setRelativeTransform(const Eigen::Isometry3d& _tf);

  /// Get the relative transform that this WeldJointConstraint will enforce
  const Eigen::Isometry3d& getRelativeTransform() const;

  /// Destructor
  virtual ~WeldJointConstraint();

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
  ///
  Eigen::Isometry3d mRelativeTransform;

  ///
  Eigen::Vector6d mViolation;

  ///
  const Eigen::Matrix6d mJacobian1;

  ///
  Eigen::Matrix6d mJacobian2;

  ///
  double mOldX[6];

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_WELDJOINTCONSTRAINT_HPP_
