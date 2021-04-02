/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_CONSTRAINT_JOINTLIMITCONSTRAINT_HPP_
#define DART_CONSTRAINT_JOINTLIMITCONSTRAINT_HPP_

#include <Eigen/Dense>

#include "dart/constraint/ConstraintBase.hpp"

namespace dart {

namespace dynamics {
class BodyNode;
class Joint;
} // namespace dynamics

namespace constraint {

/// JointLimitConstraint handles joint position and velocity limits
class JointLimitConstraint : public ConstraintBase
{
public:
  /// Constructor
  explicit JointLimitConstraint(dynamics::Joint* joint);

  /// Destructor
  ~JointLimitConstraint() override = default;

  // Documentation inherited
  const std::string& getType() const override;

  /// Returns constraint type for this class.
  static const std::string& getStaticType();

  //----------------------------------------------------------------------------
  // Property settings
  //----------------------------------------------------------------------------

  /// Set global error reduction parameter
  static void setErrorAllowance(double allowance);

  /// Get global error reduction parameter
  static double getErrorAllowance();

  /// Set global error reduction parameter
  static void setErrorReductionParameter(double erp);

  /// Get global error reduction parameter
  static double getErrorReductionParameter();

  /// Set global error reduction parameter
  static void setMaxErrorReductionVelocity(double erv);

  /// Get global error reduction parameter
  static double getMaxErrorReductionVelocity();

  /// Set global constraint force mixing parameter
  static void setConstraintForceMixing(double cfm);

  /// Get global constraint force mixing parameter
  static double getConstraintForceMixing();

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;
  friend class ConstrainedGroup;

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
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  bool isActive() const override;

private:
  /// The Joint that this constraint is associated with.
  dynamics::Joint* mJoint;

  /// The child BodyNode of the Joint that this constraint is associated with.
  dynamics::BodyNode* mBodyNode;

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

  /// Life time of constraint of each DOF.
  Eigen::Matrix<std::size_t, 6, 1> mLifeTime;

  /// Whether joint value exceeds the position limit.
  Eigen::Matrix<bool, 6, 1> mIsPositionLimitViolated;

  /// Whether joint value exceeds the velocity limit.
  Eigen::Matrix<bool, 6, 1> mIsVelocityLimitViolated;

  /// How much current joint values are exceeded from the limits.
  Eigen::Matrix<double, 6, 1> mViolation;

  /// The desired delta velocity to satisfy the joint limit constraint.
  Eigen::Matrix<double, 6, 1> mNegativeVel;

  /// Constraint impulse of the previous step.
  Eigen::Matrix<double, 6, 1> mOldX;

  /// Upper limit of the constraint impulse.
  Eigen::Matrix<double, 6, 1> mUpperBound;

  /// Lower limit of the constraint impulse.
  Eigen::Matrix<double, 6, 1> mLowerBound;

  /// Global constraint error allowance
  static double mErrorAllowance;

  /// Global constraint error redection parameter in the range of [0, 1]. The
  /// default is 0.01.
  static double mErrorReductionParameter;

  /// Maximum error reduction velocity
  static double mMaxErrorReductionVelocity;

  /// Global constraint force mixing parameter in the range of [1e-9, 1]. The
  /// default is 1e-5
  /// \sa http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
  static double mConstraintForceMixing;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_JOINTLIMITCONSTRAINT_HPP_
