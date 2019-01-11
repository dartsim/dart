/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_CONSTRAINT_MIMICMOTORCONSTRAINT_HPP_
#define DART_CONSTRAINT_MIMICMOTORCONSTRAINT_HPP_

#include "dart/constraint/ConstraintBase.hpp"

namespace dart {

namespace dynamics {
class BodyNode;
class Joint;
} // namespace dynamics

namespace constraint {

/// Servo motor constraint
class MimicMotorConstraint : public ConstraintBase
{
public:
  /// Constructor
  explicit MimicMotorConstraint(
      dynamics::Joint* joint,
      const dynamics::Joint* mimicJoint,
      double multiplier = 1.0,
      double offset = 0.0);

  /// Destructor
  ~MimicMotorConstraint() override;

  //----------------------------------------------------------------------------
  // Property settings
  //----------------------------------------------------------------------------

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
  ///
  dynamics::Joint* mJoint;

  ///
  const dynamics::Joint* mMimicJoint;

  ///
  double mMultiplier, mOffset;

  ///
  dynamics::BodyNode* mBodyNode;

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

  ///
  std::size_t mLifeTime[6];
  // TODO(JS): Lifetime should be considered only when we use iterative lcp
  // solver

  ///
  bool mActive[6];

  ///
  double mNegativeVelocityError[6];

  ///
  double mOldX[6];

  ///
  double mUpperBound[6];

  ///
  double mLowerBound[6];

  /// Global constraint force mixing parameter in the range of [1e-9, 1]. The
  /// default is 1e-5
  /// \sa http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
  static double mConstraintForceMixing;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_MIMICMOTORCONSTRAINT_HPP_
