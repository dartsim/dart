/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_CONSTRAINT_HUMANLEGJOINTLIMITCONSTRAINT_HPP_
#define DART_CONSTRAINT_HUMANLEGJOINTLIMITCONSTRAINT_HPP_

#include "dart/constraint/ConstraintBase.hpp"
#include "dart/math/MathTypes.hpp"
#include "tiny_dnn/tiny_dnn.h"

namespace dart {

namespace dynamics {
class BodyNode;
class Joint;
} // namespace dynamics

namespace constraint {

DART_COMMON_MAKE_SHARED_WEAK(HumanLegJointLimitConstraint)

/// HumanLegJointLimitConstraint handles joint position limits on human leg,
/// representing range of motion of hip, knee and ankle joints.
class HumanLegJointLimitConstraint : public ConstraintBase
{
public:
  /// Constructor
  explicit HumanLegJointLimitConstraint(
      dynamics::Joint* hipjoint,
      dynamics::Joint* kneejoint,
      dynamics::Joint* anklejoint,
      bool isMirror);

  /// Destructor
  virtual ~HumanLegJointLimitConstraint() = default;

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
  /// leg joints involved
  dynamics::Joint* mHipJoint;
  dynamics::Joint* mKneeJoint;
  dynamics::Joint* mAnkleJoint;

  /// leg body nodes involved
  dynamics::BodyNode* mThighNode;
  dynamics::BodyNode* mLowerLegNode;
  dynamics::BodyNode* mFootNode;

  /// A workaround to have a de facto left-handed euler joint
  /// for right hip, so it could share same limits with left hip.
  /// left-leg set to false, right-leg set to true.
  bool mIsMirror;

  /// the neural network for calculating limits
  tiny_dnn::network<tiny_dnn::sequential> mNet;

  /// Gradient of the neural net function
  Eigen::Vector6d mJacobian;

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

  std::size_t mLifeTime;

  double mViolation;

  double mNegativeVel;

  double mOldX;

  double mUpperBound;

  double mLowerBound;

  bool mActive;

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

#endif // DART_CONSTRAINT_HUMANLEGJOINTLIMITCONSTRAINT_HPP_
