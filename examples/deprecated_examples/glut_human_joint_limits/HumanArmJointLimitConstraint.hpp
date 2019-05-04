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

#ifndef EXAMPLES_HUMANJOINTLIMITS_HUMANLEGJOINTLIMITCONSTRAINT_HPP_
#define EXAMPLES_HUMANJOINTLIMITS_HUMANARMJOINTLIMITCONSTRAINT_HPP_

#include <dart/dart.hpp>
#include <tiny_dnn/tiny_dnn.h>

DART_COMMON_MAKE_SHARED_WEAK(HumanArmJointLimitConstraint)

/// HumanArmJointLimitConstraint handles joint position limits on human arm,
/// representing range of motion of shoulder and elbow joints.
class HumanArmJointLimitConstraint : public dart::constraint::ConstraintBase
{
public:
  /// Constructor
  explicit HumanArmJointLimitConstraint(
      dart::dynamics::Joint* shldjoint,
      dart::dynamics::Joint* elbowjoint,
      bool isMirror);

  /// Destructor
  virtual ~HumanArmJointLimitConstraint() = default;

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

  friend class dart::constraint::ConstraintSolver;
  friend class dart::constraint::ConstrainedGroup;

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(dart::constraint::ConstraintInfo* lcp) override;

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
  dart::dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  bool isActive() const override;

private:
  /// arm joints involved
  dart::dynamics::Joint* mShldJoint;
  dart::dynamics::Joint* mElbowJoint;

  /// arm body nodes involved
  dart::dynamics::BodyNode* mUArmNode;
  dart::dynamics::BodyNode* mLArmNode;

  /// A workaround to have a de facto left-handed euler joint
  /// for right shoulder, so it could share same limits with left shoulder.
  /// left-arm set to false, right-arm set to true.
  bool mIsMirror;

  /// the neural network for calculating limits
  tiny_dnn::network<tiny_dnn::sequential> mNet;

  /// Gradient of the neural net function
  Eigen::Vector4d mJacobian;

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

#endif // EXAMPLES_HUMANJOINTLIMITS_HUMANLEGJOINTLIMITCONSTRAINT_HPP_
