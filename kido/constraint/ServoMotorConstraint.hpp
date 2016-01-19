/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef KIDO_CONSTRAINT_SERVOMOTORCONSTRAINT_HPP_
#define KIDO_CONSTRAINT_SERVOMOTORCONSTRAINT_HPP_

#include "kido/constraint/ConstraintBase.hpp"

namespace kido {

namespace dynamics {
class BodyNode;
class Joint;
}  // namespace dynamics

namespace constraint {

/// Servo motor constraint
class ServoMotorConstraint : public ConstraintBase
{
public:
  /// Constructor
  explicit ServoMotorConstraint(dynamics::Joint* joint);

  /// Destructor
  virtual ~ServoMotorConstraint();

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
  virtual void update();

  // Documentation inherited
  virtual void getInformation(ConstraintInfo* lcp);

  // Documentation inherited
  virtual void applyUnitImpulse(size_t index);

  // Documentation inherited
  virtual void getVelocityChange(double* delVel, bool withCfm);

  // Documentation inherited
  virtual void excite();

  // Documentation inherited
  virtual void unexcite();

  // Documentation inherited
  virtual void applyImpulse(double* lambda);

  // Documentation inherited
  virtual dynamics::SkeletonPtr getRootSkeleton() const;

  // Documentation inherited
  virtual bool isActive() const;

private:
  ///
  dynamics::Joint* mJoint;

  ///
  dynamics::BodyNode* mBodyNode;

  /// Index of applied impulse
  size_t mAppliedImpulseIndex;

  ///
  size_t mLifeTime[6];
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

}  // namespace constraint
}  // namespace kido

#endif  // KIDO_CONSTRAINT_SERVOMOTORCONSTRAINT_HPP_

