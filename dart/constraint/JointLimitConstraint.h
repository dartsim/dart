/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#ifndef DART_CONSTRAINT_JOINTLIMITCONSTRAINT_H_
#define DART_CONSTRAINT_JOINTLIMITCONSTRAINT_H_

#include "dart/constraint/Constraint.h"

namespace dart {
namespace dynamics {
class BodyNode;
class Joint;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace constraint {

/// \brief JointLimitConstraint handles joint position or velocity limits
class JointLimitConstraint : public Constraint
{
public:
  /// \brief Default constructor
  explicit JointLimitConstraint(dynamics::Joint* _joint);

  /// \brief Default destructor
  ~JointLimitConstraint();

  // Documentaion inherited
  virtual void preprocess() {}

  // Documentaion inherited
  virtual void update();

  // Documentaion inherited
  virtual void fillLcpOde(ODELcp* _lcp, int _idx);

  // Documentaion inherited
  virtual void applyUnitImpulse(int _localIndex);

  // Documentaion inherited
  virtual void getVelocityChange(double* _delVel, int _idx, bool _withCfm);

  // Documentaion inherited
  virtual void excite();

  // Documentaion inherited
  virtual void unexcite();

  // Documentaion inherited
  virtual void applyConstraintImpulse(double* _lambda, int _idx);

  // Documentation inherited
  virtual dynamics::Skeleton* getRootSkeleton() const;

  /// \brief
  bool isActive();

protected:
  /// \brief
  dynamics::Joint* mJoint;

  /// \brief
  dynamics::BodyNode* mBodyNode;

private:
  /// \brief
  size_t mLifeTime[6];

  /// \brief
  bool mActive[6];

  /// \brief
  double mViolation[6];

  /// \brief
  double mNegativeVel[6];

  /// \brief
  double mOldX[6];

  /// \brief
  double mUpperBound[6];

  /// \brief
  double mLowerBound[6];
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_JOINTLIMITCONSTRAINT_H_

