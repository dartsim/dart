/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#ifndef KIDO_CONSTRAINT_WELDJOINTCONSTRAINT_H_
#define KIDO_CONSTRAINT_WELDJOINTCONSTRAINT_H_

#include <Eigen/Dense>

#include "kido/math/MathTypes.hpp"
#include "kido/constraint/JointConstraint.hpp"

namespace kido {
namespace constraint {

/// WeldJointConstraint represents weld joint constraint between a body and the
/// world or between two bodies
class WeldJointConstraint : public JointConstraint
{
public:
  /// Constructor that takes one body
  explicit WeldJointConstraint(dynamics::BodyNode* _body);

  /// Constructor that takes two bodies
  WeldJointConstraint(dynamics::BodyNode* _body1, dynamics::BodyNode* _body2);

  /// Destructor
  virtual ~WeldJointConstraint();

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void update();

  // Documentation inherited
  virtual void getInformation(ConstraintInfo* _lcp);

  // Documentation inherited
  virtual void applyUnitImpulse(size_t _index);

  // Documentation inherited
  virtual void getVelocityChange(double* _vel, bool _withCfm);

  // Documentation inherited
  virtual void excite();

  // Documentation inherited
  virtual void unexcite();

  // Documentation inherited
  virtual void applyImpulse(double* _lambda);

  // Documentation inherited
  virtual bool isActive() const;

  // Documentation inherited
  virtual dynamics::SkeletonPtr getRootSkeleton() const;

  // Documentation inherited
  virtual void uniteSkeletons();

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
  size_t mAppliedImpulseIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace constraint
}  // namespace kido

#endif  // KIDO_CONSTRAINT_WELDJOINTCONSTRAINT_H_

