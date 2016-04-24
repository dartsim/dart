/*
 * Copyright (c) 2014-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *            C. Karen Liu <karenliu@cc.gatech.edu>
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

#ifndef DART_CONSTRAINT_BALLJOINTCONSTRAINT_H_
#define DART_CONSTRAINT_BALLJOINTCONSTRAINT_H_

#include <Eigen/Dense>

#include "dart/math/MathTypes.h"
#include "dart/constraint/JointConstraint.h"

namespace dart {
namespace constraint {

/// BallJointConstraint represents ball joint constraint between a body and the
/// world or between two bodies
class BallJointConstraint : public JointConstraint
{
public:
  /// Constructor that takes one body and the joint position in the world frame
  /// \param[in] _jointPos Joint position expressed in world frame
  BallJointConstraint(dynamics::BodyNode* _body,
                      const Eigen::Vector3d& _jointPos);


  /// Constructor that takes two bodies and the joint position in the frame of
  /// _body1
  /// \param[in] _jointPos Joint position expressed in world frame
  BallJointConstraint(dynamics::BodyNode* _body1, dynamics::BodyNode* _body2,
                      const Eigen::Vector3d& _jointPos);

  /// Destructor
  virtual ~BallJointConstraint();

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
  /// Offset from the origin of body frame 1 to the ball joint position where
  /// the offset is expressed in body frame 1
  Eigen::Vector3d mOffset1;

  /// Offset from the origin of body frame 2 to the ball joint position where
  /// the offset is expressed in body frame 2
  Eigen::Vector3d mOffset2;

  /// Position constraint violation expressed in body frame 1
  Eigen::Vector3d mViolation;

  /// Linear map between constraint space and Cartesian space for body1
  Eigen::Matrix<double, 3, 6> mJacobian1;

  /// Linear map between constraint space and Cartesian space for body2
  Eigen::Matrix<double, 3, 6> mJacobian2;

  ///
  double mOldX[3];

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_BALLJOINTCONSTRAINT_H_

