/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *            C. Karen Liu <karenliu@cc.gatech.edu>
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
  explicit BallJointConstraint(dynamics::BodyNode* _body, 
                               Eigen::Vector3d _jointPos);


  /// Constructor that takes two bodies and the joint position in the frame of _body1
  BallJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode* _body2,
                      Eigen::Vector3d _jointPos);


  /// Destructor
  virtual ~BallJointConstraint();

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
  virtual dynamics::Skeleton* getRootSkeleton() const;

  // Documentation inherited
  virtual void uniteSkeletons();

private:
  ///
  Eigen::Vector3d mOffset1;

  ///
  Eigen::Vector3d mOffset2;

  ///
  Eigen::Vector3d mViolation;

  ///
  Eigen::MatrixXd mJacobian1;

  ///
  Eigen::MatrixXd mJacobian2;

  ///
  double mOldX[3];

  /// Index of applied impulse
  size_t mAppliedImpulseIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_WELDJOINTCONSTRAINT_H_

  


/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_CONSTRAINT_BALLJOINTCONSTRAINT_H_
#define DART_CONSTRAINT_BALLJOINTCONSTRAINT_H_

#include "dart/constraint/JointConstraint.h"

#include <Eigen/Dense>

namespace dart {
namespace constraint {

class BallJointConstraint : public JointConstraint
{
public:
  /// Constructor
  /// \param[in] _body
  /// \param[in] _offset Offset from _body's origin to ball joint position
  ///                    expressed in _body's frame
  BallJointConstraint(dynamics::BodyNode *_body,
                      const Eigen::Vector3d& _offset);

  /// Constructor
  /// \param[in] _body1
  /// \param[in] _body2
  /// \param[in] _offset1 Offset from _body1's origin to ball joint position
  ///                     expressed in _body1's frame
  /// \param[in] _offset2 Offset from _body2's origin to ball joint position
  ///                     expressed in _body2's frame
  BallJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2,
                      const Eigen::Vector3d& _offset1,
                      const Eigen::Vector3d& _offset2);

  /// Destructor
  ~BallJointConstraint();

protected:
  // Documentation inherited
  virtual void update();

  // Documentation inherited
  virtual void getLCPVectors(ConstraintInfo* _info);

  // Documentation inherited
  virtual void applyUnitImpulse(size_t _index);

  // Documentation inherited
  virtual void getVelocityChange(double* _vel, bool _withCfm);

  // Documentation inherited
  virtual void excite();

  // Documentation inherited
  virtual void unexcite();

  // Documentation inherited
  virtual void applyConstraintImpulse(double* _lambda);

  // Documentation inherited
  virtual dynamics::Skeleton* getRootSkeleton() const {return NULL;}

protected:
  ///
  Eigen::Vector3d mOffset1;

  ///
  Eigen::Vector3d mOffset2;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_BALLJOINTCONSTRAINT_H_

