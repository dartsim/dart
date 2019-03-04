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

#ifndef DART_CONSTRAINT_CONSTRAINTBASE_HPP_
#define DART_CONSTRAINT_CONSTRAINTBASE_HPP_

#include <cstddef>

#include "dart/dynamics/SmartPointer.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace constraint {

/// ConstraintInfo
struct ConstraintInfo
{
  /// Impulse
  double* x;

  /// Lower bound of x
  double* lo;

  /// Upper bound of x
  double* hi;

  /// Bias term
  double* b;

  /// Slack variable
  double* w;

  /// Friction index
  int* findex;

  /// Inverse of time step
  double invTimeStep;
};

/// Constraint is a base class of concrete constraints classes
class ConstraintBase
{
public:
  /// Return dimesion of this constranit
  std::size_t getDimension() const;

  /// Update constraint using updated Skeleton's states
  virtual void update() = 0;

  /// Fill LCP variables
  virtual void getInformation(ConstraintInfo* info) = 0;

  /// Apply unit impulse to constraint space
  virtual void applyUnitImpulse(std::size_t index) = 0;

  /// Get velocity change due to the uint impulse
  virtual void getVelocityChange(double* vel, bool withCfm) = 0;

  /// Excite the constraint
  virtual void excite() = 0;

  /// Unexcite the constraint
  virtual void unexcite() = 0;

  /// Apply computed constraint impulse to constrained skeletons
  virtual void applyImpulse(double* lambda) = 0;

  /// Return true if this constraint is active
  virtual bool isActive() const = 0;

  ///
  virtual dynamics::SkeletonPtr getRootSkeleton() const = 0;

  ///
  virtual void uniteSkeletons();

  ///
  static dynamics::SkeletonPtr compressPath(dynamics::SkeletonPtr skeleton);

  ///
  static dynamics::SkeletonPtr getRootSkeleton(dynamics::SkeletonPtr skeleton);

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;
  friend class ConstrainedGroup;

protected:
  /// Default contructor
  ConstraintBase();

  /// Destructor
  virtual ~ConstraintBase();

protected:
  /// Dimension of constraint
  std::size_t mDim;
};

} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINTBASE_HPP_

