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

#ifndef DART_CONSTRAINT_CONSTRAINEDGROUP_HPP_
#define DART_CONSTRAINT_CONSTRAINEDGROUP_HPP_

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "dart/constraint/SmartPointer.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace constraint {

struct ConstraintInfo;
class ConstraintBase;
class ConstraintSolver;

/// ConstrainedGroup is a group of skeletons that interact each other with
/// constraints
/// \sa class ConstraintSolver
class ConstrainedGroup
{
public:
  //----------------------------------------------------------------------------
  // Constructor / Desctructor
  //----------------------------------------------------------------------------

  /// Default contructor
  ConstrainedGroup();

  /// Destructor
  virtual ~ConstrainedGroup();

  //----------------------------------------------------------------------------
  // Setting
  //----------------------------------------------------------------------------

  /// Add constraint
  void addConstraint(const ConstraintBasePtr& _constraint);

  /// Return number of constraints in this constrained group
  std::size_t getNumConstraints() const;

  /// Return a constraint
  ConstraintBasePtr getConstraint(std::size_t _index);

  /// Return a constraint
  ConstConstraintBasePtr getConstraint(std::size_t _index) const;

  /// Remove constraint
  void removeConstraint(const ConstraintBasePtr& _constraint);

  /// Remove all constraints
  void removeAllConstraints();

  /// Get total dimension of contraints in this group
  std::size_t getTotalDimension() const;

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;

private:
#ifndef NDEBUG
  /// Return true if _constraint is contained
  bool containConstraint(const ConstConstraintBasePtr& _constraint) const;
#endif

  /// List of constraints
  std::vector<ConstraintBasePtr> mConstraints;

  ///
  std::shared_ptr<dynamics::Skeleton> mRootSkeleton;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINEDGROUP_HPP_

