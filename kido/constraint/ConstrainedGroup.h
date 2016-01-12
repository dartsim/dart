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

#ifndef KIDO_CONSTRAINT_CONSTRAINEDGROUP_H_
#define KIDO_CONSTRAINT_CONSTRAINEDGROUP_H_

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace kido {

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
  void addConstraint(ConstraintBase* _constraint);

  /// Return number of constraints in this constrained group
  size_t getNumConstraints() const;

  /// Return a constraint
  ConstraintBase* getConstraint(size_t _index) const;

  /// Remove constraint
  void removeConstraint(ConstraintBase* _constraint);

  /// Remove all constraints
  void removeAllConstraints();

  /// Get total dimension of contraints in this group
  size_t getTotalDimension() const;

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;

private:
  /// Return true if _constraint is contained
  bool containConstraint(ConstraintBase* _constraint) const;

  /// Return true and add the constraint if _constraint is contained
  bool checkAndAddConstraint(ConstraintBase* _constraint);

  /// List of constraints
  std::vector<ConstraintBase*> mConstraints;

  ///
  std::shared_ptr<dynamics::Skeleton> mRootSkeleton;
};

}  // namespace constraint
}  // namespace kido

#endif  // KIDO_CONSTRAINT_CONSTRAINEDGROUP_H_

