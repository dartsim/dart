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

#ifndef DART_CONSTRAINT_CONSTRAINTSOVER_H_
#define DART_CONSTRAINT_CONSTRAINTSOVER_H_

#include <vector>

#include <Eigen/Dense>

#include "dart/constraint/Constraint.h"
#include "dart/collision/CollisionDetector.h"

namespace dart {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace constraint {

class ConstrainedGroup;
class Constraint;
class BallJointContraintTEST;
class ClosedLoopConstraint;
class ContactConstraint;
class JointLimitConstraint;
class RevoluteJointContraintTEST;
class WeldJointContraintTEST;
class JointConstraint;

//==============================================================================
/// ConstraintSolver manages all the constraints and solves constraint impulses
/// for skeletons
///
/// -# Warm start
///   -# Build solid skeleton groups using solid constraints
///   -# Build constraint groups using solid skeleton groups
///
/// -# Solving
///   -# Update or create blink constraints with new skeleton states
///   -# Merge solid skeleton groups using blink constraints
///   -# Merge constraint groups using merged skeleton groups
//==============================================================================
class ConstraintSolver
{
public:
  /// Constructor
  ConstraintSolver(const std::vector<dynamics::Skeleton*>& _skeletons,
      double _timeStep, bool   _useODE = true);

  /// Destructor
  virtual ~ConstraintSolver();

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  /// Add single skeleton
  void addSkeleton(dynamics::Skeleton* _skeleton);

  /// Add mutiple skeletons
  void addSkeletons(const std::vector<dynamics::Skeleton*>& _skeletons);

  /// Remove single skeleton
  void removeSkeleton(dynamics::Skeleton* _skeleton);

  /// Remove multiple skeletons
  void removeSkeletons(const std::vector<dynamics::Skeleton*>& _skeletons);

  /// Remove all skeletons in this constraint solver
  void removeAllSkeletons();

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  /// Add single constraint.
  void addConstraint(Constraint* _constraint);

  /// Add constraints.
  void addConstraints(const std::vector<Constraint*>& _constraints);

  ///
  size_t getNumConstraints() const;

  ///
  Constraint* getConstraint(size_t _index);

  /// Remove single constraint.
  void removeConstraint(Constraint* _constraint);

  /// Remove constraints.
  void removeConstraints(const std::vector<Constraint*>& _constraints);

  /// Remove all constraints.
  void removeAllConstraints();

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  /// Set timestep
  void setTimeStep(double _timeStep);

  /// Get timestep
  double getTimeStep() const;

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  /// Set collision detector
  void setCollisionDetector(collision::CollisionDetector* _collisionDetector);

  /// Get collision detector
  collision::CollisionDetector* getCollisionDetector() const;

  //----------------------------------------------------------------------------
  // Solving
  //----------------------------------------------------------------------------

  /// Solve constraint impulses and apply them to the skeletons
  virtual void solve();

protected:
  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  ///
  std::vector<dynamics::Skeleton*> mSkeletons;

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

//  ///
//  std::vector<ConstraintTEST*> mBakedConstraints;

  ///
  std::vector<ContactConstraint*> mBakedContactConstraints;

  ///
  std::vector<JointLimitConstraint*> mBakedJointLimitConstraints;

  ///
  std::vector<ClosedLoopConstraint*> mBakedClosedLoopConstraints;

  ///
  std::vector<JointConstraint*> mBakedJointConstraints;

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  /// Constraint list that should be satisfied regardless of skeleton's state
  /// such as dynamic joint constraints
  std::vector<Constraint*> mStaticConstraints;

  /// Constraint list that should be satisfied depend on skeleton's state such
  /// as contact constraint and joint limit constraint.
  std::vector<Constraint*> mDynamicConstraints;

  //----------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------

  /// Constraint group list
  std::vector<ConstrainedGroup> mConstrainedGroups;

private:
  ///
  void init();

  ///
  void bakeConstraints();

  ///
  void bakeContactConstraints();

  ///
  void bakeJointLimitConstraints();

  ///
  void bakeClosedLoopConstraints();

  ///
  void bakeJointConstraints();

  /// Check if the skeleton is contained in this solver
  bool containSkeleton(const dynamics::Skeleton* _skeleton) const;

  /// Add skeleton if the constraint is not contained in this solver
  bool checkAndAddSkeleton(dynamics::Skeleton* _skeleton);

  /// Check if the constraint is contained in this solver
  bool containConstraint(const Constraint* _constraint) const;

  /// Add constraint if the constraint is not contained in this solver
  bool checkAndAddConstraint(Constraint* _constraint);

  /// Update static constraints
  void updateStaticConstraints();

  /// Update dynamic constraints
  void updateDynamicConstraints();

  /// Build constrained groups
  void buildConstrainedGroups();

  /// Solve constrained groups
  void solveConstrainedGroups();

  /// Collision detector
  collision::CollisionDetector* mCollisionDetector;

  /// Time step
  double mTimeStep;

  /// Flag for using ODE
  bool mUseODE;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINTSOVER_H_
