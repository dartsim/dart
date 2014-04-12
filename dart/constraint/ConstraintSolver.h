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
/// \brief Constraint solver
///
/// Procedure:
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
  /// \brief Constructor
  ConstraintSolver(const std::vector<dynamics::Skeleton*>& _skeletons,
      double _timeStep,
      bool   _useODE                = true);

  /// \brief Destructor
  virtual ~ConstraintSolver();

  //----------------------------------------------------------------------------
  /// \brief Add single skeleton
  void addSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Add skeletons
  void addSkeletons(const std::vector<dynamics::Skeleton*>& _skeletons);

  /// \brief Remove single skeleton.
  void removeSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Remove skeletons.
  void removeSkeletons(const std::vector<dynamics::Skeleton*>& _skeletons);

  /// \brief Remove all skeletons.
  void removeAllSkeletons();

  //----------------------------------------------------------------------------
  /// \brief Add single constraint.
  void addConstraint(Constraint* _constraint);

  /// \brief Add constraints.
  void addConstraints(const std::vector<Constraint*>& _constraints);

  /// \brief Remove single constraint.
  void removeConstraint(Constraint* _constraint);

  /// \brief Remove constraints.
  void removeConstraints(const std::vector<Constraint*>& _constraints);

  /// \brief Remove all constraints.
  void removeAllConstraints();

  //----------------------------------------------------------------------------
  /// \brief Set timestep
  void setTimeStep(double _timeStep);

  /// \brief Get timestep
  double getTimeStep() const;

  //----------------------------------------------------------------------------
  /// \brief Set collision detector
  void setCollisionDetector(collision::CollisionDetector* _collisionDetector);

  /// \brief Get collision detector
  collision::CollisionDetector* getCollisionDetector() const;

  //----------------------------------------------------------------------------
  // Solving
  //----------------------------------------------------------------------------
  /// \brief Solve constraint impulses and apply them to the skeletons
  virtual void solve();

protected:
  //----------------------------------------------------------------------------
  /// \brief
  std::vector<dynamics::Skeleton*> mSkeletons;

  //----------------------------------------------------------------------------
//  /// \brief
//  std::vector<ConstraintTEST*> mBakedConstraints;

  /// \brief
  std::vector<ContactConstraint*> mBakedContactConstraints;

  /// \brief
  std::vector<JointLimitConstraint*> mBakedJointLimitConstraints;

  /// \brief
  std::vector<ClosedLoopConstraint*> mBakedClosedLoopConstraints;

  /// \brief
  std::vector<JointConstraint*> mBakedJointConstraints;

  //----------------------------------------------------------------------------
  /// \brief Constraint list that should be satisfied regardless of skeleton's
  /// state such as dynamic joint constraints
  std::vector<Constraint*> mStaticConstraints;

  /// \brief Constraint list that should be satisfied depend on skeleton's state
  /// such as contact constraint and joint limit constraint.
  std::vector<Constraint*> mDynamicConstraints;

  //----------------------------------------------------------------------------
  /// \brief Constraint group list
  std::vector<ConstrainedGroup> mConstrainedGroups;

private:
  /// \brief
  void init();

  /// \brief
  void bakeConstraints();
  void bakeContactConstraints();
  void bakeJointLimitConstraints();
  void bakeClosedLoopConstraints();
  void bakeJointConstraints();

  /// \brief Check if the skeleton is contained in this solver
  bool containSkeleton(const dynamics::Skeleton* _skeleton) const;

  /// \brief Add skeleton if the constraint is not contained in this solver
  bool checkAndAddSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Check if the constraint is contained in this solver
  bool containConstraint(const Constraint* _constraint) const;

  /// \brief Add constraint if the constraint is not contained in this solver
  bool checkAndAddConstraint(Constraint* _constraint);

  /// \brief Update static constraints
  void updateStaticConstraints();

  /// \brief Update dynamic constraints
  void updateDynamicConstraints();

  /// \brief Build constrained groups
  void buildConstrainedGroups();

  /// \brief Solve constrained groups
  void solveConstrainedGroups();

  /// \brief Collision detector
  collision::CollisionDetector* mCollisionDetector;

  /// \brief Time step
  double mTimeStep;

  /// \brief Flag for using ODE
  bool mUseODE;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINTSOVER_H_
