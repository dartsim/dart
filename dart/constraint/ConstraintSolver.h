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

#ifndef KIDO_CONSTRAINT_CONSTRAINTSOVER_H_
#define KIDO_CONSTRAINT_CONSTRAINTSOVER_H_

#include <vector>

#include <Eigen/Dense>

#include "kido/constraint/ConstraintBase.h"
#include "kido/collision/CollisionDetector.h"

namespace kido {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace constraint {

class ConstrainedGroup;
class ConstraintBase;
class ClosedLoopConstraint;
class ContactConstraint;
class SoftContactConstraint;
class JointLimitConstraint;
class ServoMotorConstraint;
class JointCoulombFrictionConstraint;
class JointConstraint;
class LCPSolver;

// TODO:
//   - RootSkeleton concept

/// ConstraintSolver manages constraints and computes constraint impulses
class ConstraintSolver
{
public:
  /// Constructor
  explicit ConstraintSolver(double _timeStep);

  /// Copy constructor
  // TODO: implement copy constructor since this class contains a pointer to
  // allocated memory.
  ConstraintSolver(const ConstraintSolver& _other) = delete;

  /// Destructor
  virtual ~ConstraintSolver();

  /// Add single skeleton
  void addSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// Add mutiple skeletons
  void addSkeletons(const std::vector<dynamics::SkeletonPtr>& _skeletons);

  /// Remove single skeleton
  void removeSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// Remove multiple skeletons
  void removeSkeletons(const std::vector<dynamics::SkeletonPtr>& _skeletons);

  /// Remove all skeletons in this constraint solver
  void removeAllSkeletons();

  /// Add a constraint
  void addConstraint(ConstraintBase* _constraint);

  /// Remove a constraint
  void removeConstraint(ConstraintBase* _constraint);

  /// Remove all constraints
  void removeAllConstraints();

  /// Set time step
  void setTimeStep(double _timeStep);

  /// Get time step
  double getTimeStep() const;

  /// Set collision detector
  void setCollisionDetector(collision::CollisionDetector* _collisionDetector);

  /// Get collision detector
  collision::CollisionDetector* getCollisionDetector() const;

  /// Solve constraint impulses and apply them to the skeletons
  void solve();

private:
  /// Check if the skeleton is contained in this solver
  bool containSkeleton(const dynamics::ConstSkeletonPtr& _skeleton) const;

  /// Add skeleton if the constraint is not contained in this solver
  bool checkAndAddSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// Check if the constraint is contained in this solver
  bool containConstraint(const ConstraintBase* _constraint) const;

  /// Add constraint if the constraint is not contained in this solver
  bool checkAndAddConstraint(ConstraintBase* _constraint);

  /// Update constraints
  void updateConstraints();

  /// Build constrained groupsContact
  void buildConstrainedGroups();

  /// Solve constrained groups
  void solveConstrainedGroups();

  /// Return true if at least one of colliding body is soft body
  bool isSoftContact(const collision::Contact& _contact) const;

  /// Collision detector
  collision::CollisionDetector* mCollisionDetector;

  /// Time step
  double mTimeStep;

  /// LCP solver
  LCPSolver* mLCPSolver;

  /// Skeleton list
  std::vector<dynamics::SkeletonPtr> mSkeletons;

  /// Contact constraints those are automatically created
  std::vector<ContactConstraint*> mContactConstraints;

  /// Soft contact constraints those are automatically created
  std::vector<SoftContactConstraint*> mSoftContactConstraints;

  /// Joint limit constraints those are automatically created
  std::vector<JointLimitConstraint*> mJointLimitConstraints;

  /// Servo motor constraints those are automatically created
  std::vector<ServoMotorConstraint*> mServoMotorConstraints;

  /// Joint Coulomb friction constraints those are automatically created
  std::vector<JointCoulombFrictionConstraint*> mJointCoulombFrictionConstraints;

  /// Constraints that manually added
  std::vector<ConstraintBase*> mManualConstraints;

  /// Active constraints
  std::vector<ConstraintBase*> mActiveConstraints;

  /// Constraint group list
  std::vector<ConstrainedGroup> mConstrainedGroups;
};

}  // namespace constraint
}  // namespace kido

#endif  // KIDO_CONSTRAINT_CONSTRAINTSOVER_H_
