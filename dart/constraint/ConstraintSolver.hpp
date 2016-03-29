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

#ifndef DART_CONSTRAINT_CONSTRAINTSOVER_HPP_
#define DART_CONSTRAINT_CONSTRAINTSOVER_HPP_

#include <vector>

#include <Eigen/Dense>

#include "dart/common/Deprecated.hpp"
#include "dart/constraint/SmartPointer.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/collision/CollisionDetector.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
class ShapeNodeCollisionObject;
}  // namespace dynamics

namespace constraint {

// TODO:
//   - RootSkeleton concept

/// ConstraintSolver manages constraints and computes constraint impulses
class ConstraintSolver
{
public:
  /// Constructor
  explicit ConstraintSolver(double timeStep);

  /// Copy constructor
  // TODO: implement copy constructor since this class contains a pointer to
  // allocated memory.
  ConstraintSolver(const ConstraintSolver& _other) = delete;

  /// Destructor
  virtual ~ConstraintSolver();

  /// Add single skeleton
  void addSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Add mutiple skeletons
  void addSkeletons(const std::vector<dynamics::SkeletonPtr>& skeletons);

  /// Remove single skeleton
  void removeSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Remove multiple skeletons
  void removeSkeletons(const std::vector<dynamics::SkeletonPtr>& skeletons);

  /// Remove all skeletons in this constraint solver
  void removeAllSkeletons();

  /// Add a constraint
  void addConstraint(const ConstraintBasePtr& _constraint);

  /// Remove a constraint
  void removeConstraint(const ConstraintBasePtr& _constraint);

  /// Remove all constraints
  void removeAllConstraints();

  /// Set time step
  void setTimeStep(double _timeStep);

  /// Get time step
  double getTimeStep() const;

  /// Set collision detector. This function acquires ownership of the
  /// CollisionDetector passed as an argument. This method is deprecated in
  /// favor of the overload that accepts a std::shared_ptr.
  DEPRECATED(7.1)
  void setCollisionDetector(collision::CollisionDetector* collisionDetector);

  /// Set collision detector
  void setCollisionDetector(
      const std::shared_ptr<collision::CollisionDetector>& collisionDetector);

  /// Get collision detector
  collision::CollisionDetector* getCollisionDetector();

  /// Get collision detector
  const collision::CollisionDetector* getCollisionDetector() const;

  /// Return collision group of collision objects that are added to this
  /// ConstraintSolver
  collision::CollisionGroup* getCollisionGroup();

  /// Return (const) collision group of collision objects that are added to this
  /// ConstraintSolver
  const collision::CollisionGroup* getCollisionGroup() const;

  /// Return the last collision checking result
  collision::Result& getLastCollisionResult();

  /// Return the last collision checking result
  const collision::Result& getLastCollisionResult() const;

  /// Set LCP solver
  void setLCPSolver(std::unique_ptr<LCPSolver> _lcpSolver);

  /// Get LCP solver
  LCPSolver* getLCPSolver() const;

  /// Solve constraint impulses and apply them to the skeletons
  void solve();

private:
  /// Check if the skeleton is contained in this solver
  bool containSkeleton(const dynamics::ConstSkeletonPtr& _skeleton) const;

  /// Add skeleton if the constraint is not contained in this solver
  bool checkAndAddSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// Check if the constraint is contained in this solver
  bool containConstraint(const ConstConstraintBasePtr& _constraint) const;

  /// Add constraint if the constraint is not contained in this solver
  bool checkAndAddConstraint(const ConstraintBasePtr& _constraint);

  /// Update constraints
  void updateConstraints();

  /// Build constrained groupsContact
  void buildConstrainedGroups();

  /// Solve constrained groups
  void solveConstrainedGroups();

  /// Return true if at least one of colliding body is soft body
  bool isSoftContact(const collision::Contact& _contact) const;

  using CollisionDetector = collision::CollisionDetector;

  /// Collision detector
  std::shared_ptr<collision::CollisionDetector> mCollisionDetector;

  /// Collision group
  std::shared_ptr<collision::CollisionGroup> mCollisionGroup;

  /// Last collision checking result
  collision::Option mCollisionOption;

  /// Last collision checking result
  collision::Result mCollisionResult;

  /// Time step
  double mTimeStep;

  /// LCP solver
  std::unique_ptr<LCPSolver> mLCPSolver;

  /// Skeleton list
  std::vector<dynamics::SkeletonPtr> mSkeletons;

  /// Contact constraints those are automatically created
  std::vector<ContactConstraintPtr> mContactConstraints;

  /// Soft contact constraints those are automatically created
  std::vector<SoftContactConstraintPtr> mSoftContactConstraints;

  /// Joint limit constraints those are automatically created
  std::vector<JointLimitConstraintPtr> mJointLimitConstraints;

  /// Servo motor constraints those are automatically created
  std::vector<ServoMotorConstraintPtr> mServoMotorConstraints;

  /// Joint Coulomb friction constraints those are automatically created
  std::vector<JointCoulombFrictionConstraintPtr> mJointCoulombFrictionConstraints;

  /// Constraints that manually added
  std::vector<ConstraintBasePtr> mManualConstraints;

  /// Active constraints
  std::vector<ConstraintBasePtr> mActiveConstraints;

  /// Constraint group list
  std::vector<ConstrainedGroup> mConstrainedGroups;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINTSOVER_HPP_
