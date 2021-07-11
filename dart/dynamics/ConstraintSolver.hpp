/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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
#include "dart/dynamics/CollisionDetector.hpp"
#include "dart/dynamics/ConstrainedGroup.hpp"
#include "dart/dynamics/ConstraintBase.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
class ShapeNodeCollisionObject;
} // namespace dynamics

namespace dynamics {

/// ConstraintSolver manages constraints and computes constraint impulses
class ConstraintSolver
{
public:
  /// Constructor
  ///
  /// \deprecated Deprecated in DART 6.8. Please use other constructors that
  /// doesn't take timespte. Timestep should be set by the owner of this solver
  /// such as dart::simulation::World when the solver added.
  DART_DEPRECATED(6.8)
  explicit ConstraintSolver(double timeStep);

  // TODO(JS): Remove timeStep. The timestep can be set by world when a
  // constraint solver is assigned to a world.
  // Deprecate

  /// Default constructor
  ConstraintSolver();

  /// Copy constructor
  // TODO: implement copy constructor since this class contains a pointer to
  // allocated memory.
  ConstraintSolver(const ConstraintSolver& other) = delete;

  /// Destructor
  virtual ~ConstraintSolver() = default;

  /// Add single skeleton
  void addSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Add mutiple skeletons
  void addSkeletons(const std::vector<dynamics::SkeletonPtr>& skeletons);

  /// Returns all the skeletons added to this ConstraintSolver.
  const std::vector<dynamics::SkeletonPtr>& getSkeletons() const;

  /// Remove single skeleton
  void removeSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Remove multiple skeletons
  void removeSkeletons(const std::vector<dynamics::SkeletonPtr>& skeletons);

  /// Remove all skeletons in this constraint solver
  void removeAllSkeletons();

  /// Add a constraint
  void addConstraint(const ConstraintBasePtr& constraint);

  /// Remove a constraint
  void removeConstraint(const ConstraintBasePtr& constraint);

  /// Remove all constraints
  void removeAllConstraints();

  /// Returns the number of constraints that was manually added to this
  /// ConstraintSolver.
  std::size_t getNumConstraints() const;

  /// Returns a constraint by index.
  dynamics::ConstraintBasePtr getConstraint(std::size_t index);

  /// Returns a constraint by index.
  dynamics::ConstConstraintBasePtr getConstraint(std::size_t index) const;

  /// Returns all the constraints added to this ConstraintSolver.
  std::vector<dynamics::ConstraintBasePtr> getConstraints();

  /// Returns all the constraints added to this ConstraintSolver.
  std::vector<dynamics::ConstConstraintBasePtr> getConstraints() const;

  /// Clears the last collision result
  void clearLastCollisionResult();

  /// Set time step
  virtual void setTimeStep(double _timeStep);

  /// Get time step
  double getTimeStep() const;

  /// Set collision detector. This function acquires ownership of the
  /// CollisionDetector passed as an argument. This method is deprecated in
  /// favor of the overload that accepts a std::shared_ptr.
  DART_DEPRECATED(6.0)
  void setCollisionDetector(dynamics::CollisionDetector* collisionDetector);

  /// Set collision detector
  void setCollisionDetector(
      const std::shared_ptr<dynamics::CollisionDetector>& collisionDetector);

  /// Get collision detector
  dynamics::CollisionDetectorPtr getCollisionDetector();

  /// Get (const) collision detector
  dynamics::ConstCollisionDetectorPtr getCollisionDetector() const;

  /// Return collision group of collision objects that are added to this
  /// ConstraintSolver
  dynamics::CollisionGroupPtr getCollisionGroup();

  /// Return (const) collision group of collision objects that are added to this
  /// ConstraintSolver
  dynamics::ConstCollisionGroupPtr getCollisionGroup() const;

  /// Returns collision option that is used for collision checkings in this
  /// ConstraintSolver to generate contact constraints.
  dynamics::CollisionOption& getCollisionOption();

  /// Returns collision option that is used for collision checkings in this
  /// ConstraintSolver to generate contact constraints.
  const dynamics::CollisionOption& getCollisionOption() const;

  /// Return the last collision checking result
  dynamics::CollisionResult& getLastCollisionResult();

  /// Return the last collision checking result
  const dynamics::CollisionResult& getLastCollisionResult() const;

  /// Set LCP solver
  DART_DEPRECATED(6.7)
  void setLCPSolver(std::unique_ptr<LCPSolver> lcpSolver);

  /// Get LCP solver
  DART_DEPRECATED(6.7)
  LCPSolver* getLCPSolver() const;

  /// Solve constraint impulses and apply them to the skeletons
  void solve();

  /// Sets this constraint solver using other constraint solver. All the
  /// properties and registered skeletons and constraints will be copied over.
  virtual void setFromOtherConstraintSolver(const ConstraintSolver& other);

protected:
  // TODO(JS): Docstring
  virtual void solveConstrainedGroup(ConstrainedGroup& group) = 0;

  /// Check if the skeleton is contained in this solver
  bool containSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const;

  /// Add skeleton if the constraint is not contained in this solver
  bool checkAndAddSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Check if the constraint is contained in this solver
  bool containConstraint(const ConstConstraintBasePtr& constraint) const;

  /// Add constraint if the constraint is not contained in this solver
  bool checkAndAddConstraint(const ConstraintBasePtr& constraint);

  /// Update constraints
  void updateConstraints();

  /// Build constrained groupsContact
  void buildConstrainedGroups();

  /// Solve constrained groups
  void solveConstrainedGroups();

  /// Return true if at least one of colliding body is soft body
  bool isSoftContact(const dynamics::Contact& contact) const;

  using CollisionDetector = dynamics::CollisionDetector;

  /// Collision detector
  dynamics::CollisionDetectorPtr mCollisionDetector;

  /// Collision group
  dynamics::CollisionGroupPtr mCollisionGroup;

  /// Collision detection option
  dynamics::CollisionOption mCollisionOption;

  /// Last collision checking result
  dynamics::CollisionResult mCollisionResult;

  /// Time step
  double mTimeStep;

  /// Skeleton list
  std::vector<dynamics::SkeletonPtr> mSkeletons;

  /// Contact constraints those are automatically created
  std::vector<ContactConstraintPtr> mContactConstraints;

  /// Soft contact constraints those are automatically created
  std::vector<SoftContactConstraintPtr> mSoftContactConstraints;

  /// Joint limit constraints those are automatically created
  std::vector<JointConstraintPtr> mJointConstraints;

  /// Mimic motor constraints those are automatically created
  std::vector<MimicMotorConstraintPtr> mMimicMotorConstraints;

  /// Joint Coulomb friction constraints those are automatically created
  std::vector<JointCoulombFrictionConstraintPtr>
      mJointCoulombFrictionConstraints;

  /// Constraints that manually added
  std::vector<ConstraintBasePtr> mManualConstraints;

  /// Active constraints
  std::vector<ConstraintBasePtr> mActiveConstraints;

  /// Constraint group list
  std::vector<ConstrainedGroup> mConstrainedGroups;
};

} // namespace dynamics
} // namespace dart

#endif // DART_CONSTRAINT_CONSTRAINTSOVER_HPP_
