/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/constraint/ConstrainedGroup.hpp>
#include <dart/constraint/ConstraintBase.hpp>
#include <dart/constraint/SmartPointer.hpp>

#include <dart/collision/CollisionDetector.hpp>

#include <dart/common/Deprecated.hpp>

#include <Eigen/Dense>

#include <vector>

namespace dart {

namespace dynamics {
class Skeleton;
class ShapeNodeCollisionObject;
} // namespace dynamics

namespace constraint {

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
  constraint::ConstraintBasePtr getConstraint(std::size_t index);

  /// Returns a constraint by index.
  constraint::ConstConstraintBasePtr getConstraint(std::size_t index) const;

  /// Returns all the constraints added to this ConstraintSolver.
  ///
  /// \deprecated Use eachConstraint() instead
  DART_DEPRECATED(6.13)
  std::vector<constraint::ConstraintBasePtr> getConstraints();

  /// Returns all the constraints added to this ConstraintSolver.
  ///
  /// \deprecated Use eachConstraint() instead
  DART_DEPRECATED(6.13)
  std::vector<constraint::ConstConstraintBasePtr> getConstraints() const;

  /// Iterates all the constraints and invokes the callback function.
  ///
  /// \tparam Func: The callback function type. The function signature should be
  /// equivalent to \c void(const ConstraintBase*) or \c bool(const
  /// ConstraintBase*). If you want to conditionally iterate, use \c bool(const
  /// ConstraintBase*) and return false when to stop iterating.
  ///
  /// \param[in] func: The callback function to be called for each
  /// ConstraintBase.
  template <typename Func>
  void eachConstraint(Func func) const;

  /// Iterates all the constraints and invokes the callback function.
  ///
  /// \tparam Func: The callback function type. The function signature should be
  /// equivalent to \c void(ConstraintBase*) or \c bool(ConstraintBase*). If
  /// you want to conditionally iterate, use \c bool(ConstraintBase*) and
  /// return false when to stop iterating.
  ///
  /// \param[in] func: The callback function to be called for each
  /// ConstraintBase.
  template <typename Func>
  void eachConstraint(Func func);

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
  void setCollisionDetector(collision::CollisionDetector* collisionDetector);

  /// Set collision detector
  void setCollisionDetector(
      const std::shared_ptr<collision::CollisionDetector>& collisionDetector);

  /// Get collision detector
  collision::CollisionDetectorPtr getCollisionDetector();

  /// Get (const) collision detector
  collision::ConstCollisionDetectorPtr getCollisionDetector() const;

  /// Return collision group of collision objects that are added to this
  /// ConstraintSolver
  collision::CollisionGroupPtr getCollisionGroup();

  /// Return (const) collision group of collision objects that are added to this
  /// ConstraintSolver
  collision::ConstCollisionGroupPtr getCollisionGroup() const;

  /// Returns collision option that is used for collision checkings in this
  /// ConstraintSolver to generate contact constraints.
  collision::CollisionOption& getCollisionOption();

  /// Returns collision option that is used for collision checkings in this
  /// ConstraintSolver to generate contact constraints.
  const collision::CollisionOption& getCollisionOption() const;

  /// Return the last collision checking result
  collision::CollisionResult& getLastCollisionResult();

  /// Return the last collision checking result
  const collision::CollisionResult& getLastCollisionResult() const;

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

  /// Get the handler used for computing contact surface parameters based on
  /// the contact properties of the two colliding bodies.
  ContactSurfaceHandlerPtr getLastContactSurfaceHandler() const;

  /// Set the handler used for computing contact surface parameters based on
  /// the contact properties of the two colliding bodies. This function
  /// automatically sets the previous handler as parent of the given handler.
  void addContactSurfaceHandler(ContactSurfaceHandlerPtr handler);

  /// Remove the given contact surface handler. If it is not the last in the
  /// chain of handlers, the neighbor handlers are automatically connected
  /// when the given handler is removed. This function returns true when the
  /// given handler was found. It returns false when the handler is not found.
  /// The search procedure utilizes pointer equality (i.e. the shared pointers
  /// have to point to the same address to be treated equal). Take special care
  /// to make sure at least one handler is always available.
  bool removeContactSurfaceHandler(const ContactSurfaceHandlerPtr& handler);

protected:
  // TODO(JS): Docstring
  virtual void solveConstrainedGroup(ConstrainedGroup& group) = 0;

  /// Checks if the skeleton is contained in this solver
  ///
  /// \deprecated Use hasSkeleton() instead.
  DART_DEPRECATED(6.13)
  bool containSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const;

  /// Checks if the skeleton is contained in this solver
  bool hasSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const;

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
  bool isSoftContact(const collision::Contact& contact) const;

  using CollisionDetector = collision::CollisionDetector;

  /// Collision detector
  collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision group
  collision::CollisionGroupPtr mCollisionGroup;

  /// Collision detection option
  collision::CollisionOption mCollisionOption;

  /// Last collision checking result
  collision::CollisionResult mCollisionResult;

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

  /// Factory for ContactSurfaceParams for each contact
  ContactSurfaceHandlerPtr mContactSurfaceHandler;
};

} // namespace constraint
} // namespace dart

#include <dart/constraint/detail/ConstraintSolver-impl.hpp>

#endif // DART_CONSTRAINT_CONSTRAINTSOVER_HPP_
