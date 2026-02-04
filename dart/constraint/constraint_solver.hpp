/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/constraint/constrained_group.hpp>
#include <dart/constraint/constraint_base.hpp>
#include <dart/constraint/fwd.hpp>

#include <dart/collision/collision_detector.hpp>

#include <dart/dynamics/fwd.hpp>

#include <dart/math/lcp/lcp_solver.hpp>

#include <dart/common/deprecated.hpp>
#include <dart/common/frame_allocator.hpp>

#include <dart/export.hpp>

#include <Eigen/Dense>

#include <span>
#include <vector>

namespace dart {
namespace constraint {

/// ConstraintSolver manages constraints and computes constraint impulses
class DART_API ConstraintSolver
{
public:
  // TODO(JS): Remove timeStep. The timestep can be set by world when a
  // constraint solver is assigned to a world.
  // Deprecate

  /// Default constructor
  ConstraintSolver();
  ConstraintSolver(math::LcpSolverPtr primary);
  ConstraintSolver(math::LcpSolverPtr primary, math::LcpSolverPtr secondary);

  /// Copy constructor
  // TODO: implement copy constructor since this class contains a pointer to
  // allocated memory.
  ConstraintSolver(const ConstraintSolver& other) = delete;

  /// Destructor
  virtual ~ConstraintSolver();

  /// Add single skeleton
  void addSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Add multiple skeletons
  void addSkeletons(std::span<const dynamics::SkeletonPtr> skeletons);

  /// Returns all the skeletons added to this ConstraintSolver.
  std::span<const dynamics::SkeletonPtr> getSkeletons() const;

  /// Remove single skeleton
  void removeSkeleton(const dynamics::SkeletonPtr& skeleton);

  /// Remove multiple skeletons
  void removeSkeletons(std::span<const dynamics::SkeletonPtr> skeletons);

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

  /// Iterates all the constraints and invokes the callback function.
  ///
  /// @tparam Func: The callback function type. The function signature should be
  /// equivalent to @c void(const ConstraintBase*) or @c bool(const
  /// ConstraintBase*). If you want to conditionally iterate, use @c bool(const
  /// ConstraintBase*) and return false when to stop iterating.
  ///
  /// @param[in] func: The callback function to be called for each
  /// ConstraintBase.
  template <typename Func>
  void eachConstraint(Func func) const;

  /// Iterates all the constraints and invokes the callback function.
  ///
  /// @tparam Func: The callback function type. The function signature should be
  /// equivalent to @c void(ConstraintBase*) or @c bool(ConstraintBase*). If
  /// you want to conditionally iterate, use @c bool(ConstraintBase*) and
  /// return false when to stop iterating.
  ///
  /// @param[in] func: The callback function to be called for each
  /// ConstraintBase.
  template <typename Func>
  void eachConstraint(Func func);

  /// Clears the last collision result
  void clearLastCollisionResult();

  /// Set time step
  virtual void setTimeStep(double _timeStep);

  /// Get time step
  double getTimeStep() const;

  void setFrameAllocator(common::FrameAllocator* alloc);

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

  /// Set the primary LCP solver (default: math::DantzigSolver)
  void setLcpSolver(math::LcpSolverPtr lcpSolver);

  /// Get the primary LCP solver
  math::LcpSolverPtr getLcpSolver() const;

  /// Set the secondary LCP solver (default: Pgs, nullptr disables)
  void setSecondaryLcpSolver(math::LcpSolverPtr lcpSolver);

  /// Get the secondary LCP solver
  math::LcpSolverPtr getSecondaryLcpSolver() const;

  /// Enable or disable split impulse position correction.
  void setSplitImpulseEnabled(bool enabled);

  /// Get whether split impulse position correction is enabled.
  bool isSplitImpulseEnabled() const;

protected:
  /// Solve constrained group with the selected phase.
  void solveConstrainedGroupInternal(
      std::span<const ConstraintBasePtr> constraints, ConstraintPhase phase);

  /// Solve constrained groups for split impulse position correction.
  void solvePositionConstrainedGroups();

  // TODO(JS): Docstring
  virtual void solveConstrainedGroup(ConstrainedGroup& group);

  /// Return true if the matrix is symmetric
  bool isSymmetric(std::size_t n, double* A);

  /// Return true if the diagonal block of matrix is symmetric
  bool isSymmetric(
      std::size_t n, double* A, std::size_t begin, std::size_t end);

  /// Print debug information
  void print(
      std::size_t n,
      double* A,
      double* x,
      double* lo,
      double* hi,
      double* b,
      double* w,
      int* findex);

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

  /// Coupler constraints that are automatically created
  std::vector<CouplerConstraintPtr> mCouplerConstraints;

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

  /// LCP solver (primary)
  math::LcpSolverPtr mLcpSolver;

  /// True if the primary LCP solver was set explicitly by the caller
  bool mLcpSolverSetExplicitly{false};

  /// LCP solver to use as fallback
  math::LcpSolverPtr mSecondaryLcpSolver;

  /// True if the secondary LCP solver was set explicitly by the caller
  bool mSecondaryLcpSolverSetExplicitly{false};

  /// Enable split impulse position correction.
  bool mSplitImpulseEnabled{false};

  /// Cache data for boxed LCP formulation
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mA;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      mABackup;
  Eigen::VectorXd mX;
  Eigen::VectorXd mXBackup;
  Eigen::VectorXd mB;
  Eigen::VectorXd mBBackup;
  Eigen::VectorXd mW;
  Eigen::VectorXd mLo;
  Eigen::VectorXd mLoBackup;
  Eigen::VectorXd mHi;
  Eigen::VectorXd mHiBackup;
  Eigen::VectorXi mFIndex;
  Eigen::VectorXi mFIndexBackup;
  Eigen::VectorXi mOffset;

  std::unique_ptr<common::FrameAllocator> mOwnedFrameAllocator;
  common::FrameAllocator* mFrameAllocator = nullptr;

  /// Persistent containers (reused across steps to avoid per-step allocation)
  std::vector<collision::Contact*> mContactPtrs;
  std::vector<bool> mImpulseAppliedStates;
  std::vector<ConstraintBasePtr> mPositionConstraints;
};

} // namespace constraint
} // namespace dart

#include <dart/constraint/detail/constraint_solver-impl.hpp>

#endif // DART_CONSTRAINT_CONSTRAINTSOVER_HPP_
