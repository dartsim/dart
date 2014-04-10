/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

class SkeletonGroup
{
public:
  SkeletonGroup();
  ~SkeletonGroup();

  dynamics::Skeleton* getRootSkeleton() const { return mSkeletons[0]; }

  void appendSkeletonGroup(const SkeletonGroup& _group)
  {
    mSkeletons.insert(mSkeletons.end(),
                      _group.mSkeletons.begin(), _group.mSkeletons.end());
  }

protected:
  std::vector<dynamics::Skeleton*> mSkeletons;
};


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
  std::vector<JointLimitConstraint*> mBakedJointLimitContraints;

  /// \brief
  std::vector<ClosedLoopConstraint*> mBakedClosedLoopConstraints;

  /// \brief
  std::vector<JointConstraint*> mBakedJointConstraints;

  //----------------------------------------------------------------------------
  /// \brief
  std::vector<Constraint*> mStaticConstraints;

  /// \brief
  std::vector<Constraint*> mDynamicConstraints;

  //----------------------------------------------------------------------------
  /// \brief List of communities
  std::vector<ConstrainedGroup*> mConstrainedGroups;

  /// \brief
//  std::vector<ClosedLoopContraint_TEST*> mBakedClosedLoopConstraints;

  std::vector<dynamics::Skeleton*> mSkeletonUnion;

private:
  /// \brief
  void _init();

  /// \brief
  void _bakeConstraints();
  void __bakeContactConstraints();
  void __bakeJointLimitConstraints();
  void __bakeClosedLoopConstraints();
  void __bakeJointConstraints();

  /// \brief Check if the skeleton is contained in this solver
  bool _containSkeleton(const dynamics::Skeleton* _skeleton) const;

  /// \brief Add skeleton if the constraint is not contained in this solver
  bool _checkAndAddSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Check if the constraint is contained in this solver
  bool _containConstraint(const Constraint* _constraint) const;

  /// \brief Add constraint if the constraint is not contained in this solver
  bool _checkAndAddConstraint(Constraint* _constraint);

  /// \brief Update dynamic constraints
  void _updateDynamicConstraints();

  /// \brief Build constrained groups
  void _buildConstrainedGroups();

  /// \brief Solve constrained groups
  void _solveConstrainedGroups();

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
