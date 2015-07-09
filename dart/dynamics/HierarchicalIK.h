/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_HIERARCHICALIK_H_
#define DART_DYNAMICS_HIERARCHICALIK_H_

#include "dart/dynamics/InverseKinematics.h"

namespace dart {
namespace dynamics {

/// The HierarchicalIK class provides a convenient way of setting up a
/// hierarchical inverse kinematics optimization problem which combines several
/// InverseKinematics problems into one. InverseKinematics problems with a
/// larger hierarchy level will be projected into null spaces of the problems
/// that have a smaller hierarchy number.
class HierarchicalIK : public common::Subject
{
public:

  virtual ~HierarchicalIK();

  const Eigen::VectorXd& solve();

  SkeletonPtr getSkeleton() const;

  virtual std::shared_ptr<HierarchicalIK> clone(const SkeletonPtr& _newSkel) const = 0;

  class Function
  {
  public:
    virtual optimizer::FunctionPtr clone(HierarchicalIK* _newIK) const = 0;

    virtual ~Function() = default;
  };

  void setObjective(std::shared_ptr<optimizer::Function> _objective);

  const std::shared_ptr<optimizer::Function>& getObjective();

  std::shared_ptr<const optimizer::Function> getObjective() const;

  void setNullSpaceObjective(
      const std::shared_ptr<optimizer::Function>& _nsObjective);

  const std::shared_ptr<optimizer::Function>& getNullSpaceObjective();

  std::shared_ptr<const optimizer::Function> getNullSpaceObjective() const;

  bool hasNullSpaceObjective() const;

  const std::shared_ptr<optimizer::Problem>& getProblem();

  std::shared_ptr<const optimizer::Problem> getProblem() const;

  void resetProblem(bool _clearSeeds=false);

  void setSolver(const std::shared_ptr<optimizer::Solver>& _newSolver);

  const std::shared_ptr<optimizer::Solver>& getSolver();

  std::shared_ptr<const optimizer::Solver> getSolver() const;

  void setConfiguration(const Eigen::VectorXd& _q);

  virtual void refreshIKs() = 0;

  virtual const std::vector< std::shared_ptr<InverseKinematics> >& getIKs() const = 0;

protected:

  HierarchicalIK(const SkeletonPtr& _skeleton);

  WeakSkeletonPtr mSkeleton;

};

/// The CompositeIK class allows you to specify an arbitrary hierarchy of
/// InverseKinematics modules for a single Skeleton. Simply add in each IK
/// module that should be used.
class CompositeIK : public HierarchicalIK
{
public:

  static std::shared_ptr<CompositeIK> create(const SkeletonPtr& _skel);

  std::shared_ptr<HierarchicalIK> clone(
      const SkeletonPtr &_newSkel) const override;

  virtual std::shared_ptr<CompositeIK> cloneCompositeIK(
      const SkeletonPtr& _newSkel) const;

  /// Add an IK module to this CompositeIK. This function will return true if
  /// the module belongs to the Skeleton that this CompositeIK is associated
  /// with, otherwise it will return false.
  bool addIK(const InverseKinematicsPtr& _ik);

  void refreshIKs() override;

  const std::vector< std::shared_ptr<InverseKinematics> >& getIKs() const override;

protected:

  CompositeIK(const SkeletonPtr& _skel);
};

/// The WholeBodyIK class provides an interface for simultaneously solving all
/// the IK constraints of all BodyNodes and EndEffectors belonging to a single
/// Skeleton.
class WholeBodyIK : public HierarchicalIK
{
public:

  static std::shared_ptr<WholeBodyIK> create(const SkeletonPtr& _skel);

  std::shared_ptr<HierarchicalIK> clone(
      const SkeletonPtr &_newSkel) const override;

  virtual std::shared_ptr<WholeBodyIK> cloneWholeBodyIK(
      const SkeletonPtr& _newSkel) const;

  void refreshIKs() override;

  const std::vector< std::shared_ptr<InverseKinematics> >& getIKs() const override;

protected:

  WholeBodyIK(const SkeletonPtr& _skel);
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_HIERARCHICALIK_H_
