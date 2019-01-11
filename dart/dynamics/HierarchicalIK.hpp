/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_DYNAMICS_HIERARCHICALIK_HPP_
#define DART_DYNAMICS_HIERARCHICALIK_HPP_

#include <unordered_set>

#include "dart/dynamics/InverseKinematics.hpp"

namespace dart {
namespace dynamics {

/// An IKHierarchy is a sorted set of IK modules. The outer vector represents
/// the precedence of the IK modules. Modules with the same precedence will have
/// their gradients added. Precedence of the modules decreases as the index of
/// the outer vector increases. Modules with lower precedence will be projected
/// through the null spaces of modules with higher precedence.
typedef std::vector< std::vector< std::shared_ptr<InverseKinematics> > > IKHierarchy;

/// The HierarchicalIK class provides a convenient way of setting up a
/// hierarchical inverse kinematics optimization problem which combines several
/// InverseKinematics problems into one. InverseKinematics problems with a
/// larger hierarchy level will be projected into null spaces of the problems
/// that have a smaller hierarchy number.
///
/// Note that the HierarchicalIK will only account for the
/// InverseKinematics::ErrorMethod and InverseKinematics::GradientMethod that
/// the IK modules specify; it will ignore any other constraints or objectives
/// put into the IK modules' Problems. Any additional constraints or objectives
/// that you want the HierarchicalIK to solve should be put directly into the
/// HierarchicalIK's Problem.
class HierarchicalIK : public common::Subject
{
public:

  /// Virtual destructor
  virtual ~HierarchicalIK() = default;

  /// Solve the IK Problem. By default, the Skeleton itself will retain the
  /// solved joint positions. If you pass in false for _applySolution, then the
  /// joint positions will be return to their original positions after the
  /// problem is solved.
  bool solve(bool _applySolution = true);

  /// Same as solve(bool), but the positions vector will be filled with the
  /// solved positions.
  bool solve(Eigen::VectorXd& positions, bool _applySolution = true);

  /// Clone this HierarchicalIK module
  virtual std::shared_ptr<HierarchicalIK> clone(
      const SkeletonPtr& _newSkel) const = 0;

  /// This class should be inherited by optimizer::Function classes that have a
  /// dependency on the HierarchicalIK module that they belong to. If you
  /// pass an HierarchicalIK::Function into the Problem of an
  /// HierarchicalIK module, then it will be properly cloned whenever the
  /// HierarchicalIK module that it belongs to gets cloned. Any Function
  /// classes in the Problem that do not inherit HierarchicalIK::Function
  /// will just be copied over by reference.
  class Function
  {
  public:

    /// Enable this function to be cloned to a new IK module.
    virtual optimizer::FunctionPtr clone(
        const std::shared_ptr<HierarchicalIK>& _newIK) const = 0;

    /// Virtual destructor
    virtual ~Function() = default;
  };

  /// Set the objective function for this HierarchicalIK.
  void setObjective(const std::shared_ptr<optimizer::Function>& _objective);

  /// Get the objective function for this HierarchicalIK.
  const std::shared_ptr<optimizer::Function>& getObjective();

  /// Get the objective function for this HierarchicalIK.
  std::shared_ptr<const optimizer::Function> getObjective() const;

  /// Set the null space objective for this HierarchicalIK.
  void setNullSpaceObjective(
      const std::shared_ptr<optimizer::Function>& _nsObjective);

  /// Get the null space objective for this HierarchicalIK.
  const std::shared_ptr<optimizer::Function>& getNullSpaceObjective();

  /// Get the null space objective for this HierarchicalIK.
  std::shared_ptr<const optimizer::Function> getNullSpaceObjective() const;

  /// Returns true if this HierarchicalIK has a null space objective.
  bool hasNullSpaceObjective() const;

  /// Get the Problem that is being maintained by this HierarchicalIK module
  const std::shared_ptr<optimizer::Problem>& getProblem();

  /// Get the Problem that is being maintained by this HierarchicalIK module
  std::shared_ptr<const optimizer::Problem> getProblem() const;

  /// Reset the Problem that is being maintained by this HierarchicalIK module.
  /// This will clear out all Functions from the Problem and then configure the
  /// Problem to use this IK module's Objective and Constraint functions.
  ///
  /// Setting _clearSeeds to true will clear out any seeds that have been loaded
  /// into the Problem.
  void resetProblem(bool _clearSeeds=false);

  /// Set the Solver that should be used by this IK module, and set it up with
  /// the Problem that is configured by this IK module
  void setSolver(const std::shared_ptr<optimizer::Solver>& _newSolver);

  /// Get the Solver that is being used by this IK module.
  const std::shared_ptr<optimizer::Solver>& getSolver();

  /// Get the Solver that is being used by this IK module.
  std::shared_ptr<const optimizer::Solver> getSolver() const;

  /// Refresh the IK hierarchy of this IK module
  virtual void refreshIKHierarchy() = 0;

  /// Get the IK hierarchy of this IK module
  const IKHierarchy& getIKHierarchy() const;

  /// Compute the null spaces of each level of the hierarchy
  const std::vector<Eigen::MatrixXd>& computeNullSpaces() const;

  /// Get the current joint positions of the Skeleton associated with this
  /// IK module.
  Eigen::VectorXd getPositions() const;

  /// Set the current joint positions of the Skeleton associated with this
  /// IK module. The vector must include all DOFs in the Skeleton.
  void setPositions(const Eigen::VectorXd& _q);

  /// Get the Skeleton that this IK module is associated with
  SkeletonPtr getSkeleton();

  /// Get the Skeleton that this IK module is associated with
  ConstSkeletonPtr getSkeleton() const;

  /// This is the same as getSkeleton(). It is used by the HierarchicalIKPtr to
  /// provide a common interface for the various IK smart pointer types.
  SkeletonPtr getAffiliation();

  /// This is the same as getSkeleton(). It is used by the HierarchicalIKPtr to
  /// provide a common interface for the various IK smart pointer types.
  ConstSkeletonPtr getAffiliation() const;

  /// Clear the caches of this IK module. It should generally not be necessary
  /// to call this function.
  void clearCaches();

protected:

  /// The HierarchicalIK::Objective Function is simply used to merge the
  /// objective and null space objective functions that are being held by this
  /// HierarchicalIK module. This class is not meant to be extended or
  /// instantiated by a user. Call HierarchicalIK::resetProblem() to set
  /// the objective of the module's Problem to an HierarchicalIK::Objective.
  class Objective final : public Function, public optimizer::Function
  {
  public:

    /// Constructor
    Objective(const std::shared_ptr<HierarchicalIK>& _ik);

    /// Virtual destructor
    virtual ~Objective() = default;

    // Documentation inherited
    optimizer::FunctionPtr clone(
        const std::shared_ptr<HierarchicalIK>& _newIK) const override;

    // Documentation inherited
    double eval(const Eigen::VectorXd &_x) const override;

    // Documentation inherited
    void evalGradient(const Eigen::VectorXd& _x,
                      Eigen::Map<Eigen::VectorXd> _grad) const override;

  protected:

    /// Pointer to this Objective's HierarchicalIK module
    std::weak_ptr<HierarchicalIK> mIK;

    /// Cache for the gradient computation
    mutable Eigen::VectorXd mGradCache;
  };

  /// The HierarchicalIK::Constraint Function is simply used to merge the
  /// constraints of the InverseKinematics modules that belong to the hierarchy
  /// of this HierarchicalIK module. This class is not meant to be extended or
  /// instantiated by a user. Call HierarchicalIK::resetProblem() to set
  /// the constraint of the module's Problem to an HierarchicalIK::Constraint.
  class Constraint final : public Function, public optimizer::Function
  {
  public:

    /// Constructor
    Constraint(const std::shared_ptr<HierarchicalIK>& _ik);

    /// Virtual destructor
    virtual ~Constraint() = default;

    // Documentation inherited
    optimizer::FunctionPtr clone(
        const std::shared_ptr<HierarchicalIK>& _newIK) const override;

    // Documentation inherited
    double eval(const Eigen::VectorXd& _x) const override;

    // Documentation inherited
    void evalGradient(const Eigen::VectorXd& _x,
                      Eigen::Map<Eigen::VectorXd> _grad) const override;

  protected:

    /// Pointer to this Constraint's HierarchicalIK module
    std::weak_ptr<HierarchicalIK> mIK;

    /// Cache for the gradient of a level
    mutable Eigen::VectorXd mLevelGradCache;

    /// Cache for temporary gradients
    mutable Eigen::VectorXd mTempGradCache;
  };

  /// Constructor
  HierarchicalIK(const SkeletonPtr& _skeleton);

  /// Setup the module
  void initialize(const std::shared_ptr<HierarchicalIK>& my_ptr);

  /// Copy the setup of this HierarchicalIK module into another HierarchicalIK
  /// module
  void copyOverSetup(const std::shared_ptr<HierarchicalIK>& _otherIK) const;

  /// Pointer to the Skeleton that this IK is tied to
  WeakSkeletonPtr mSkeleton;

  /// Cache for the IK hierarcy
  IKHierarchy mHierarchy;

  /// The Problem that this IK module is maintaining
  std::shared_ptr<optimizer::Problem> mProblem;

  /// The Solver that this IK module will use
  std::shared_ptr<optimizer::Solver> mSolver;

  /// The Objective of this IK module
  optimizer::FunctionPtr mObjective;

  /// The null space Objective of this IK module
  optimizer::FunctionPtr mNullSpaceObjective;

  /// Weak pointer to self
  std::weak_ptr<HierarchicalIK> mPtr;

  /// Cache for the last positions
  mutable Eigen::VectorXd mLastPositions;

  /// Cache for null space computations
  mutable std::vector<Eigen::MatrixXd> mNullSpaceCache;

  /// Cache for a partial null space computation
  mutable Eigen::MatrixXd mPartialNullspaceCache;

  /// Cache for the null space SVD
  mutable Eigen::JacobiSVD<math::Jacobian> mSVDCache;

  /// Cache for Jacobians
  mutable math::Jacobian mJacCache;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// The CompositeIK class allows you to specify an arbitrary hierarchy of
/// InverseKinematics modules for a single Skeleton. Simply add in each IK
/// module that should be used.
class CompositeIK : public HierarchicalIK
{
public:

  typedef std::unordered_set< std::shared_ptr<InverseKinematics> > ModuleSet;
  typedef std::unordered_set< std::shared_ptr<const InverseKinematics> > ConstModuleSet;

  /// Create a CompositeIK module
  static std::shared_ptr<CompositeIK> create(const SkeletonPtr& _skel);

  // Documentation inherited
  std::shared_ptr<HierarchicalIK> clone(
      const SkeletonPtr &_newSkel) const override;

  /// Same as clone(), but passes back a more complete type
  virtual std::shared_ptr<CompositeIK> cloneCompositeIK(
      const SkeletonPtr& _newSkel) const;

  /// Add an IK module to this CompositeIK. This function will return true if
  /// the module belongs to the Skeleton that this CompositeIK is associated
  /// with, otherwise it will return false.
  bool addModule(const std::shared_ptr<InverseKinematics>& _ik);

  /// Get the set of modules being used by this CompositeIK
  const ModuleSet& getModuleSet();

  /// Get the set of modules being used by this CompositeIK
  ConstModuleSet getModuleSet() const;

  // Documentation inherited
  void refreshIKHierarchy() override;

protected:

  /// Constructor
  CompositeIK(const SkeletonPtr& _skel);

  /// The set of modules being used by this CompositeIK
  std::unordered_set< std::shared_ptr<InverseKinematics> > mModuleSet;
};

/// The WholeBodyIK class provides an interface for simultaneously solving all
/// the IK constraints of all BodyNodes and EndEffectors belonging to a single
/// Skeleton.
class WholeBodyIK : public HierarchicalIK
{
public:

  /// Create a WholeBodyIK
  static std::shared_ptr<WholeBodyIK> create(const SkeletonPtr& _skel);

  // Documentation inherited
  std::shared_ptr<HierarchicalIK> clone(
      const SkeletonPtr &_newSkel) const override;

  /// Same as clone(), but produces a more complete type
  virtual std::shared_ptr<WholeBodyIK> cloneWholeBodyIK(
      const SkeletonPtr& _newSkel) const;

  // Documentation inherited
  void refreshIKHierarchy() override;

protected:

  /// Constructor
  WholeBodyIK(const SkeletonPtr& _skel);
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_HIERARCHICALIK_HPP_
