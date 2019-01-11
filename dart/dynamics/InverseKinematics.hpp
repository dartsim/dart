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

#ifndef DART_DYNAMICS_INVERSEKINEMATICS_HPP_
#define DART_DYNAMICS_INVERSEKINEMATICS_HPP_

#include <memory>
#include <functional>

#include <Eigen/SVD>

#include "dart/common/sub_ptr.hpp"
#include "dart/common/Signal.hpp"
#include "dart/common/Subject.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/optimizer/Solver.hpp"
#include "dart/optimizer/Problem.hpp"
#include "dart/optimizer/Function.hpp"
#include "dart/dynamics/SmartPointer.hpp"
#include "dart/dynamics/JacobianNode.hpp"

namespace dart {
namespace dynamics {

const double DefaultIKTolerance = 1e-6;
const double DefaultIKErrorClamp = 1.0;
const double DefaultIKGradientComponentClamp = 0.2;
const double DefaultIKGradientComponentWeight = 1.0;
const double DefaultIKDLSCoefficient = 0.05;
const double DefaultIKAngularWeight = 0.4;
const double DefaultIKLinearWeight = 1.0;

/// The InverseKinematics class provides a convenient way of setting up an IK
/// optimization problem. It generates constraint functions based on the
/// specified InverseKinematics::ErrorMethod and
/// InverseKinematics::GradientMethod. The optimization problem is then solved
/// by any classes derived from optimizer::Solver class. The default solver is
/// optimizer::GradientDescentSolver.
///
/// It also provides a convenient way of specifying a configuration space
/// objective and a null space objective. It is also possible to fully customize
/// the optimizer::Problem that the module creates, and the IK module can be
/// safely cloned  over to another JacobianNode, as long as every
/// optimizer::Function that depends on the JacobianNode inherits the
/// InverseKinematics::Function class and correctly overloads the clone function
class InverseKinematics : public common::Subject
{
public:

  /// Create an InverseKinematics module for a specified node
  static InverseKinematicsPtr create(JacobianNode* _node);

  /// Copying is not allowed
  InverseKinematics(const InverseKinematics&) = delete;

  /// Assignment is not allowed
  InverseKinematics& operator = (const InverseKinematics&) = delete;

  /// Virtual destructor
  virtual ~InverseKinematics();

  /// Solve the IK Problem.
  ///
  /// The initial guess for the IK optimization problem is the current joint
  /// positions of the target system. If the iterative solver fails to find a
  /// successive solution, it attempts more to solve the problem with other seed
  /// configurations or random configurations if enough seed is not provided.
  ///
  /// Here is the pseudocode as described above:
  ///
  /// \code
  /// attempts <- 0
  /// initial_guess <- current_joint_positions
  /// while attempts <= max_attempts:
  ///   result <- solve(initial_guess)
  ///   if result = success:
  ///     return
  ///   else:
  ///     attempts <- attempts + 1
  ///     if attempts <= num_seed:
  ///       initial_guess <- seed[attempts - 1]
  ///     else:
  ///       initial_guess <- random_configuration  // within the bounds
  /// \endcode
  ///
  /// By default, the max_attempts is 1, but this can be changed by calling
  /// InverseKinematics::getSolver() and casting the SolverPtr to an
  /// optimizer::GradientDescentSolver (unless you have changed the Solver type)
  /// and then calling GradientDescentSolver::setMaxAttempts(std::size_t).
  ///
  /// By default, the list of seeds is empty, but they can be added by calling
  /// InverseKinematics::getProblem() and then using
  /// Problem::addSeed(Eigen::VectorXd).
  ///
  /// By default, the Skeleton itself will retain the solved joint positions.
  /// If you pass in false for _applySolution, then the joint positions will be
  /// returned to their original positions after the problem is solved.
  ///
  /// Calling this function will automatically call Position::setLowerBounds(~)
  /// and Position::setUpperBounds(~) with the lower/upper position bounds of
  /// the corresponding Degrees Of Freedom. Problem::setDimension(~) will be
  /// taken care of automatically, and Problem::setInitialGuess(~) will be
  /// called with the current positions of the Degrees Of Freedom.
  bool solve(bool _applySolution = true);

  /// Same as solve(bool), but the positions vector will be filled with the
  /// solved positions.
  bool solve(Eigen::VectorXd& positions, bool _applySolution = true);

  /// Clone this IK module, but targeted at a new Node. Any Functions in the
  /// Problem that inherit InverseKinematics::Function will be adapted to the
  /// new IK module. Any generic optimizer::Function will just be copied over
  /// by pointer instead of being cloned.
  InverseKinematicsPtr clone(JacobianNode* _newNode) const;

  // For the definitions of these classes, see the bottom of this header
  class Function;

  // Methods for computing IK error
  class ErrorMethod;
  class TaskSpaceRegion;

  // Methods for computing IK gradient
  class GradientMethod;
  class JacobianDLS;
  class JacobianTranspose;
  class Analytical;

  /// If this IK module is set to active, then it will be utilized by any
  /// HierarchicalIK that has it in its list. If it is set to inactive, then it
  /// will be ignored by any HierarchicalIK holding onto it, but you can still
  /// use the solve() function with this.
  void setActive(bool _active=true);

  /// Equivalent to setActive(false)
  void setInactive();

  /// Returns true if this IK module is allowed to be active in a HierarchicalIK
  bool isActive() const;

  /// Set the hierarchy level of this module. Modules with a larger hierarchy
  /// value will be projected through the null spaces of all modules with a
  /// smaller hierarchy value. In other words, IK modules with a hierarchy level
  /// closer to 0 are given higher priority.
  void setHierarchyLevel(std::size_t _level);

  /// Get the hierarchy level of this modle.
  std::size_t getHierarchyLevel() const;

  /// When solving the IK for this module's Node, use the longest available
  /// dynamics::Chain that goes from this module's Node towards the root of the
  /// Skeleton. Using this will prevent any other branches in the Skeleton from
  /// being affected by this IK module.
  void useChain();

  /// Use all relevant joints on the Skeleton to solve the IK.
  void useWholeBody();

  /// Explicitly set which degrees of freedom should be used to solve the IK for
  /// this module.
  template <class DegreeOfFreedomT>
  void setDofs(const std::vector<DegreeOfFreedomT*>& _dofs);

  /// Explicitly set which degrees of freedom should be used to solve the IK for
  /// this module. The values in the vector should correspond to the Skeleton
  /// indices of each DOF.
  void setDofs(const std::vector<std::size_t>& _dofs);

  /// Get the indices of the DOFs that this IK module will use when solving.
  const std::vector<std::size_t>& getDofs() const;

  /// When a Jacobian is computed for a JacobianNode, it will include a column
  /// for every DegreeOfFreedom that the node depends on. Given the column index
  /// of one of those dependent coordinates, this map will return its location
  /// in the mDofs vector. A value of -1 means that it is not present in the
  /// mDofs vector and therefore should not be used when performing inverse
  /// kinematics.
  const std::vector<int>& getDofMap() const;

  /// Set an objective function that should be minimized while satisfying the
  /// inverse kinematics constraint. Pass in a nullptr to remove the objective
  /// and make it a constant-zero function.
  void setObjective(const std::shared_ptr<optimizer::Function>& _objective);

  /// Get the objective function for this IK module
  const std::shared_ptr<optimizer::Function>& getObjective();

  /// Get the objective function for this IK module
  std::shared_ptr<const optimizer::Function> getObjective() const;

  /// Set an objective function that should be minimized within the null space
  /// of the inverse kinematics constraint. The gradient of this function will
  /// always be projected through the null space of this IK module's Jacobian.
  /// Pass in a nullptr to remove the null space objective.
  ///
  /// Note: The objectives given to setObjective() and setNullSpaceObjective()
  /// will always be superimposed (added together) via the evalObjective()
  /// function.
  void setNullSpaceObjective(
      const std::shared_ptr<optimizer::Function>& _nsObjective);

  /// Get the null space objective for this IK module
  const std::shared_ptr<optimizer::Function>& getNullSpaceObjective();

  /// Get the null space objective for this IK module
  std::shared_ptr<const optimizer::Function> getNullSpaceObjective() const;

  /// Returns false if the null space objective does nothing
  bool hasNullSpaceObjective() const;

  /// Set the ErrorMethod for this IK module. You can pass in arguments for the
  /// constructor, but you should ignore the constructor's first argument. The
  /// first argument of the ErrorMethod's constructor must be a pointer to this
  /// IK module, which will be automatically passed by this function.
  template <class IKErrorMethod, typename... Args>
  IKErrorMethod& setErrorMethod(Args&&... args);

  /// Get the ErrorMethod for this IK module. Every IK module always has an
  /// ErrorMethod available, so this is passed by reference.
  ErrorMethod& getErrorMethod();

  /// Get the ErrorMethod for this IK module. Every IK module always has an
  /// ErrorMethod available, so this is passed by reference.
  const ErrorMethod& getErrorMethod() const;

  /// Set the GradientMethod for this IK module. You can pass in arguments for
  /// the constructor, but you should ignore the constructor's first argument.
  /// The first argument of the GradientMethod's constructor must be a pointer
  /// to this IK module, which will be automatically passed by this function.
  template <class IKGradientMethod, typename... Args>
  IKGradientMethod& setGradientMethod(Args&&... args);

  /// Get the GradientMethod for this IK module. Every IK module always has a
  /// GradientMethod available, so this is passed by reference.
  GradientMethod& getGradientMethod();

  /// Get the GradientMethod for this IK module. Every IK module always has a
  /// GradientMethod available, so this is passed by reference.
  const GradientMethod& getGradientMethod() const;

  /// Get the Analytical IK method for this module, if one is available.
  /// Analytical methods are not provided by default. If this IK module does not
  /// have an analytical method, then this will return a nullptr.
  Analytical* getAnalytical();

  /// Get the Analytical IK method for this module, if one is available.
  /// Analytical methods are not provided by default. If this IK module does not
  /// have an analytical method, then this will return a nullptr.
  const Analytical* getAnalytical() const;

  /// Get the Problem that is being maintained by this IK module.
  const std::shared_ptr<optimizer::Problem>& getProblem();

  /// Get the Problem that is being maintained by this IK module.
  std::shared_ptr<const optimizer::Problem> getProblem() const;

  /// Reset the Problem that is being maintained by this IK module. This will
  /// clear out all Functions from the Problem and then configure the Problem to
  /// use this IK module's Objective and Constraint functions.
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

  /// Inverse kinematics can be performed on any point within the body frame.
  /// The default point is the origin of the body frame. Use this function to
  /// change the point that will be used. _offset must represent the offset of
  /// the desired point from the body origin, expressed in coordinates of the
  /// body frame.
  void setOffset(const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero());

  /// Get the offset from the origin of the body frame that will be used when
  /// performing inverse kinematics. The offset will be expressed in the
  /// coordinates of the body frame.
  const Eigen::Vector3d& getOffset() const;

  /// This returns false if the offset for the inverse kinematics is a zero
  /// vector. Otherwise, it returns true. Use setOffset() to set the offset and
  /// getOffset() to get the offset.
  bool hasOffset() const;

  /// Set the target for this IK module.
  ///
  /// Note that a target will automatically be created for the IK module upon
  /// instantiation, so you typically do not need to use this function. If you
  /// want to attach the target to an arbitrary (non-SimpleFrame) reference
  /// frame, you can do getTarget()->setParentFrame(arbitraryFrame)
  void setTarget(std::shared_ptr<SimpleFrame> _newTarget);

  /// Get the target that is being used by this IK module. You never have to
  /// check whether this is a nullptr, because it cannot ever be set to nullptr.
  std::shared_ptr<SimpleFrame> getTarget();

  /// Get the target that is being used by this IK module. You never have to
  /// check whether this is a nullptr, because it cannot ever be set to nullptr.
  std::shared_ptr<const SimpleFrame> getTarget() const;

  /// Get the JacobianNode that this IK module operates on.
  JacobianNode* getNode();

  /// Get the JacobianNode that this IK module operates on.
  const JacobianNode* getNode() const;

  /// This is the same as getNode(). It is used by the InverseKinematicsPtr to
  /// provide a common interface for the various IK smart pointer types.
  JacobianNode* getAffiliation();

  /// This is the same as getNode(). It is used by the InverseKinematicsPtr to
  /// provide a common interface for the various IK smart pointer types.
  const JacobianNode* getAffiliation() const;

  /// Compute the Jacobian for this IK module's node, using the Skeleton's
  /// current joint positions and the DOFs that have been assigned to the
  /// module.
  const math::Jacobian& computeJacobian() const;

  /// Get the current joint positions of the Skeleton. This will only include
  /// the DOFs that have been assigned to this IK module, and the components of
  /// the vector will correspond to the components of getDofs().
  Eigen::VectorXd getPositions() const;

  /// Set the current joint positions of the Skeleton. This must only include
  /// the DOFs that have been assigned to this IK module, and the components of
  /// the vector must correspond to the components of getDofs().
  void setPositions(const Eigen::VectorXd& _q);

  /// Clear the caches of this IK module. It should generally not be necessary
  /// to call this function. However, if you have some non-standard external
  /// dependency for your error and/or gradient method computations, then you
  /// will need to tie this function to something that tracks changes in that
  /// dependency.
  void clearCaches();

protected:

  // For the definitions of these functions, see the bottom of this header
  class Objective;
  friend class Objective;
  class Constraint;
  friend class Constraint;

  /// Constructor that accepts a JacobianNode
  InverseKinematics(JacobianNode* _node);

  /// Gets called during construction
  void initialize();

  /// Reset the signal connection for this IK module's target
  void resetTargetConnection();

  /// Reset the signal connection for this IK module's Node
  void resetNodeConnection();

  /// Connection to the target update
  common::Connection mTargetConnection;

  /// Connection to the node update
  common::Connection mNodeConnection;

  /// True if this IK module should be active in its IK hierarcy
  bool mActive;

  /// Hierarchy level for this IK module
  std::size_t mHierarchyLevel;

  /// A list of the DegreeOfFreedom Skeleton indices that will be used by this
  /// IK module
  std::vector<std::size_t> mDofs;

  /// Map for the DOFs that are to be used by this IK module
  std::vector<int> mDofMap;

  /// Objective for the IK module
  std::shared_ptr<optimizer::Function> mObjective;

  /// Null space objective for the IK module
  std::shared_ptr<optimizer::Function> mNullSpaceObjective;

  /// The method that this IK module will use to compute errors
  std::unique_ptr<ErrorMethod> mErrorMethod;

  /// The method that this IK module will use to compute gradients
  std::unique_ptr<GradientMethod> mGradientMethod;

  /// If mGradientMethod is an Analytical extension, then this will have the
  /// same value is mGradientMethod. Otherwise, this will be a nullptr.
  ///
  /// Note that this pointer's memory does not need to be managed, because it is
  /// always either nullptr or a reference to mGradientMethod.
  Analytical* mAnalytical;

  /// The Problem that will be maintained by this IK module
  std::shared_ptr<optimizer::Problem> mProblem;

  /// The solver that this IK module will use for iterative methods
  std::shared_ptr<optimizer::Solver> mSolver;

  /// The offset that this IK module should use when computing IK
  Eigen::Vector3d mOffset;

  /// True if the offset is non-zero
  bool mHasOffset;

  /// Target that this IK module should use
  std::shared_ptr<SimpleFrame> mTarget;

  /// JacobianNode that this IK module is associated with
  sub_ptr<JacobianNode> mNode;

  /// Jacobian cache for the IK module
  mutable math::Jacobian mJacobian;
};

typedef InverseKinematics IK;

//==============================================================================
/// This class should be inherited by optimizer::Function classes that have a
/// dependency on the InverseKinematics module that they belong to. If you
/// pass an InverseKinematics::Function into the Problem of an
/// InverseKinematics module, then it will be properly cloned whenever the
/// InverseKinematics module that it belongs to gets cloned. Any Function
/// classes in the Problem that do not inherit InverseKinematics::Function
/// will just be copied over by reference.
class InverseKinematics::Function
{
public:

  /// Enable this function to be cloned to a new IK module.
  virtual optimizer::FunctionPtr clone(InverseKinematics* _newIK) const = 0;

  /// Virtual destructor
  virtual ~Function() = default;
};

//==============================================================================
/// ErrorMethod is a base class for different ways of computing the error of
/// an InverseKinematics module.
class InverseKinematics::ErrorMethod : public common::Subject
{
public:

  typedef std::pair<Eigen::Vector6d, Eigen::Vector6d> Bounds;

  /// The Properties struct contains settings that are commonly used by
  /// methods that compute error for inverse kinematics.
  struct Properties
  {
    /// Default constructor
    Properties(const Bounds& _bounds =
          Bounds(Eigen::Vector6d::Constant(-DefaultIKTolerance),
                 Eigen::Vector6d::Constant( DefaultIKTolerance)),

        double _errorClamp = DefaultIKErrorClamp,

        const Eigen::Vector6d& _errorWeights = Eigen::compose(
          Eigen::Vector3d::Constant(DefaultIKAngularWeight),
          Eigen::Vector3d::Constant(DefaultIKLinearWeight)));

    /// Bounds that define the acceptable range of the Node's transform
    /// relative to its target frame.
    std::pair<Eigen::Vector6d, Eigen::Vector6d> mBounds;

    /// The error vector will be clamped to this length with each iteration.
    /// This is used to enforce sane behavior, even when there are extremely
    /// large error vectors.
    double mErrorLengthClamp;

    /// These weights will be applied to the error vector component-wise. This
    /// allows you to set some components of error as more important than
    /// others, or to scale their coordinate spaces. For example, you will
    /// often want the first three components (orientation error) to have
    /// smaller weights than the last three components (translation error).
    Eigen::Vector6d mErrorWeights;

    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /// Constructor
  ErrorMethod(InverseKinematics* _ik,
              const std::string& _methodName,
              const Properties& _properties = Properties());

  /// Virtual destructor
  virtual ~ErrorMethod() = default;

  /// Enable this ErrorMethod to be cloned to a new IK module.
  virtual std::unique_ptr<ErrorMethod> clone(
      InverseKinematics* _newIK) const = 0;

  /// Override this function with your implementation of the error vector
  /// computation. The expectation is that the first three components of the
  /// vector will correspond to orientation error (in an angle-axis format)
  /// while the last three components correspond to translational error.
  ///
  /// When implementing this function, you should assume that the Skeleton's
  /// current joint positions corresponds to the positions that you
  /// must use to compute the error. This function will only get called when
  /// an update is needed.
  virtual Eigen::Vector6d computeError() = 0;

  /// Override this function with your implementation of computing the desired
  /// given the current transform and error vector. If you want the desired
  /// transform to always be equal to the Target's transform, you can simply
  /// call ErrorMethod::computeDesiredTransform to implement this function.
  virtual Eigen::Isometry3d computeDesiredTransform(
      const Eigen::Isometry3d& _currentTf, const Eigen::Vector6d& _error) = 0;

  /// This function is used to handle caching the error vector.
  const Eigen::Vector6d& evalError(const Eigen::VectorXd& _q);

  /// Get the name of this ErrorMethod.
  const std::string& getMethodName() const;

  /// Set all the error bounds.
  void setBounds(
      const Eigen::Vector6d& _lower =
          Eigen::Vector6d::Constant(-DefaultIKTolerance),
      const Eigen::Vector6d& _upper =
          Eigen::Vector6d::Constant( DefaultIKTolerance));

  /// Set all the error bounds.
  void setBounds(const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds);

  /// Get all the error bounds.
  const std::pair<Eigen::Vector6d, Eigen::Vector6d>& getBounds() const;

  /// Set the error bounds for orientation.
  void setAngularBounds(
      const Eigen::Vector3d& _lower =
          Eigen::Vector3d::Constant(-DefaultIKTolerance),
      const Eigen::Vector3d& _upper =
          Eigen::Vector3d::Constant( DefaultIKTolerance));

  /// Set the error bounds for orientation.
  void setAngularBounds(
      const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds);

  /// Get the error bounds for orientation.
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getAngularBounds() const;

  /// Set the error bounds for translation.
  void setLinearBounds(
      const Eigen::Vector3d& _lower =
          Eigen::Vector3d::Constant(-DefaultIKTolerance),
      const Eigen::Vector3d& _upper =
          Eigen::Vector3d::Constant( DefaultIKTolerance));

  /// Set the error bounds for translation.
  void setLinearBounds(
      const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds);

  /// Get the error bounds for translation.
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getLinearBounds() const;

  /// Set the clamp that will be applied to the length of the error vector
  /// each iteration.
  void setErrorLengthClamp(double _clampSize = DefaultIKErrorClamp);

  /// Set the clamp that will be applied to the length of the error vector
  /// each iteration.
  double getErrorLengthClamp() const;

  /// Set the weights that will be applied to each component of the error
  /// vector.
  void setErrorWeights(const Eigen::Vector6d& _weights);

  /// Get the weights that will be applied to each component of the error
  /// vector.
  const Eigen::Vector6d& getErrorWeights() const;

  /// Set the weights that will be applied to each angular component of the
  /// error vector.
  void setAngularErrorWeights(
      const Eigen::Vector3d& _weights =
        Eigen::Vector3d::Constant(DefaultIKAngularWeight));

  /// Get the weights that will be applied to each angular component of the
  /// error vector.
  Eigen::Vector3d getAngularErrorWeights() const;

  /// Set the weights that will be applied to each linear component of the
  /// error vector.
  void setLinearErrorWeights(
      const Eigen::Vector3d& _weights =
        Eigen::Vector3d::Constant(DefaultIKLinearWeight));

  /// Get the weights that will be applied to each linear component of the
  /// error vector.
  Eigen::Vector3d getLinearErrorWeights() const;

  /// Get the Properties of this ErrorMethod
  Properties getErrorMethodProperties() const;

  /// Clear the cache to force the error to be recomputed. It should generally
  /// not be necessary to call this function.
  void clearCache();

protected:

  /// Pointer to the IK module of this ErrorMethod
  common::sub_ptr<InverseKinematics> mIK;

  /// Name of this error method
  std::string mMethodName;

  /// The last joint positions passed into this ErrorMethod
  Eigen::VectorXd mLastPositions;

  /// The last error vector computed by this ErrorMethod
  Eigen::Vector6d mLastError;

  /// The properties of this ErrorMethod
  Properties mErrorP;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

//==============================================================================
/// The TaskSpaceRegion is a nicely generalized method for computing the error
/// of an IK Problem.
class InverseKinematics::TaskSpaceRegion : public ErrorMethod
{
public:

  struct UniqueProperties
  {
    /// Setting this to true (which is default) will tell it to compute the
    /// error based on the center of the Task Space Region instead of the edge
    /// of the Task Space Region. This often results in faster convergence, as
    /// the Node will enter the Task Space Region more aggressively.
    ///
    /// Once the Node is inside the Task Space Region, the error vector will
    /// drop to zero, regardless of whether this flag is true or false.
    bool mComputeErrorFromCenter;

    /// Default constructor
    UniqueProperties(bool computeErrorFromCenter = true);
  };

  struct Properties : ErrorMethod::Properties, UniqueProperties
  {
    /// Default constructor
    Properties(
        const ErrorMethod::Properties& errorProperties =
            ErrorMethod::Properties(),
        const UniqueProperties& taskSpaceProperties = UniqueProperties());
  };

  /// Default Constructor
  explicit TaskSpaceRegion(InverseKinematics* _ik,
                           const Properties& _properties = Properties());

  /// Virtual destructor
  virtual ~TaskSpaceRegion() = default;

  // Documentation inherited
  std::unique_ptr<ErrorMethod> clone(InverseKinematics* _newIK) const override;

  // Documentation inherited
  Eigen::Isometry3d computeDesiredTransform(
      const Eigen::Isometry3d& _currentTf,
      const Eigen::Vector6d& _error) override;

  // Documentation inherited
  Eigen::Vector6d computeError() override;

  /// Set whether this TaskSpaceRegion should compute its error vector from
  /// the center of the region.
  void setComputeFromCenter(bool computeFromCenter);

  /// Get whether this TaskSpaceRegion is set to compute its error vector from
  /// the center of the region.
  bool isComputingFromCenter() const;

  /// Get the Properties of this TaskSpaceRegion
  Properties getTaskSpaceRegionProperties() const;

protected:

  /// Properties of this TaskSpaceRegion
  UniqueProperties mTaskSpaceP;

};

//==============================================================================
/// GradientMethod is a base class for different ways of computing the
/// gradient of an InverseKinematics module.
class InverseKinematics::GradientMethod : public common::Subject
{
public:

  struct Properties
  {
    /// The component-wise clamp for this GradientMethod
    double mComponentWiseClamp;

    /// The weights for this GradientMethod
    Eigen::VectorXd mComponentWeights;

    /// Default constructor
    Properties(double clamp = DefaultIKGradientComponentClamp,
               const Eigen::VectorXd& weights = Eigen::VectorXd());
  };

  /// Constructor
  GradientMethod(InverseKinematics* _ik,
                 const std::string& _methodName,
                 const Properties& _properties);

  /// Virtual destructor
  virtual ~GradientMethod() = default;

  /// Enable this GradientMethod to be cloned to a new IK module
  virtual std::unique_ptr<GradientMethod> clone(
      InverseKinematics* _newIK) const = 0;

  /// Override this function with your implementation of the gradient
  /// computation. The direction that this gradient points in should make the
  /// error **worse** if applied to the joint positions, because the
  /// Problem is configured as a gradient **descent** error minimization
  /// Problem.
  ///
  /// The error vector that is passed in will be determined by the IK module's
  /// ErrorMethod. The expectation is that the first three components of the
  /// vector correspond to orientation error (in an angle-axis format) while
  /// the last three components correspond to translational error.
  ///
  /// When implementing this function, you should assume that the Skeleton's
  /// current joint positions corresponds to the positions that you
  /// must use to compute the error. This function will only get called when
  /// an update is needed.
  virtual void computeGradient(const Eigen::Vector6d& _error,
                               Eigen::VectorXd& _grad) = 0;

  /// This function is used to handle caching the gradient vector and
  /// interfacing with the solver.
  void evalGradient(const Eigen::VectorXd& _q,
                    Eigen::Map<Eigen::VectorXd> _grad);

  /// Get the name of this GradientMethod.
  const std::string& getMethodName() const;

  /// Clamp the gradient based on the clamp settings of this GradientMethod.
  void clampGradient(Eigen::VectorXd& _grad) const;

  /// Set the component-wise clamp for this GradientMethod. Each component
  /// of the gradient will be individually clamped to this size.
  void setComponentWiseClamp(double _clamp = DefaultIKGradientComponentClamp);

  /// Get the component-wise clamp for this GradientMethod.
  double getComponentWiseClamp() const;

  /// Apply weights to the gradient based on the weight settings of this
  /// GradientMethod.
  void applyWeights(Eigen::VectorXd& _grad) const;

  /// Set the weights that will be applied to each component of the gradient.
  /// If the number of components in _weights is smaller than the number of
  /// components in the gradient, then a weight of 1.0 will be applied to all
  /// components that are out of the range of _weights. Passing in an empty
  /// vector for _weights will effectively make all the gradient components
  /// unweighted.
  void setComponentWeights(const Eigen::VectorXd& _weights);

  /// Get the weights of this GradientMethod.
  const Eigen::VectorXd& getComponentWeights() const;

  /// Convert the gradient that gets generated by Jacobian methods into a
  /// gradient that can be used by a GradientDescentSolver.
  ///
  /// Not all Joint types can be integrated using standard addition (e.g.
  /// FreeJoint and BallJoint), therefore Jacobian-based differential methods
  /// will tend to generate gradients that cannot be correctly used by a
  /// simple addition-based GradientDescentSolver. Running your gradient
  /// through this function before returning it will make the gradient
  /// suitable for a standard solver.
  void convertJacobianMethodOutputToGradient(
      Eigen::VectorXd& grad, const std::vector<std::size_t>& dofs);

  /// Get the Properties of this GradientMethod
  Properties getGradientMethodProperties() const;

  /// Clear the cache to force the gradient to be recomputed. It should
  /// generally not be necessary to call this function.
  void clearCache();

  /// Returns the IK module that this GradientMethod belongs to.
  InverseKinematics* getIK();

  /// Returns the IK module that this GradientMethod belongs to.
  const InverseKinematics* getIK() const;

protected:

  /// The IK module that this GradientMethod belongs to.
  common::sub_ptr<InverseKinematics> mIK;

  /// The name of this method
  std::string mMethodName;

  /// The last positions that was passed to this GradientMethod
  Eigen::VectorXd mLastPositions;

  /// The last gradient that was computed by this GradientMethod
  Eigen::VectorXd mLastGradient;

  /// Properties for this GradientMethod
  Properties mGradientP;

private:

  /// Cache used by convertJacobianMethodOutputToGradient to avoid
  /// reallocating this vector on each iteration.
  Eigen::VectorXd mInitialPositionsCache;
};

//==============================================================================
/// JacobianDLS refers to the Damped Least Squares Jacobian Pseudoinverse
/// (specifically, Moore-Penrose Pseudoinverse). This is a very precise method
/// for computing the gradient and is especially suitable for performing IK on
/// industrial manipulators that need to follow very exact workspace paths.
/// However, it is vulnerable to be jittery around singularities (though the
/// damping helps with this), and each cycle might take more time to compute
/// than the JacobianTranspose method (although the JacobianDLS method will
/// usually converge in fewer cycles than JacobianTranspose).
class InverseKinematics::JacobianDLS : public GradientMethod
{
public:

  struct UniqueProperties
  {
    /// Damping coefficient
    double mDamping;

    /// Default constructor
    UniqueProperties(double damping = DefaultIKDLSCoefficient);
  };

  struct Properties : GradientMethod::Properties, UniqueProperties
  {
    /// Default constructor
    Properties(
        const GradientMethod::Properties& gradientProperties =
            GradientMethod::Properties(),
        const UniqueProperties& dlsProperties = UniqueProperties());
  };

  /// Constructor
  explicit JacobianDLS(InverseKinematics* _ik,
                       const Properties& properties = Properties());

  /// Virtual destructor
  virtual ~JacobianDLS() = default;

  // Documentation inherited
  std::unique_ptr<GradientMethod> clone(
      InverseKinematics* _newIK) const override;

  // Documentation inherited
  void computeGradient(const Eigen::Vector6d& _error,
                       Eigen::VectorXd& _grad) override;

  /// Set the damping coefficient. A higher damping coefficient will smooth
  /// out behavior around singularities but will also result in less precision
  /// in general. The default value is appropriate for most use cases.
  void setDampingCoefficient(double _damping = DefaultIKDLSCoefficient);

  /// Get the damping coefficient.
  double getDampingCoefficient() const;

  /// Get the Properties of this JacobianDLS
  Properties getJacobianDLSProperties() const;

protected:

  /// Properties of this Damped Least Squares method
  UniqueProperties mDLSProperties;

};

//==============================================================================
/// JacobianTranspose will simply apply the transpose of the Jacobian to the
/// error vector in order to compute the gradient. This method tends to be
/// very smooth but imprecise, requiring more iterations before converging
/// and being less precise in general. This method is suitable for animations
/// where smoothness is prefered over precision.
class InverseKinematics::JacobianTranspose : public GradientMethod
{
public:

  /// Constructor
  explicit JacobianTranspose(InverseKinematics* _ik,
                             const Properties& properties = Properties());

  /// Virtual destructor
  virtual ~JacobianTranspose() = default;

  // Documentation inherited
  std::unique_ptr<GradientMethod> clone(
      InverseKinematics* _newIK) const override;

  // Documentation inherited
  void computeGradient(const Eigen::Vector6d& _error,
                       Eigen::VectorXd& _grad) override;
};

//==============================================================================
/// Analytical is a base class that should be inherited by methods that are
/// made to solve the IK analytically instead of iteratively. This provides an
/// extended API that is relevant to Analytical solvers but not iterative
/// solvers.
///
/// Creating an Analytical solver will have the side effect of removing the
/// error clamp and error weights from your ErrorMethod. If you still want
/// your error computations to be clamped and weighted, you should set it
/// again after creating the Analytical solver. Clamping and weighting the
/// error vector often helps iterative methods to converge smoothly, but it is
/// counter-productive for analytical methods which do not typically rely on
/// convergence; analytical methods can usually solve the entire error vector
/// directly.
class InverseKinematics::Analytical : public GradientMethod
{
public:

  /// Bitwise enumerations that are used to describe some properties of each
  /// solution produced by the analytical IK.
  enum Validity_t
  {
    VALID = 0,                ///< The solution is completely valid and reaches the target
    OUT_OF_REACH   = 1 << 0,  ///< The solution does not reach the target
    LIMIT_VIOLATED = 1 << 1   ///< The solution has one or more joint positions that violate the joint limits
  };
  // TODO(JS): Change to enum class?

  /// If there are extra DOFs in the IK module which your Analytical solver
  /// implementation does not make use of, those DOFs can be used to
  /// supplement the analytical solver using Jacobian transpose iteration.
  /// This enumeration is used to indicate whether you want those DOFs to be
  /// used before applying the analytical solution, after applying the
  /// analytical solution, or not be used at all.
  ///
  /// Jacobian transpose is used for the extra DOFs because it is inexpensive
  /// and robust to degenerate Jacobians which are common in low dimensional
  /// joint spaces. The primary advantage of pseudoinverse methods over
  /// Jacobian transpose methods is their precision, but analytical methods
  /// are even more precise than pseudoinverse methods, so that precision is
  /// not needed in this case.
  ///
  /// If you want the extra DOFs to use a different method than Jacobian
  /// transpose, you can create two seperate IK modules (one which is
  /// analytical and one with the iterative method of your choice) and combine
  /// them in a HierarchicalIK.
  enum ExtraDofUtilization
  {
    UNUSED = 0,
    PRE_ANALYTICAL,
    POST_ANALYTICAL,
    PRE_AND_POST_ANALYTICAL
  };
  // TODO(JS): Change to enum class?

  struct Solution
  {
    /// Default constructor
    Solution(const Eigen::VectorXd& _config = Eigen::VectorXd(),
             int _validity = VALID);

    /// Configuration computed by the Analytical solver
    Eigen::VectorXd mConfig;

    /// Bitmap for whether this configuration is valid. Bitwise-compare it to
    /// the enumerations in Validity_t to whether this configuration is valid.
    int mValidity;
  };

  // std::function template for comparing the quality of configurations
  typedef std::function<bool(
      const Eigen::VectorXd& _better,
      const Eigen::VectorXd& _worse,
      const InverseKinematics* _ik)> QualityComparison;

  struct UniqueProperties
  {
    /// Flag for how to use the extra DOFs in the IK module.
    ExtraDofUtilization mExtraDofUtilization;

    /// How much to clamp the extra error that gets applied to DOFs
    double mExtraErrorLengthClamp;

    /// Function for comparing the qualities of solutions
    QualityComparison mQualityComparator;

    /// Default constructor. Uses a default quality comparison function.
    UniqueProperties(ExtraDofUtilization extraDofUtilization = UNUSED,
               double extraErrorLengthClamp = DefaultIKErrorClamp);

    /// Constructor that allows you to set the quality comparison function.
    UniqueProperties(ExtraDofUtilization extraDofUtilization,
               double extraErrorLengthClamp,
               QualityComparison qualityComparator);

    /// Reset the quality comparison function to its default behavior.
    void resetQualityComparisonFunction();
  };

  struct Properties : GradientMethod::Properties, UniqueProperties
  {
    // Default constructor
    Properties(
        const GradientMethod::Properties& gradientProperties =
            GradientMethod::Properties(),
        const UniqueProperties& analyticalProperties = UniqueProperties());

    // Construct Properties by specifying the UniqueProperties. The
    // GradientMethod::Properties components will be set to defaults.
    Properties(const UniqueProperties& analyticalProperties);
  };

  /// Constructor
  Analytical(InverseKinematics* _ik, const std::string& _methodName,
             const Properties& _properties);

  /// Virtual destructor
  virtual ~Analytical() = default;

  /// Get the solutions for this IK module, along with a tag indicating
  /// whether each solution is valid. This function will assume that you want
  /// to use the desired transform given by the IK module's current
  /// ErrorMethod.
  const std::vector<Solution>& getSolutions();

  /// Get the solutions for this IK module, along with a tag indicating
  /// whether each solution is valid. This function will compute the
  /// configurations using the given desired transform instead of using the
  /// IK module's current ErrorMethod.
  const std::vector<Solution>& getSolutions(
      const Eigen::Isometry3d& _desiredTf);

  /// You should not need to override this function. Instead, you should
  /// override computeSolutions.
  void computeGradient(const Eigen::Vector6d& _error,
                       Eigen::VectorXd& _grad) override;

  /// Use this function to fill the entries of the mSolutions variable. Be
  /// sure to clear the mSolutions vector at the start, and to also return the
  /// mSolutions vector at the end. Note that you are not expected to evaluate
  /// any of the solutions for their quality. However, you should set the
  /// Solution::mValidity flag to OUT_OF_REACH for each solution that does not
  /// actually reach the desired transform, and you should call
  /// checkSolutionJointLimits() and the end of the function, which will set
  /// the LIMIT_VIOLATED flags of any configurations that are outside of the
  /// position limits.
  virtual const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& _desiredBodyTf) = 0;

  /// Get a list of the DOFs that will be included in the entries of the
  /// solutions returned by getSolutions(). Ideally, this should match up with
  /// the DOFs being used by the InverseKinematics module, but this might not
  /// always be possible, so this function ensures that solutions can be
  /// interpreted correctly.
  virtual const std::vector<std::size_t>& getDofs() const = 0;

  /// Set the configuration of the DOFs. The components of _config must
  /// correspond to the DOFs provided by getDofs().
  void setPositions(const Eigen::VectorXd& _config);

  /// Get the configuration of the DOFs. The components of this vector will
  /// correspond to the DOFs provided by getDofs().
  Eigen::VectorXd getPositions() const;

  /// Set how you want extra DOFs to be utilized by the IK module
  void setExtraDofUtilization(ExtraDofUtilization _utilization);

  /// Get how extra DOFs are being utilized by the IK module
  ExtraDofUtilization getExtraDofUtilization() const;

  /// Set how much to clamp the error vector that gets applied to extra DOFs
  void setExtraErrorLengthClamp(double _clamp);

  /// Get how much we will clamp the error vector that gets applied to extra
  /// DOFs
  double getExtraErrorLengthClamp() const;

  /// Set the function that will be used to compare the qualities of two
  /// solutions. This function should return true if the first argument is a
  /// better solution than the second argument.
  ///
  /// By default, it will prefer the solution which has the smallest size for
  /// its largest change in joint angle. In other words, for each
  /// configuration that it is given, it will compare the largest change in
  /// joint angle for each configuration and pick the one that is smallest.
  ///
  /// Note that outside of this comparison function, the Solutions will be
  /// split between which are valid, which are out-of-reach, and which are
  /// in violation of joint limits. Valid solutions will always be ranked
  /// above invalid solutions, and joint limit violations will always be
  /// ranked last.
  void setQualityComparisonFunction(const QualityComparison& _func);

  /// Reset the quality comparison function to the default method.
  void resetQualityComparisonFunction();

  /// Get the Properties for this Analytical class
  Properties getAnalyticalProperties() const;

  /// Construct a mapping from the DOFs of getDofs() to their indices within
  /// the Node's list of dependent DOFs. This will be called immediately after
  /// the Analytical is constructed; this one call is sufficient as long as
  /// the DOFs of Analytical::getDofs() is not changed. However, if your
  /// Analytical is able to change the DOFs that it operates on, then you will
  /// need to call this function each time the DOFs have changed.
  void constructDofMap();

protected:

  /// This function will compute a gradient which utilizes the extra DOFs
  /// that go unused by the Analytical solution and then it will add the
  /// components of that gradient to the output parameter: grad.
  ///
  /// You can override this function to customize how the extra DOFs are used.
  /// The default behavior is to use a simple Jacobian Transpose method.
  ///
  /// The utilization flag will be PRE_ANALYTICAL if the function is being
  /// called before the Analytical solution is computed; it will be
  /// POST_ANALYTICAL if the function is being called after the Analytical
  /// solution is computed.
  virtual void addExtraDofGradient(
      Eigen::VectorXd& grad,
      const Eigen::Vector6d& error,
      ExtraDofUtilization utilization);

  /// Go through the mSolutions vector and tag entries with LIMIT_VIOLATED if
  /// any components of their configuration are outside of their position
  /// limits. This will NOT clear the LIMIT_VIOLATED flag from entries of
  /// mSolutions which are already tagged with it, even if they do not violate
  /// any limits.
  void checkSolutionJointLimits();

  /// Vector of solutions
  std::vector<Solution> mSolutions;

  /// Properties for this Analytical IK solver
  UniqueProperties mAnalyticalP;

private:

  /// This maps the DOFs provided by getDofs() to their index in the Node's
  /// list of dependent DOFs. This map is constructed by constructDofMap().
  std::vector<int> mDofMap;

  /// List of extra DOFs in the module which are not covered by the Analytical
  /// IK implementation. The index of each DOF is its dependency index in the
  /// JacobianNode (i.e. the column it applies to in the Node's Jacobian).
  std::vector<std::size_t> mExtraDofs;

  /// A cache for the valid solutions. The valid and invalid solution caches
  /// are kept separate so that they can each be sorted by quality
  /// individually. Valid solutions will always be at the top of mFinalResults
  /// even if their quality is scored below the invalid solutions.
  std::vector<Solution> mValidSolutionsCache;

  /// A cache for the out of reach solutions.
  std::vector<Solution> mOutOfReachCache;

  /// A cache for solutions that violate joint limits
  std::vector<Solution> mLimitViolationCache;

  /// A cache for storing the current configuration
  Eigen::VectorXd mConfigCache;

  /// A cache for storing the current configuration so that it can be restored
  /// later. This is different from mConfigCache which will not be used to
  /// restore the configuration.
  Eigen::VectorXd mRestoreConfigCache;

  /// A cache for storing the gradient for the extra DOFs
  Eigen::VectorXd mExtraDofGradCache;
};

//==============================================================================
/// The InverseKinematics::Objective Function is simply used to merge the
/// objective and null space objective functions that are being held by an
/// InverseKinematics module. This class is not meant to be extended or
/// instantiated by a user. Call InverseKinematics::resetProblem() to set
/// the objective of the module's Problem to an InverseKinematics::Objective.
class InverseKinematics::Objective final :
    public Function, public optimizer::Function
{
public:

  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(InverseKinematics::Objective)

  /// Constructor
  Objective(InverseKinematics* _ik);

  /// Virtual destructor
  virtual ~Objective() = default;

  // Documentation inherited
  optimizer::FunctionPtr clone(InverseKinematics* _newIK) const override;

  // Documentation inherited
  double eval(const Eigen::VectorXd& _x) const override;

  // Documentation inherited
  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) const override;

protected:

  /// Pointer to this Objective's IK module
  sub_ptr<InverseKinematics> mIK;

  /// Cache for the gradient of the Objective
  mutable Eigen::VectorXd mGradCache;

  /// Cache for the null space SVD
  mutable Eigen::JacobiSVD<math::Jacobian> mSVDCache;
  // TODO(JS): Need to define aligned operator new for this?

  /// Cache for the null space
  mutable Eigen::MatrixXd mNullSpaceCache;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
/// The InverseKinematics::Constraint Function is simply meant to be used to
/// merge the ErrorMethod and GradientMethod that are being held by an
/// InverseKinematics module. This class is not meant to be extended or
/// instantiated by a user. Call InverseKinematics::resetProblem() to set the
/// first equality constraint of the module's Problem to an
/// InverseKinematics::Constraint.
class InverseKinematics::Constraint final :
    public Function, public optimizer::Function
{
public:

  /// Constructor
  Constraint(InverseKinematics* _ik);

  /// Virtual constructor
  virtual ~Constraint() = default;

  // Documentation inherited
  optimizer::FunctionPtr clone(InverseKinematics* _newIK) const override;

  // Documentation inherited
  double eval(const Eigen::VectorXd& _x) const override;

  // Documentation inherited
  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) const override;

protected:

  /// Pointer to this Constraint's IK module
  sub_ptr<InverseKinematics> mIK;
};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/InverseKinematics.hpp"

#endif // DART_DYNAMICS_INVERSEKINEMATICS_HPP_
