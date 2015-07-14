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

#ifndef DART_DYNAMICS_INVERSEKINEMATICS_H_
#define DART_DYNAMICS_INVERSEKINEMATICS_H_

#include <memory>

#include <Eigen/SVD>

#include "dart/common/sub_ptr.h"
#include "dart/common/Signal.h"
#include "dart/common/Subject.h"
#include "dart/math/Geometry.h"
#include "dart/optimizer/Solver.h"
#include "dart/optimizer/GradientDescentSolver.h"
#include "dart/optimizer/Problem.h"
#include "dart/optimizer/Function.h"
#include "dart/dynamics/SmartPointer.h"

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
/// InverseKinematics::GradientMethod.
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

  virtual ~InverseKinematics();

  bool solve(bool _resetConfiguration = false);

  bool solve(Eigen::VectorXd& config, bool _resetConfiguration = false);

  InverseKinematicsPtr clone(JacobianNode* _newEntity) const;

  /// This class should be inherited by optimizer::Function classes that have a
  /// dependency on the InverseKinematics module that they belong to. If you
  /// pass an InverseKinematics::Function into the Problem of an
  /// InverseKinematics module, then it will be properly cloned whenever the
  /// InverseKinematics module that it belongs to gets cloned. Any Function
  /// classes in the Problem that do not inherit InverseKinematics::Function
  /// will just be copied over by reference.
  class Function
  {
  public:
    virtual optimizer::FunctionPtr clone(InverseKinematics* _newIK) const = 0;

    virtual ~Function() = default;
  };

  /// ErrorMethod is a base class for different ways of computing the error of
  /// an InverseKinematics module
  class ErrorMethod : public common::Subject
  {
  public:

    typedef std::pair<Eigen::Vector6d, Eigen::Vector6d> Bounds;
    struct Properties
    {
      Properties(const Bounds& _bounds =
            Bounds(Eigen::Vector6d::Constant(-DefaultIKTolerance),
                   Eigen::Vector6d::Constant( DefaultIKTolerance)),

          double _errorClamp = DefaultIKErrorClamp,

          const Eigen::Vector6d& _errorWeights = Eigen::compose(
            Eigen::Vector3d::Constant(DefaultIKAngularWeight),
            Eigen::Vector3d::Constant(DefaultIKLinearWeight)));

      std::pair<Eigen::Vector6d, Eigen::Vector6d> mBounds;

      double mErrorLengthClamp;

      Eigen::Vector6d mErrorWeights;
    };

    ErrorMethod(InverseKinematics* _ik,
                const std::string& _methodName,
                const Properties& _properties = Properties());

    virtual ~ErrorMethod() = default;

    virtual std::unique_ptr<ErrorMethod> clone(
        InverseKinematics* _newIK) const = 0;

    virtual Eigen::Vector6d computeError() = 0;

    const Eigen::Vector6d& evalError(const Eigen::VectorXd& _q);

    const std::string& getMethodName() const;

    void setBounds(
        const Eigen::Vector6d& _lower =
            Eigen::Vector6d::Constant(-DefaultIKTolerance),
        const Eigen::Vector6d& _upper =
            Eigen::Vector6d::Constant( DefaultIKTolerance));

    void setBounds(const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds);

    const std::pair<Eigen::Vector6d, Eigen::Vector6d>& getBounds() const;

    void setAngularBounds(
        const Eigen::Vector3d& _lower =
            Eigen::Vector3d::Constant(-DefaultIKTolerance),
        const Eigen::Vector3d& _upper =
            Eigen::Vector3d::Constant( DefaultIKTolerance));

    void setAngularBounds(
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> getAngularBounds() const;

    void setLinearBounds(
        const Eigen::Vector3d& _lower =
            Eigen::Vector3d::Constant(-DefaultIKTolerance),
        const Eigen::Vector3d& _upper =
            Eigen::Vector3d::Constant( DefaultIKTolerance));

    void setLinearBounds(
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> getLinearBounds() const;

    void setErrorLengthClamp(double _clampSize = DefaultIKErrorClamp);

    double getErrorLengthClamp() const;

    void setErrorWeights(const Eigen::Vector6d& _weights);

    const Eigen::Vector6d& getErrorWeights() const;

    void setAngularErrorWeights(
        const Eigen::Vector3d& _weights =
          Eigen::Vector3d::Constant(DefaultIKAngularWeight));

    Eigen::Vector3d getAngularErrorWeights() const;

    void setLinearErrorWeights(
        const Eigen::Vector3d& _weights =
          Eigen::Vector3d::Constant(DefaultIKLinearWeight));

    Eigen::Vector3d getLinearErrorWeights() const;

    void clearCache();

  protected:

    common::sub_ptr<InverseKinematics> mIK;

    std::string mMethodName;

    Eigen::VectorXd mLastConfig;

    Eigen::Vector6d mLastError;

    Properties mProperties;

  };

  class TaskSpaceRegion : public ErrorMethod
  {
  public:

    explicit TaskSpaceRegion(InverseKinematics* _ik,
                             const Properties& _properties = Properties(),
                             bool _computeFromCenter = true);

    virtual ~TaskSpaceRegion() = default;

    virtual std::unique_ptr<ErrorMethod> clone(InverseKinematics* _newIK) const;

    virtual Eigen::Vector6d computeError() override;

    bool mComputeErrorFromCenter;
  };

  class GradientMethod : public common::Subject
  {
  public:

    GradientMethod(InverseKinematics* _ik,
                   const std::string& _methodName,
                   double _clamp = DefaultIKGradientComponentClamp);

    virtual ~GradientMethod() = default;

    virtual std::unique_ptr<GradientMethod> clone(
        InverseKinematics* _newIK) const = 0;

    virtual void computeGradient(const Eigen::Vector6d& _error,
                                 Eigen::VectorXd& _grad) = 0;

    void evalGradient(const Eigen::VectorXd& _q,
                      Eigen::Map<Eigen::VectorXd> _grad);

    const std::string& getMethodName() const;

    void clampGradient(Eigen::VectorXd& _grad) const;

    void setComponentWiseClamp(double _clamp = DefaultIKGradientComponentClamp);

    double getComponentWiseClamp() const;

    void applyWeights(Eigen::VectorXd& _grad) const;

    /// Set the weights that will be applied to each component of the gradient.
    /// If the number of components in _weights is smaller than the number of
    /// components in the gradient, then a weight of 1.0 will be applied to all
    /// components that are out of the range of _weights. Passing in an empty
    /// vector for _weights will effectively make all the gradient components
    /// unweighted.
    void setComponentWeights(const Eigen::VectorXd& _weights);

    const Eigen::VectorXd& getComponentWeights() const;

    void clearCache();

  protected:

    common::sub_ptr<InverseKinematics> mIK;

    std::string mMethodName;

    Eigen::VectorXd mLastConfig;

    Eigen::VectorXd mLastGradient;

    double mComponentWiseClamp;

    Eigen::VectorXd mComponentWeights;

  };

  class JacobianDLS : public GradientMethod
  {
  public:

    explicit JacobianDLS(InverseKinematics* _ik,
                         double _clamp = DefaultIKGradientComponentClamp,
                         double _damping = DefaultIKDLSCoefficient);

    virtual ~JacobianDLS() = default;

    virtual std::unique_ptr<GradientMethod> clone(
        InverseKinematics* _newIK) const;

    virtual void computeGradient(const Eigen::Vector6d& _error,
                                 Eigen::VectorXd& _grad) override;

    void setDampingCoefficient(double _damping = DefaultIKDLSCoefficient);

    double getDampingCoefficient() const;

  protected:

    double mDamping;
  };

  class JacobianTranspose : public GradientMethod
  {
  public:

    explicit JacobianTranspose(InverseKinematics* _ik,
                               double _clamp = DefaultIKGradientComponentClamp);

    virtual ~JacobianTranspose() = default;

    virtual std::unique_ptr<GradientMethod> clone(
        InverseKinematics* _newIK) const;

    virtual void computeGradient(const Eigen::Vector6d& _error,
                                 Eigen::VectorXd& _grad) override;
  };

  void setActive(bool _active=true);

  void setInactive();

  bool isActive() const;

  void setHierarchyLevel(size_t _level);

  size_t getHierarchyLevel() const;

  void useChain();

  void useWholeBody();

  template <class DegreeOfFreedomT>
  void setDofs(const std::vector<DegreeOfFreedomT*>& _dofs);

  void setDofs(const std::vector<size_t>& _dofs);

  const std::vector<size_t>& getDofs() const;

  const std::vector<int>& getDofMap() const;

  /// Set an objective function that should be minimized while satisfying the
  /// inverse kinematics constraint. Pass in a nullptr to remove the objective.
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

  template <class IKErrorMethod, typename... Args>
  IKErrorMethod& setErrorMethod(Args&&... args);

  ErrorMethod& getErrorMethod();

  const ErrorMethod& getErrorMethod() const;

  template <class IKGradientMethod, typename... Args>
  IKGradientMethod& setGradientMethod(Args&&... args);

  GradientMethod& getGradientMethod();

  const GradientMethod& getGradientMethod() const;

  const std::shared_ptr<optimizer::Problem>& getProblem();

  std::shared_ptr<const optimizer::Problem> getProblem() const;

  void resetProblem(bool _clearSeeds=false);

  /// Set the Solver that should be used by this IK module, and set it up with
  /// the Problem configured by this IK module
  void setSolver(const std::shared_ptr<optimizer::Solver>& _newSolver);

  const std::shared_ptr<optimizer::Solver>& getSolver();

  std::shared_ptr<const optimizer::Solver> getSolver() const;

  /// Inverse kinematics can be performed on any point within the body frame.
  /// The default point is the origin of the body frame. Use this function to
  /// change the point that will be used. _offset must represent the offset from
  /// the body origin of the desired point, expressed in coordinates of the body
  /// frame.
  void setOffset(const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero());

  /// Get the offset from the origin of the body frame that will be used when
  /// performing inverse kinematics. The offset will be expressed in the
  /// coordinates of the body frame.
  const Eigen::Vector3d& getOffset() const;

  /// This returns false if the offset for the inverse kinematics is a zero
  /// vector. Otherwise, it returns true. Use setOffset() to set the offset and
  /// getOffset() to get the offset.
  bool hasOffset() const;

  void setTarget(std::shared_ptr<SimpleFrame> _newTarget);

  std::shared_ptr<SimpleFrame> getTarget();

  std::shared_ptr<const SimpleFrame> getTarget() const;

  JacobianNode* getNode();

  const JacobianNode* getNode() const;

  JacobianNode* getAffiliation();

  const JacobianNode* getAffiliation() const;

  const math::Jacobian& computeJacobian() const;

  Eigen::VectorXd getConfiguration() const;

  void setConfiguration(const Eigen::VectorXd& _q);

  void clearCaches();

protected:

  /// The InverseKinematics::Objective Function is simply used to merge the
  /// objective and null space objective functions that are being held by an
  /// InverseKinematics module. This class is not meant to be extended or
  /// instantiated by a user. Call InverseKinematics::resetProblem() to set
  /// the objective of the module's Problem to an InverseKinematics::Objective.
  class Objective : public Function, public optimizer::Function
  {
  public:

    Objective(InverseKinematics* _ik);

    optimizer::FunctionPtr clone(InverseKinematics* _newIK) const override;

    double eval(const Eigen::VectorXd& _x) override;

    void evalGradient(const Eigen::VectorXd& _x,
                      Eigen::Map<Eigen::VectorXd> _grad) override;

    virtual ~Objective() = default;

  protected:

    sub_ptr<InverseKinematics> mIK;

    Eigen::VectorXd mGradCache;

    Eigen::JacobiSVD<math::Jacobian> mSVDCache;

    Eigen::MatrixXd mNullSpaceCache;
  };

  friend class Objective;

  /// The InverseKinematics::Constraint Function is simply meant to be used to
  /// merge the ErrorMethod and GradientMethod that are being held by an
  /// InverseKinematics module. This class is not meant to be extended or
  /// instantiated by a user. Call InverseKinematics::resetProblem() to set the
  /// first equality constraint of the module's Problem to an
  /// InverseKinematics::Constraint.
  class Constraint : public Function, public optimizer::Function
  {
  public:

    Constraint(InverseKinematics* _ik);

    optimizer::FunctionPtr clone(InverseKinematics* _newIK) const override;

    double eval(const Eigen::VectorXd& _x) override;

    void evalGradient(const Eigen::VectorXd& _x,
                      Eigen::Map<Eigen::VectorXd> _grad) override;

    virtual ~Constraint() = default;

  protected:

    sub_ptr<InverseKinematics> mIK;
  };

  friend class Constraint;

  /// Constructor that accepts a JacobianNode
  InverseKinematics(JacobianNode* _node);

  /// Gets called during construction
  void initialize();

  void resetTargetConnection();

  void resetEntityConnection();

  common::Connection mTargetConnection;

  common::Connection mEntityConnection;

  bool mActive;

  size_t mHierarchyLevel;

  /// A list of the DegreeOfFreedom Skeleton indices that will be used by this
  /// IK module
  std::vector<size_t> mDofs;

  /// When a Jacobian is computed for a JacobianNode, it will include a column
  /// for every DegreeOfFreedom that the node depends on. Given the columb index
  /// of one of those dependent coordinates, this map will return its location
  /// in the mDofs vector. A value of -1 means that it is not present in the
  /// mDofs vector and therefore should not be used when performing inverse
  /// kinematics.
  std::vector<int> mDofMap;

  std::shared_ptr<optimizer::Function> mObjective;

  std::shared_ptr<optimizer::Function> mNullSpaceObjective;

  /// The method that this IK module will use to compute errors
  std::unique_ptr<ErrorMethod> mErrorMethod;

  /// The method that this IK module will use to compute gradients
  std::unique_ptr<GradientMethod> mGradientMethod;

  std::shared_ptr<optimizer::Problem> mProblem;

  /// The solver that this IK module will use for iterative methods
  std::shared_ptr<optimizer::Solver> mSolver;

  Eigen::Vector3d mOffset;

  bool mHasOffset;

  std::shared_ptr<SimpleFrame> mTarget;

  sub_ptr<JacobianNode> mNode;

  mutable math::Jacobian mJacobian;

  const Eigen::VectorXd mEmptyVector = Eigen::VectorXd();
};

#include "dart/dynamics/detail/InverseKinematics.h"

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_INVERSEKINEMATICS_H_
