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
#include "dart/math/Geometry.h"
#include "dart/optimizer/Solver.h"
#include "dart/optimizer/GradientDescentSolver.h"
#include "dart/optimizer/Problem.h"
#include "dart/optimizer/Function.h"
#include "dart/dynamics/SimpleFrame.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/JacobianEntity.h"

namespace dart {
namespace dynamics {

const double DefaultIKTolerance = 1e-6;
const double DefaultIKErrorClamp = 0.2;

/// The inverse kinematics class is templated to operate on either a BodyNode
/// or an EndEffector
class InverseKinematics : public common::Subject
{
public:

  /// Method is a base class for different InverseKinematics methods
  class ErrorMethod : public common::Subject
  {
  public:

    ErrorMethod(InverseKinematics* _ik,
                const std::string& _methodName);

    virtual ~ErrorMethod() = default;

    virtual Eigen::Vector6d computeError() = 0;

    const Eigen::Vector6d& computeError(const Eigen::VectorXd& _q);

    const std::string& getMethodName() const;

    void setBounds(
        const Eigen::Vector6d& _lower =
            Eigen::Vector6d::Constant(DefaultIKTolerance),
        const Eigen::Vector6d& _upper =
            Eigen::Vector6d::Constant(DefaultIKTolerance));

    void setBounds(const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds);

    const std::pair<Eigen::Vector6d, Eigen::Vector6d>& getBounds() const;

    void setAngularBounds(
        const Eigen::Vector3d& _lower =
            Eigen::Vector3d::Constant(DefaultIKTolerance),
        const Eigen::Vector3d& _upper =
            Eigen::Vector3d::Constant(DefaultIKTolerance));

    void setAngularBounds(
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> getAngularBounds() const;

    void setLinearBounds(
        const Eigen::Vector3d& _lower =
            Eigen::Vector3d::Constant(DefaultIKTolerance),
        const Eigen::Vector3d& _upper =
            Eigen::Vector3d::Constant(DefaultIKTolerance));

    void setLinearBounds(
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> getLinearBounds() const;

    void setErrorLengthClamp(double _clampSize = DefaultIKErrorClamp);

    double getErrorLengthClamp() const;

    void setErrorWeights(const Eigen::Vector6d& _weights);

    const Eigen::Vector6d& getErrorWeights() const;

    void setAngularErrorWeights(
        const Eigen::Vector3d& _weights = Eigen::Vector3d::Constant(0.2));

    Eigen::Vector3d getAngularErrorWeights() const;

    void setLinearErrorWeights(
        const Eigen::Vector3d& _weights = Eigen::Vector3d::Constant(1.0));

    Eigen::Vector3d getLinearErrorWeights() const;

    void clearCache();

  protected:

    common::sub_ptr<InverseKinematics> mIK;

    std::string mMethodName;

    Eigen::VectorXd mLastConfig;

    Eigen::Vector6d mLastError;

    std::pair<Eigen::Vector6d, Eigen::Vector6d> mBounds;

    double mErrorLengthClamp;

    Eigen::Vector6d mErrorWeights;

  };

  class EulerAngleXYZMethod : public ErrorMethod
  {
  public:

    explicit EulerAngleXYZMethod(InverseKinematics* _ik);

    virtual ~EulerAngleXYZMethod() = default;

    virtual Eigen::Vector6d computeError() override;

    bool mComputeErrorFromCenter;
  };

  class GradientMethod : public common::Subject
  {
  public:

    GradientMethod(InverseKinematics* _ik,
                   const std::string& _methodName);

    virtual ~GradientMethod() = default;

    virtual void computeGradient(const Eigen::Vector6d& _error,
                                 Eigen::VectorXd& _grad) = 0;

    void computeGradient(const Eigen::VectorXd& _q,
                         Eigen::Map<Eigen::VectorXd> _grad);

    const std::string& getMethodName() const;

    void clampGradient(Eigen::VectorXd& _grad) const;

    void setComponentWiseClamp(double _clamp = 0.2);

    double getComponentWiseClamp() const;

    void clearCache();

  protected:

    common::sub_ptr<InverseKinematics> mIK;

    std::string mMethodName;

    Eigen::VectorXd mLastConfig;

    Eigen::VectorXd mLastGradient;

    double mComponentWiseClamp;

  };

  class JacobianDLS : public GradientMethod
  {
  public:

    explicit JacobianDLS(InverseKinematics* _ik);

    virtual ~JacobianDLS() = default;

    virtual void computeGradient(const Eigen::Vector6d& _error,
                                 Eigen::VectorXd& _grad) override;

    void setDampingCoefficient(double _damping = 0.05);

    double getDampingCoefficient() const;

  protected:

    double mDamping;
  };

  class JacobianTranspose : public GradientMethod
  {
  public:

    explicit JacobianTranspose(InverseKinematics* _ik);

    virtual ~JacobianTranspose() = default;

    virtual void computeGradient(const Eigen::Vector6d& _error,
                                 Eigen::VectorXd& _grad) override;
  };

  void setActive(bool _active=true);

  void setInactive();

  bool isActive() const;

  void setHierarchyLevel(size_t _level);

  size_t getHierarchyLevel() const;

  void useLinkage();

  void useWholeBody();

  void useDofs(const std::vector<size_t>& _dofs);

  const std::vector<size_t>& getDofs() const;

  const std::vector<int>& getDofMap() const;

  /// Set an objective function that should be minimized while satisfying the
  /// inverse kinematics constraint. Pass in a nullptr to remove the objective.
  void setObjective(std::shared_ptr<optimizer::Function> _objective);

  /// Get the objective function for this IK module
  std::shared_ptr<optimizer::Function> getObjective();

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
  void setNullSpaceObjective(std::shared_ptr<optimizer::Function> _nsObjective);

  /// Get the null space objective for this IK module
  std::shared_ptr<optimizer::Function> getNullSpaceObjective();

  /// Get the null space objective for this IK module
  std::shared_ptr<const optimizer::Function> getNullSpaceObjective() const;

  double evalObjective(const Eigen::VectorXd& _q);

  void evalObjectiveGradient(const Eigen::VectorXd& _q,
                             Eigen::Map<Eigen::VectorXd> _grad);

  template <class IKErrorMethod>
  IKErrorMethod* setErrorMethod();

  ErrorMethod* getErrorMethod();

  const ErrorMethod* getErrorMethod() const;

  template <class IKGradientMethod>
  IKGradientMethod* setGradientMethod();

  GradientMethod* getGradientMethod();

  const GradientMethod* getGradientMethod() const;

  std::shared_ptr<optimizer::Problem> getProblem();

  std::shared_ptr<const optimizer::Problem> getProblem() const;

  void resetProblem(bool _clearSeeds=false);

  void setSolver(std::shared_ptr<optimizer::Solver> _newSolver);

  const std::shared_ptr<optimizer::Solver>& getSolver();

  std::shared_ptr<const optimizer::Solver> getSolver() const;

  void setTarget(std::shared_ptr<SimpleFrame> _newTarget);

  std::shared_ptr<SimpleFrame> getTarget();

  std::shared_ptr<const SimpleFrame> getTarget() const;

  JacobianEntity* getEntity();

  const JacobianEntity* getEntity() const;

  const math::Jacobian& computeJacobian() const;

  void setConfiguration(const Eigen::VectorXd& _q);

  void clearCaches();

  virtual ~InverseKinematics();

protected:

  InverseKinematics(JacobianEntity* _entity);

  void initialize();

  void resetTargetConnection();

  void resetEntityConnection();

  common::Connection mTargetConnection;

  common::Connection mEntityConnection;

  bool mActive;

  size_t mHierarchyLevel;

  std::vector<size_t> mDofs;

  std::vector<int> mDofMap;

  std::shared_ptr<optimizer::Function> mObjective;

  std::shared_ptr<optimizer::Function> mNullSpaceObjective;

  bool mUseNullSpace;

  Eigen::VectorXd mGradCache;

  Eigen::MatrixXd mNullspaceCache;

  Eigen::JacobiSVD<math::Jacobian> mSVDCache;

  std::shared_ptr<optimizer::ModularFunction> mOverallObjective;

  std::shared_ptr<optimizer::ModularFunction> mConstraint;

  /// The method that this IK module will use to compute errors
  std::unique_ptr<ErrorMethod> mErrorMethod;

  /// The method that this IK module will use to compute gradients
  std::unique_ptr<GradientMethod> mGradientMethod;

  std::shared_ptr<optimizer::Problem> mProblem;

  /// The solver that this IK module will use for iterative methods
  std::shared_ptr<optimizer::Solver> mSolver;


  std::shared_ptr<SimpleFrame> mTarget;


  sub_ptr<JacobianEntity> mEntity;


  mutable math::Jacobian mJacobian;
};

//==============================================================================
template <class IKErrorMethod>
IKErrorMethod* InverseKinematics::setErrorMethod()
{
  IKErrorMethod* newMethod = new IKErrorMethod(this);
  mErrorMethod = std::move(std::unique_ptr<IKErrorMethod>(newMethod));
  return newMethod;
}

//==============================================================================
template <class IKGradientMethod>
IKGradientMethod* InverseKinematics::setGradientMethod()
{
  IKGradientMethod* newMethod = new IKGradientMethod(this);
  mGradientMethod = std::move(std::unique_ptr<IKGradientMethod>(newMethod));
  return newMethod;
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_INVERSEKINEMATICS_H_
