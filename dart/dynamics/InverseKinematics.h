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

#include "dart/common/sub_ptr.h"
#include "dart/optimizer/Solver.h"
#include "dart/optimizer/Function.h"
#include "dart/dynamics/SimpleFrame.h"

namespace dart {
namespace dynamics {

/// This enumeration will be used by the IK Method to determine which joints
/// are allowed to be used when solving the IK problem.
enum class IKMode : int
{
  INACTIVE = 0, ///< Do not actually attempt to solve the IK problem
  LINKAGE,      ///< Only use the largest unbranching linkage that leads up to the Entity
  WHOLEBODY     ///< Use all relevant joints in the body
};

/// The inverse kinematics class is templated to operate on either a BodyNode
/// or an EndEffector
template <class JacobianEntity>
class InverseKinematics : public common::Subject
{
public:

  /// Method is a base class for different InverseKinematics methods
  class ErrorMethod : public common::Subject
  {
  public:
    /// Constructor which should be called via the templated
    /// InverseKinematics::setMethod function.
    ErrorMethod(InverseKinematics<JacobianEntity>* _ik,
                const std::string& _name);

    virtual Eigen::Vector6d computeError(Eigen::Map<const Eigen::VectorXd>& _x);

  protected:

    common::sub_ptr< InverseKinematics<JacobianEntity> > mIK;

    std::string mMethodName;

  };

  class GradientMethod : public common::Subject
  {
  public:

    GradientMethod(InverseKinematics<JacobianEntity>* _ik,
                   const std::string& _name);

    virtual void computeGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                                 Eigen::Map<Eigen::VectorXd> _grad);

  protected:

    common::sub_ptr< InverseKinematics<JacobianEntity> > mIK;

    std::string mMethodName;

  };

  class JacobianDLS : public GradientMethod
  {

  };

  class JacobianTranspose : public GradientMethod
  {

  };



  /// Set the mode that this InverseKinematics Solver should use
  void setMode(IKMode _mode);

  /// Check the mode of this InverseKinematics Solver
  IKMode getMode() const;

  template <class IKErrorMethod>
  typename IKErrorMethod* setErrorMethod();

  ErrorMethod* getErrorMethod();

  const ErrorMethod* getErrorMethod() const;

  template <class IKGradientMethod>
  typename IKGradientMethod* setGradientMethod();

  GradientMethod* getGradientMethod();

  const GradientMethod* getGradientMethod() const;

  void setSolver(std::shared_ptr<optimizer::Solver> _newSolver);

  std::shared_ptr<optimizer::Solver> getSolver();

  std::shared_ptr<const optimizer::Solver> getSolver() const;

  void setTarget(std::shared_ptr<SimpleFrame> _newTarget);

  std::shared_ptr<SimpleFrame> getTarget();

  std::shared_ptr<const SimpleFrame> getTarget() const;

  void setEntity(JacobianEntity* _newEntity);

  JacobianEntity* getEntity();

  const JacobianEntity* getEntity() const;


protected:

  /// The solver that this IK module will use for iterative methods
  std::shared_ptr<optimizer::Solver> mSolver;

  /// The method that this IK module will use to compute errors
  std::unique_ptr<ErrorMethod> mErrorMethod;

  /// The method that this IK module will use to compute gradients
  std::unique_ptr<GradientMethod> mGradientMethod;


  sub_ptr<JacobianEntity> mEntity;


  std::shared_ptr<SimpleFrame> mTarget;



};

}
}


#endif // DART_DYNAMICS_INVERSEKINEMATICS_H_
