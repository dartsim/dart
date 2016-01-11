/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/optimizer/Function.h"

#include "dart/common/Console.h"

namespace kido {
namespace optimizer {

//==============================================================================
Function::Function(const std::string& _name)
  : mName(_name)
{
  // Do nothing
}

//==============================================================================
Function::~Function()
{
  // Do nothing
}

//==============================================================================
void Function::setName(const std::string& _newName)
{
  mName = _newName;
}

//==============================================================================
const std::string& Function::getName() const
{
  return mName;
}

//==============================================================================
double Function::eval(Eigen::Map<const Eigen::VectorXd>& _x)
{
  dterr << "[Function::eval(Eigen::Map<const Eigen::VectorXd>&)] Using a "
        << "deprecated member function! Please override "
        << "Function::eval(const Eigen::VectorXd&) and use that instead!\n";
  assert(false);
  return 0.0;
}

//==============================================================================
double Function::eval(const Eigen::VectorXd& _x)
{
  // TODO(MXG): This is for backwards compatibility. This function should be
  // made pure abstract with the next major version-up. We suppress the
  // deprecated-warnings until then (see #544).
  Eigen::Map<const Eigen::VectorXd> temp(_x.data(), _x.size());

  KIDO_SUPPRESS_DEPRECATED_BEGIN
  return eval(temp);
  KIDO_SUPPRESS_DEPRECATED_END
}

//==============================================================================
void Function::evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd> _grad)
{
  // TODO(MXG): This content should be moved into the other evalGradient
  // function and this version of the function should be removed during the next
  // major version-up
  dtwarn << "Gradient is not provided by function named [" << mName
         << "]. Use gradient-free algorithm.\n";
}

//==============================================================================
void Function::evalGradient(const Eigen::VectorXd& _x,
                            Eigen::Map<Eigen::VectorXd> _grad)
{
  // TODO(MXG): This is for backwards compatibility. We suppress the
  // deprecated-warnings until then (see #544).
  Eigen::Map<const Eigen::VectorXd> temp(_x.data(), _x.size());

  KIDO_SUPPRESS_DEPRECATED_BEGIN
  evalGradient(temp, _grad);
  KIDO_SUPPRESS_DEPRECATED_END
}

//==============================================================================
void Function::evalGradient(const Eigen::VectorXd& _x, Eigen::VectorXd& _grad)
{
  Eigen::Map<Eigen::VectorXd> tmpGrad(_grad.data(), _grad.size());
  evalGradient(_x, tmpGrad);
}

//==============================================================================
void Function::evalHessian(const Eigen::VectorXd& _x,
                           Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess)
{
  dterr << "Hessian is not provided by funciton named [" << mName
        << "]. Use Hessian-free algorithm.\n";
}

//==============================================================================
ModularFunction::ModularFunction(const std::string& _name)
  : Function(_name)
{
  clearCostFunction();
  clearGradientFunction();
  clearHessianFunction();
}

//==============================================================================
ModularFunction::~ModularFunction()
{
  // Do nothing
}

//==============================================================================
double ModularFunction::eval(const Eigen::VectorXd& _x)
{
  return mCostFunction(_x);
}

//==============================================================================
void ModularFunction::evalGradient(const Eigen::VectorXd& _x,
                                   Eigen::Map<Eigen::VectorXd> _grad)
{
  mGradientFunction(_x, _grad);
}

//==============================================================================
void ModularFunction::evalHessian(const Eigen::VectorXd& _x,
    Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess)
{
  mHessianFunction(_x, _Hess);
}

//==============================================================================
void ModularFunction::setCostFunction(CostFunction _cost)
{
  mCostFunction = _cost;
}

//==============================================================================
void ModularFunction::clearCostFunction(bool _printWarning)
{
  mCostFunction = [=](const Eigen::VectorXd&)
  {
    if(_printWarning)
    {
      dterr << "A cost function has not yet been assigned to the ModularFunction "
            << "named [" << this->mName << "]. Returning 0.0\n";
    }
    return 0;
  };
}

//==============================================================================
void ModularFunction::setGradientFunction(GradientFunction _gradient)
{
  mGradientFunction = _gradient;
}

//==============================================================================
void ModularFunction::clearGradientFunction()
{
  mGradientFunction = [&](const Eigen::VectorXd& _x,
                          Eigen::Map<Eigen::VectorXd> _grad)
  {
    this->Function::evalGradient(_x, _grad);
  };
}

//==============================================================================
void ModularFunction::setHessianFunction(HessianFunction _hessian)
{
  mHessianFunction = _hessian;
}

//==============================================================================
void ModularFunction::clearHessianFunction()
{
  mHessianFunction = [&](const Eigen::VectorXd& _x,
                         Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess)
  {
    this->Function::evalHessian(_x, _Hess);
  };
}

//==============================================================================
NullFunction::NullFunction(const std::string& _name)
  : Function(_name)
{
  // Do nothing
}

//==============================================================================
NullFunction::~NullFunction()
{
  // Do nothing
}

//==============================================================================
double NullFunction::eval(const Eigen::VectorXd&)
{
  return 0;
}

//==============================================================================
void NullFunction::evalGradient(const Eigen::VectorXd& _x,
                                Eigen::Map<Eigen::VectorXd> _grad)
{
  _grad.resize(_x.size());
  _grad.setZero();
}

//==============================================================================
void NullFunction::evalHessian(
    const Eigen::VectorXd& _x,
    Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess)
{
  _Hess.resize(pow(_x.size(),2));
  _Hess.setZero();
}

//==============================================================================
MultiFunction::MultiFunction()
{
  // Do nothing
}

//==============================================================================
MultiFunction::~MultiFunction()
{
  // Do nothing
}

}  // namespace optimizer
}  // namespace kido
