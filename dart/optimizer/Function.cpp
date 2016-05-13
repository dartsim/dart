/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/optimizer/Function.hpp"

#include "dart/common/Console.hpp"

namespace dart {
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
void Function::evalGradient(const Eigen::VectorXd& /*_x*/,
                            Eigen::Map<Eigen::VectorXd> /*_grad*/)
{
  dtwarn << "Gradient is not provided by function named [" << mName
         << "]. Use gradient-free algorithm.\n";
}

//==============================================================================
void Function::evalGradient(const Eigen::VectorXd& _x, Eigen::VectorXd& _grad)
{
  Eigen::Map<Eigen::VectorXd> tmpGrad(_grad.data(), _grad.size());
  evalGradient(_x, tmpGrad);
}

//==============================================================================
void Function::evalHessian(
    const Eigen::VectorXd& /*_x*/,
    Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> /*_Hess*/)
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
}  // namespace dart
