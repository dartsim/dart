/*
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_OPTIMIZER_MODULARFUNCTION_HPP_
#define DART_OPTIMIZER_MODULARFUNCTION_HPP_

#include <vector>
#include <memory>
#include <functional>

#include <Eigen/Dense>

#include "dart/optimizer/Function.hpp"

namespace dart {
namespace optimizer {

/// ModularFunction uses C++11 std::function to allow you to easily swap
/// out the cost function, gradient function, and Hessian function during
/// runtime for an optimizer::Function instance.
class ModularFunction : public Function
{
public:
  using CostFunction = std::function<double(const Eigen::VectorXd&)>;

  using GradientFunction = std::function<void(
      const Eigen::VectorXd&,
      Eigen::Map<Eigen::VectorXd>)>;

  using HessianFunction = std::function<void(
      const Eigen::VectorXd&,
      Eigen::Map<Eigen::VectorXd, Eigen::RowMajor>)>;

  /// \brief Constructor
  explicit ModularFunction(const std::string& _name = "modular_function");

  /// \brief Destructor
  virtual ~ModularFunction();

  /// \brief eval() will now call whatever CostFunction you set using
  /// setCostFunction()
  double eval(const Eigen::VectorXd& _x) override;

  /// \brief evalGradient() will now call whatever GradientFunction you set
  /// using setGradientFunction()
  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) override;

  /// \brief evalHessian() will now call whatever HessianFunction you set using
  /// setHessianFunction()
  void evalHessian(
      const Eigen::VectorXd& _x,
      Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess) override;

  /// \brief Set the function that gets called by eval()
  void setCostFunction(CostFunction _cost);

  /// \brief Replace the cost function with a constant-zero function. Passing in
  /// true will cause a warning to be printed out whenever eval() is called.
  void clearCostFunction(bool _printWarning = true);

  /// \brief Set the function that gets called by evalGradient()
  void setGradientFunction(GradientFunction _gradient);

  /// \brief Replace the gradient function with the default evalGradient() of
  /// the base Function class. A warning will be printed whenever evalGradient()
  /// gets called.
  void clearGradientFunction();

  /// \brief Set the function that gets called by evalHessian()
  void setHessianFunction(HessianFunction _hessian);

  /// \brief Replace the Hessian function with the default evalHessian() of the
  /// base Function class. A warning will be printed whenever evalHessian() gets
  /// called.
  void clearHessianFunction();

protected:
  /// Storage for the cost function
  CostFunction mCostFunction;

  /// Storage for the gradient function
  GradientFunction mGradientFunction;

  /// Storage for the Hessian function
  HessianFunction mHessianFunction;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_MODULARFUNCTION_HPP_

