/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_OPTIMIZER_FUNCTION_HPP_
#define DART_OPTIMIZER_FUNCTION_HPP_

#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <vector>

namespace dart {
namespace optimizer {

class Function
{
public:
  /// Constructor
  explicit Function(const std::string& _name = "function");

  /// Destructor
  virtual ~Function();

  /// Sets the name of this Function
  virtual void setName(const std::string& newName);

  /// Returns the name of this Function
  const std::string& getName() const;

  /// Evaluates and returns the objective function at the point x
  virtual double eval(const Eigen::VectorXd& x) = 0;

  /// Evaluates and returns the objective function at the point x
  virtual void evalGradient(
      const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad);

  /// Evaluates and return the objective function at the point x.
  ///
  /// If you have a raw array that the gradient will be passed in, then use
  /// evalGradient(const Eigen::VectorXd&, Eigen::Map<Eigen::VectorXd>)
  /// for better performance.
  void evalGradient(const Eigen::VectorXd& _x, Eigen::VectorXd& _grad);

  /// Evaluates and return the objective function at the point x
  virtual void evalHessian(
      const Eigen::VectorXd& _x,
      Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess);

protected:
  /// Name of this function
  std::string mName;
};

typedef std::shared_ptr<Function> FunctionPtr;
typedef std::unique_ptr<Function> UniqueFunctionPtr;

typedef std::function<double(const Eigen::VectorXd&)> CostFunction;

typedef std::function<void(const Eigen::VectorXd&, Eigen::Map<Eigen::VectorXd>)>
    GradientFunction;

typedef std::function<void(
    const Eigen::VectorXd&, Eigen::Map<Eigen::VectorXd, Eigen::RowMajor>)>
    HessianFunction;

/// ModularFunction uses C++11 std::function to allow you to easily swap
/// out the cost function, gradient function, and Hessian function during
/// runtime for an optimizer::Function instance.
class ModularFunction : public Function
{
public:
  /// Constructor
  explicit ModularFunction(const std::string& _name = "modular_function");

  /// Destructor
  ~ModularFunction() override;

  /// eval() will now call whatever CostFunction you set using
  /// setCostFunction()
  double eval(const Eigen::VectorXd& _x) override;

  /// evalGradient() will now call whatever GradientFunction you set
  /// using setGradientFunction()
  void evalGradient(
      const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) override;

  /// evalHessian() will now call whatever HessianFunction you set using
  /// setHessianFunction()
  void evalHessian(
      const Eigen::VectorXd& _x,
      Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess) override;

  /// Set the function that gets called by eval()
  void setCostFunction(CostFunction _cost);

  /// Replace the cost function with a constant-zero function. Passing in
  /// true will cause a warning to be printed out whenever eval() is called.
  void clearCostFunction(bool _printWarning = true);

  /// Set the function that gets called by evalGradient()
  void setGradientFunction(GradientFunction _gradient);

  /// Replace the gradient function with the default evalGradient() of
  /// the base Function class. A warning will be printed whenever evalGradient()
  /// gets called.
  void clearGradientFunction();

  /// Set the function that gets called by evalHessian()
  void setHessianFunction(HessianFunction _hessian);

  /// Replace the Hessian function with the default evalHessian() of the
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

/// NullFunction is a constant-zero Function
class NullFunction : public Function
{
public:
  /// Constructor
  explicit NullFunction(const std::string& _name = "null_function");

  /// Destructor
  ~NullFunction() override;

  /// eval() will always return exactly zero
  double eval(const Eigen::VectorXd&) override;

  /// evalGradient will always set _grad to a zero vector that
  /// matches the dimensionality of _x
  void evalGradient(
      const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) override;

  /// evalHessian() will always set _Hess to a zero matrix that matches
  /// the dimensionality of _x
  void evalHessian(
      const Eigen::VectorXd& _x,
      Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess) override;
};

/// class MultiFunction
class MultiFunction
{
public:
  /// Constructor
  MultiFunction();

  /// Destructor
  virtual ~MultiFunction();

  /// Operator ()
  virtual void operator()(
      const Eigen::VectorXd& _x,
      Eigen::Map<Eigen::VectorXd>& _f,
      Eigen::Map<Eigen::MatrixXd>& _grad)
      = 0;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_FUNCTION_HPP_
