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

#ifndef DART_OPTIMIZER_FUNCTION_H_
#define DART_OPTIMIZER_FUNCTION_H_

#include <vector>
#include <memory>

#include <Eigen/Dense>

namespace dart {
namespace optimizer {

/// \brief class Function
class Function
{
public:
  /// \brief Constructor
  explicit Function(const std::string& _name = "function");

  /// \brief Destructor
  virtual ~Function();

  /// \brief Set the name of this Function
  virtual void setName(const std::string& _newName);

  /// \brief Get the name of this Function
  const std::string& getName() const;

  /// \brief Evaluate and return the objective function at the point x
  virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x) = 0;

  /// \brief Evaluate and return the objective function at the point x
  virtual void evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd> _grad);

  /// \brief Evaluate and return the objective function at the point x
  virtual void evalHessian(
      Eigen::Map<const Eigen::VectorXd>& _x,
      Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess);

protected:
  /// \brief Name of this function
  std::string mName;

};

typedef std::shared_ptr<Function> FunctionPtr;

/// \brief class MultiFunction
class MultiFunction
{
public:
  /// \brief Constructor
  MultiFunction();

  /// \brief Destructor
  virtual ~MultiFunction();

  /// \brief Operator ()
  virtual void operator()(Eigen::Map<const Eigen::VectorXd>& _x,
                          Eigen::Map<Eigen::VectorXd>& _f,
                          Eigen::Map<Eigen::MatrixXd>& _grad) = 0;
};

}  // namespace optimizer
}  // namespace dart

#endif  // DART_OPTIMIZER_FUNCTION_H_

