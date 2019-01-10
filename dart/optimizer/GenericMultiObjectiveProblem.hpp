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

#ifndef DART_OPTIMIZER_GENERICMULTIOBJECTIVEPROBLEM_HPP_
#define DART_OPTIMIZER_GENERICMULTIOBJECTIVEPROBLEM_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimizer {

class GenericMultiObjectiveProblem : public MultiObjectiveProblem
{
public:
  /// Constructor
  explicit GenericMultiObjectiveProblem(
      std::size_t dim, std::size_t integerDim = 0u);

  /// Destructor
  ~GenericMultiObjectiveProblem() override = default;

  /// \{ \name Objectives

  // Documentation inherited.
  std::size_t getObjectiveDimension() const override;

  /// Sets objective functions to be minimized.
  void setObjectiveFunctions(const std::vector<FunctionPtr>& objectives);

  /// Adds a minimum objective function
  void addObjectiveFunction(FunctionPtr objective);

  /// Returns the number objective functions.
  std::size_t getNumObjectiveFunctions() const;

  /// Returns objective functions
  const std::vector<FunctionPtr>& getObjectiveFunctions() const;

  /// Removes an objective function
  void removeObjectiveFunction(FunctionPtr function);

  /// Removes all objective functions
  void removeAllObjectiveFunctions();

  /// \}

  /// \{ \name Equality Constraints

  // Documentation inherited.
  std::size_t getEqConstraintDimension() const override;

  /// Adds equality constraint
  void addEqConstraintFunction(FunctionPtr eqConst);

  /// Returns number of equality constraints
  std::size_t getNumEqualityConstraintFunctions() const;

  /// Returns equality constraint
  FunctionPtr getEqConstraintFunction(std::size_t index) const;

  /// Removes equality constraint
  void removeEqConstraintFunction(FunctionPtr eqConst);

  /// Removes all equality constraints
  void removeAllEqConstraintFunctions();

  /// \}

  /// \{ \name Inequality Constraints

  // Documentation inherited.
  std::size_t getIneqConstraintDimension() const override;

  /// Adds inequality constraint. Inequality constraints must evaluate
  /// to LESS THAN or equal to zero (within some tolerance) to be satisfied.
  void addIneqConstraintFunction(FunctionPtr ineqConst);

  /// Returns number of inequality constraints
  std::size_t getNumIneqConstraintFunctions() const;

  /// Returns inequality constraint
  FunctionPtr getIneqConstraintFunction(std::size_t index) const;

  /// Removes inequality constraint
  void removeIneqConstraintFunction(FunctionPtr ineqConst);

  /// Removes all inequality constraints
  void removeAllIneqConstraintFunctions();

  /// \}

  /// \{ \name Evaluations

  /// Evaluates objectives
  Eigen::VectorXd evaluateObjectives(const Eigen::VectorXd& x) const override;

  /// Evaluates equality constraints
  Eigen::VectorXd evaluateEqConstraints(
      const Eigen::VectorXd& x) const override;

  /// Evaluates inequality constraints
  Eigen::VectorXd evaluateIneqConstraints(
      const Eigen::VectorXd& x) const override;

  /// Return dimension of fitness
  std::size_t getFitnessDimension() const;

  /// Evaluates fitness, which is [objectives, equality constraints, inequality
  /// constraints].
  Eigen::VectorXd evaluateFitness(const Eigen::VectorXd& x) const;

  /// \}

protected:
  /// Objective functions
  std::vector<FunctionPtr> mObjectiveFunctions;

  /// Equality constraint functions
  std::vector<FunctionPtr> mEqConstraintFunctions;

  /// Inequality constraint functions
  std::vector<FunctionPtr> mIneqConstraintFunctions;

private:
  /// Cache for objective dimension
  std::size_t mObjectiveDimension;

  /// Cache for equality constraint dimension
  std::size_t mEqConstraintDimension;

  /// Cache for inequality constraint dimension
  std::size_t mIneqConstraintDimension;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_GENERICMULTIOBJECTIVEPROBLEM_HPP_
