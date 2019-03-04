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

#ifndef DART_OPTIMIZER_MULTIOBJECTIVEPROBLEM_HPP_
#define DART_OPTIMIZER_MULTIOBJECTIVEPROBLEM_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "dart/optimizer/Function.hpp"

namespace dart {
namespace optimizer {

class MultiObjectiveProblem
{
public:
  /// Constructor
  explicit MultiObjectiveProblem(std::size_t dim, std::size_t integerDim = 0u);

  /// Destructor
  virtual ~MultiObjectiveProblem() = default;

  /// \{ \name Solution

  /// Sets the dimension of the solution.
  ///
  /// The element type of the solution can be either \c double or \c int. In the
  /// case that the solution partially contains integers, we assume the integer
  /// part takes place the tail of the solution, and the size of the integer
  /// part is \p integerDim. For example, a solution vector can be
  /// <tt>[floating1, floating2, ..., int1, int2]</tt>.
  ///
  /// Note that the \p dim represents the whole dimension of the solution so
  /// that the dimension of floating-point part is to be (\p dim -
  /// \p integerDim).
  ///
  /// By default, we assume the solution is homogeneously floating-point type (
  /// i.e., \p integerDim is zero).
  ///
  /// \param[in] dim Total dimension of the solution.
  /// \param[in] integerDim The dimension of integer part in the solution.
  virtual void setSolutionDimension(
      std::size_t dim, std::size_t integerDim = 0u);

  /// Returns dimension of the solution
  virtual std::size_t getSolutionDimension() const;

  /// Returns dimension of the floating-point part of the solution
  std::size_t getDoubleDimension() const;

  /// Sets dimension of the integers in the decision vector
  virtual void setIntegerDimension(std::size_t dim);

  /// Returns dimension of the integers in the decision vector
  virtual std::size_t getIntegerDimension() const;

  /// Sets lower bounds for optimization parameters
  void setLowerBounds(const Eigen::VectorXd& lb);

  /// Returns lower bounds for optimization parameters
  const Eigen::VectorXd& getLowerBounds() const;

  /// Sets upper bounds for optimization parameters
  void setUpperBounds(const Eigen::VectorXd& ub);

  /// Returns upper bounds for optimization parameters
  const Eigen::VectorXd& getUpperBounds() const;

  /// \}

  /// \{ \name Objectives

  /// Returns the total dimension of objective functions.
  virtual std::size_t getObjectiveDimension() const = 0;

  /// \}

  /// \{ \name Equality Constraints

  /// Returns the total dimension of equality constraints.
  virtual std::size_t getEqConstraintDimension() const;

  /// \}

  /// \{ \name Inequality Constraints

  /// Returns the total dimension of inequality constraints.
  virtual std::size_t getIneqConstraintDimension() const;

  /// \}

  /// \{ \name Evaluations

  /// Evaluates objectives
  virtual Eigen::VectorXd evaluateObjectives(
      const Eigen::VectorXd& x) const = 0;

  /// Evaluates equality constraints
  virtual Eigen::VectorXd evaluateEqConstraints(const Eigen::VectorXd& x) const;

  /// Evaluates inequality constraints
  virtual Eigen::VectorXd evaluateIneqConstraints(
      const Eigen::VectorXd& x) const;

  /// Return dimension of fitness
  std::size_t getFitnessDimension() const;

  /// Evaluates fitness, which is [objectives, equality constraints, inequality
  /// constraints].
  Eigen::VectorXd evaluateFitness(const Eigen::VectorXd& x) const;

  /// \}

  /// Prints information of this class to a stream.
  virtual std::ostream& print(std::ostream& os) const;

protected:
  /// Dimension of the decision vector (or optimization parameters)
  std::size_t mDimension;

  /// Dimension of integer in the decision vector. The integers are placed in
  /// the tail of the decision vector.
  std::size_t mIntegerDimension;

  /// Lower bounds for optimization parameters
  Eigen::VectorXd mLowerBounds;

  /// Upper bounds for optimization parameters
  Eigen::VectorXd mUpperBounds;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_MULTIOBJECTIVEPROBLEM_HPP_
