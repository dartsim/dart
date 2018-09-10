/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_OPTIMIZER_POPULATION_HPP_
#define DART_OPTIMIZER_POPULATION_HPP_

#include <iostream>
#include <memory>
#include <random>

#include <Eigen/Dense>

#include "dart/optimizer/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimizer {

class MultiObjectiveProblem;

class Population
{
public:
  /// Constructor
  Population(
      std::size_t solutionDimension = 0u,
      std::size_t fitnessDimension = 0u,
      std::size_t numSolutions = 0u);

  /// Constructor
  Population(
      const MultiObjectiveProblem& problem, std::size_t numSolutions = 0u);

  /// Adds the solution and its fitness at the back of the population.
  void pushBack(const Eigen::VectorXd& x, const Eigen::VectorXd& f);

  /// Sets the solution and its fitness at \c index in the population.
  void set(
      std::size_t index, const Eigen::VectorXd& x, const Eigen::VectorXd& f);

  void setRandom(const MultiObjectiveProblem& prob, std::size_t numSolutions);

  void setRandom(const MultiObjectiveProblem& prob);

  /// Returns the size of this population
  std::size_t getNumSolutions() const;

  /// Returns a solution at \c index.
  Eigen::VectorXd getSolution(std::size_t index) const;

  /// Returns a fitness vector at \c index.
  Eigen::VectorXd getFitness(std::size_t index) const;

  /// Prints information of this class to a stream.
  std::ostream& print(std::ostream& os) const;

protected:
  void resize(
      std::size_t fitnessDimension,
      std::size_t numSolutions,
      std::size_t solutionDimension);

  std::size_t mNumSolutions;

  std::size_t mSolutionDimension;

  std::size_t mFitnessDimension;

  /// Solution vectors. A column represents a solution.
  Eigen::MatrixXd mSolutions;

  /// Fitness vectors. A column represents a fitness.
  Eigen::MatrixXd mFitnesses;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_POPULATION_HPP_
