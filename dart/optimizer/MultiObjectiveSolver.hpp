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

#ifndef DART_OPTIMIZER_MULTIOBJECTIVESOLVER_HPP_
#define DART_OPTIMIZER_MULTIOBJECTIVESOLVER_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "dart/optimizer/Population.hpp"

namespace dart {
namespace optimizer {

class MultiObjectiveProblem;

/// Abstract class that provides a common interface for different
/// multi-objective optimization solvers.
///
/// The different MultiObjectiveSolver implementations each use a different
/// Pareto-optimization library, which could lead to differences in performance
/// for various problem types. This base class allows the different
/// MultiObjectiveSolver implementations to be swapped out with each other
/// quickly and easily to help with testing, benchmarking, and experimentation.
class MultiObjectiveSolver
{
public:
  /// The MultiObjectiveSolver::Properties class contains Solver parameters that
  /// are common to all MultiObjectiveSolver types. Most (but not necessarily
  /// all) Solvers will make use of these parameters, and these parameters can
  /// be directly copied or transferred between all Solver types.
  struct Properties
  {
    /// Multi-objective optimization problem to be solved
    std::shared_ptr<MultiObjectiveProblem> mProblem;

    /// Number of populations
    std::size_t mNumPopulations;

    /// Number of decision vectors in one population
    std::size_t mPopulationSize;

    /// The maximum step size allowed for the Problem to be considered converged
    double mTolerance;

    /// How many iterations per evolution.
    std::size_t mIterationsPerEvolution;

    /// How many iterations between printing the Solver's progress to the
    /// terminal. Use 0 for no printing.
    std::size_t mIterationsPerPrint;

    /// Stream for printing the Solver's progress. Default is std::cout.
    std::ostream* mOutStream;

    /// Set to true if the final result should be printed to the terminal.
    bool mPrintFinalResult;

    /// Constructor
    Properties(
        std::shared_ptr<MultiObjectiveProblem> problem = nullptr,
        std::size_t numPopulations = 1u,
        std::size_t populationSize = 100u,
        double tolerance = 1e-9,
        std::size_t numMaxIterations = 500u,
        std::size_t iterationsPerPrint = 0u,
        std::ostream* ostream = &std::cout,
        bool printFinalResult = false);
  };

  /// Default constructor
  explicit MultiObjectiveSolver(const Properties& properties = Properties());

  /// Destructor
  virtual ~MultiObjectiveSolver() = default;

  /// Solve optimization problem
  virtual bool solve(std::size_t numEvolutions = 1u) = 0;

  /// Returns the type (implementation) of this Solver
  virtual std::string getType() const = 0;

  /// Creates an identical clone of this Solver
  virtual std::shared_ptr<MultiObjectiveSolver> clone() const = 0;

  /// Set the generic Properties of this Solver
  void setProperties(const Properties& properties);

  /// Get the generic Properties of this Solver
  const Properties& getSolverProperties() const;

  virtual void setProblem(std::shared_ptr<MultiObjectiveProblem> problem);

  /// Get nonlinear optimization problem
  std::shared_ptr<MultiObjectiveProblem> getProblem() const;

  /// \{ \name Population

  /// Sets the number of decision vectors in one population
  void setPopulationSize(std::size_t size);

  /// Return sthe number of decision vectors in one population
  std::size_t getPopulationSize() const;

  /// Sets the number of populations.
  void setNumPopulations(std::size_t size);

  /// Returns the number of populations.
  std::size_t getNumPopulations() const;

  /// Returns a population at \c index.
  const Population& getPopulation(std::size_t index) const;

  /// Returns all the populations.
  const std::vector<Population>& getPopulations() const;

  /// \}

  /// Sets the number of iterations per evolution
  void setNumIterationsPerEvolution(std::size_t maxIterations);

  /// Returns the number of iterations per evolution
  std::size_t getNumIterationsPerEvolution() const;

protected:
  /// Properties
  Properties mProperties;

  /// Populations
  std::vector<Population> mPopulations;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_MULTIOBJECTIVESOLVER_HPP_
