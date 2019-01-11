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

#ifndef DART_OPTIMIZER_SOLVER_HPP_
#define DART_OPTIMIZER_SOLVER_HPP_

#include <iostream>
#include <memory>

#include <Eigen/Dense>

namespace dart {
namespace optimizer {

class Problem;

/// Abstract class that provides a common interface for different Solvers. The
/// different Solver implementations each use a different nonlinear problem
/// solving library, which could lead to differences in performance for various
/// problem types. This base class allows the different Solver implementations
/// to be swapped out with each other quickly and easily to help with testing,
/// benchmarking, and experimentation.
class Solver
{
public:

  /// The Solver::Properties class contains Solver parameters that are common
  /// to all Solver types. Most (but not necessarily all) Solvers will make use
  /// of these parameters, and these parameters can be directly copied or
  /// transferred between all Solver types.
  struct Properties
  {
    /// Nonlinear optimization Problem to be solved
    std::shared_ptr<Problem> mProblem;

    /// The relative tolerance on the optimization parameters. For example, the
    /// distance between the last parameters and the current parameters to be
    /// considered converged.
    double mTolerance;

    /// The maximum number of iterations that the solver should use. Use 0 for
    /// infinite iterations.
    std::size_t mNumMaxIterations;

    /// How many iterations between printing the Solver's progress to the
    /// terminal. Use 0 for no printing.
    std::size_t mIterationsPerPrint;

    /// Stream for printing the Solver's progress. Default is std::cout.
    std::ostream* mOutStream;

    /// Set to true if the final result should be printed to the terminal.
    bool mPrintFinalResult;

    /// Publish the results of the optimization to a file. Leave this string
    /// empty to avoid publishing anything.
    std::string mResultFile;

    Properties(std::shared_ptr<Problem> _problem = nullptr,
               double _tolerance = 1e-9,
               std::size_t _numMaxIterations = 500,
               std::size_t _iterationsPerPrint = 0,
               std::ostream* _ostream = &std::cout,
               bool _printFinalResult = false,
               const std::string& _resultFile = "");
  };

  /// Default Constructor
  explicit Solver(const Properties& _properties = Properties());

  /// Alternative Constructor
  explicit Solver(std::shared_ptr<Problem> _problem);

  /// \brief Destructor
  virtual ~Solver() = default;

  /// Solve optimization problem
  virtual bool solve() = 0;

  /// Get the type (implementation) of this Solver
  virtual std::string getType() const = 0;

  /// Create an identical clone of this Solver
  virtual std::shared_ptr<Solver> clone() const = 0;

  /// Set the generic Properties of this Solver
  void setProperties(const Properties& _properties);

  /// Get the generic Properties of this Solver
  const Properties& getSolverProperties() const;

  /// Copy the generic Properties of another Solver
  void copy(const Solver& _otherSolver);

  /// Copy the generic Properties of another Solver
  Solver& operator=(const Solver& _otherSolver);

  /// Set the nonlinear optimization problem
  virtual void setProblem(std::shared_ptr<Problem> _newProblem);

  /// Get nonlinear optimization problem
  std::shared_ptr<Problem> getProblem() const;

  /// Set the maximum step size allowed for the Problem to be considered
  /// converged
  virtual void setTolerance(double _newTolerance);

  /// Get the maximum step size allowed for the Problem to be considered
  /// converged
  double getTolerance() const;

  /// Set the maximum number of iterations that the Solver should use
  virtual void setNumMaxIterations(std::size_t _newMax);

  /// Get the maximum number of iterations that the Solver should use
  std::size_t getNumMaxIterations() const;

  /// Set the number of iterations that should pass between printing progress to
  /// the terminal. Use 0 for no printing.
  virtual void setIterationsPerPrint(std::size_t _newRatio);

  /// Get the number of iterations that should pass between printing progress to
  /// the terminal. A value of 0 means there will be no printing.
  std::size_t getIterationsPerPrint() const;

  /// Set the output stream that prints the Solver's progress.
  virtual void setOutStream(std::ostream* _os);

  /// Get the output stream that prints the Solver's progress.
  std::ostream* getOutStream() const;

  /// Set to true if the final result should be printed to the terminal
  virtual void setPrintFinalResult(bool _print);

  /// Returns true if the final result should be printed to the terminal
  bool getPrintFinalResult() const;

  /// Set the name of the file that results should be printed to. Use an empty
  /// string to indicate that results should not be printed to a file.
  virtual void setResultFileName(const std::string& _resultFile);

  /// Get the name of the file that results should be printed to. An empty
  /// string indicates that results should not be printed to a file.
  const std::string& getResultFileName() const;

protected:

  Properties mProperties;

};

}  // namespace optimizer
}  // namespace dart

#endif  // DART_OPTIMIZER_SOLVER_HPP_
