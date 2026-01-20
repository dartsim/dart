/*
 * Copyright (c) 2011, The DART development contributors
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

#ifndef DART_MATH_LCP_LCPTYPES_HPP_
#define DART_MATH_LCP_LCPTYPES_HPP_

#include <dart/Export.hpp>

#include <Eigen/Core>

#include <limits>
#include <string>

namespace dart {
namespace math {

//==============================================================================
/// Solver exit status codes
enum class LcpSolverStatus
{
  /// Solution found successfully
  Success = 0,

  /// Failed to find a solution
  Failed = 1,

  /// Maximum iterations reached
  MaxIterations = 2,

  /// Numerical issues (NaN, Inf, etc.)
  NumericalError = 3,

  /// Invalid problem formulation (e.g., inconsistent dimensions)
  InvalidProblem = 4,

  /// Solution is degenerate
  Degenerate = 5,

  /// Not yet solved
  NotSolved = 6
};

//==============================================================================
/// Convert solver status to string
///
/// \param[in] status The solver status to convert
/// \return String representation of the status
DART_API std::string toString(LcpSolverStatus status);

//==============================================================================
/// Solver result structure containing solution status and statistics
struct DART_API LcpResult
{
  /// Exit status
  LcpSolverStatus status{LcpSolverStatus::NotSolved};

  /// Number of iterations performed
  int iterations{0};

  /// Final residual norm
  double residual{std::numeric_limits<double>::quiet_NaN()};

  /// Complementarity/KKT violation (infinity norm).
  double complementarity{std::numeric_limits<double>::quiet_NaN()};

  /// Whether the solution was validated
  bool validated{false};

  /// Additional solver-specific information
  std::string message;

  /// Default constructor
  LcpResult() = default;

  /// Constructor with status
  ///
  /// \param[in] status_ Initial solver status
  explicit LcpResult(LcpSolverStatus status_) : status(status_) {}

  /// Check if solution was successful
  ///
  /// \return True if status is Success, false otherwise
  bool succeeded() const
  {
    return status == LcpSolverStatus::Success;
  }
};

//==============================================================================
/// Solver configuration options
struct DART_API LcpOptions
{
  /// Maximum number of iterations (0 = solver default)
  int maxIterations{0};

  /// Absolute tolerance for convergence
  double absoluteTolerance{1e-6};

  /// Relative tolerance for convergence
  double relativeTolerance{1e-4};

  /// Complementarity tolerance
  double complementarityTolerance{1e-6};

  /// Whether to validate the solution after solving
  bool validateSolution{true};

  /// Relaxation parameter for SOR-based methods (1.0 = Gauss-Seidel)
  double relaxation{1.0};

  /// Whether to use warm starting (reuse previous solution)
  bool warmStart{false};

  /// Whether to enable early termination for pivoting methods
  bool earlyTermination{false};

  /// Solver-specific options (reserved for future use)
  void* customOptions{nullptr};

  /// Default constructor with sensible defaults
  LcpOptions() = default;

  /// Create options for relaxation-based methods
  ///
  /// \param[in] relaxation Relaxation parameter
  /// \param[in] maxIter Maximum iterations
  /// \return Configured LcpOptions
  static LcpOptions withRelaxation(double relaxation, int maxIter = 100)
  {
    LcpOptions opts;
    opts.relaxation = relaxation;
    opts.maxIterations = maxIter;
    return opts;
  }

  /// Create options for high accuracy requirements
  ///
  /// \return Configured LcpOptions with tight tolerances
  static LcpOptions highAccuracy()
  {
    LcpOptions opts;
    opts.absoluteTolerance = 1e-10;
    opts.relativeTolerance = 1e-8;
    opts.maxIterations = 1000;
    return opts;
  }

  /// Create options for real-time use
  ///
  /// \return Configured LcpOptions for fast computation
  static LcpOptions realTime()
  {
    LcpOptions opts;
    opts.maxIterations = 50;
    opts.absoluteTolerance = 1e-4;
    opts.validateSolution = false;
    return opts;
  }
};

//==============================================================================
/// Bundles the inputs to a boxed LCP: w = Ax - b, l <= x <= u with optional
/// friction index mapping.
struct DART_API LcpProblem
{
  LcpProblem(
      Eigen::MatrixXd A_,
      Eigen::VectorXd b_,
      Eigen::VectorXd lo_,
      Eigen::VectorXd hi_,
      Eigen::VectorXi findex_)
    : A(std::move(A_)),
      b(std::move(b_)),
      lo(std::move(lo_)),
      hi(std::move(hi_)),
      findex(std::move(findex_))
  {
    // Empty
  }

  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd lo;
  Eigen::VectorXd hi;
  Eigen::VectorXi findex;
};

} // namespace math
} // namespace dart

#endif // DART_MATH_LCP_LCPTYPES_HPP_
