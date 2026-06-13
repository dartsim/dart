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

#include <dart/export.hpp>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

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
/// Public problem-form classification for LcpProblem.
///
/// A standard LCP is represented internally as a boxed problem with
/// lo = 0, hi = +inf, and no friction-index rows. This enum gives callers one
/// prioritized answer instead of requiring them to interpret overlapping
/// predicate results.
enum class LcpProblemType
{
  /// The bounds/findex vectors do not describe a recognized problem form.
  Invalid = 0,

  /// Standard LCP: w = Ax - b, x >= 0, w >= 0, x'w = 0.
  Standard = 1,

  /// Boxed LCP with fixed lower and upper bounds.
  Boxed = 2,

  /// Boxed LCP with friction-index rows whose bounds depend on another row.
  FrictionIndex = 3
};

//==============================================================================
/// Convert solver status to string
///
/// \param[in] status The solver status to convert
/// \return String representation of the status
DART_API std::string toString(LcpSolverStatus status);

//==============================================================================
/// Convert problem type to string
///
/// \param[in] type The problem type to convert
/// \return String representation of the type
DART_API std::string toString(LcpProblemType type);

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
  /// Construct a standard LCP: w = Ax - b, x >= 0, w >= 0, x'w = 0.
  ///
  /// This is the DART 7 convenience path for solvers that operate on the
  /// standard non-negative complementarity form. Internally it is represented
  /// as a boxed problem with lo = 0, hi = +inf, and findex = -1.
  LcpProblem(Eigen::MatrixXd A_, Eigen::VectorXd b_)
    : A(std::move(A_)), b(std::move(b_))
  {
    const Eigen::Index n = b.size();
    lo = Eigen::VectorXd::Zero(n);
    hi = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
    findex = Eigen::VectorXi::Constant(n, -1);
  }

  /// Construct a boxed LCP without friction-index coupling.
  ///
  /// The lower and upper bounds are supplied explicitly, while findex is set to
  /// -1 for every row.
  LcpProblem(
      Eigen::MatrixXd A_,
      Eigen::VectorXd b_,
      Eigen::VectorXd lo_,
      Eigen::VectorXd hi_)
    : A(std::move(A_)), b(std::move(b_)), lo(std::move(lo_)), hi(std::move(hi_))
  {
    findex = Eigen::VectorXi::Constant(b.size(), -1);
  }

  /// Construct a boxed LCP with optional friction-index coupling.
  ///
  /// This preserves the full ODE-compatible representation used by contact
  /// solvers: findex[i] < 0 means row i has fixed bounds; findex[i] >= 0 means
  /// row i has effective bounds scaled by the referenced normal row.
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

  /// Returns the problem dimension.
  Eigen::Index size() const
  {
    return b.size();
  }

  /// Returns true when no rows are present.
  bool empty() const
  {
    return size() == 0;
  }

  /// Returns true when dimensions, finite data, bounds, and friction indices
  /// form a valid LCP problem.
  bool isValid() const
  {
    return validate(nullptr);
  }

  /// Returns an empty string for valid problems, otherwise a diagnostic for the
  /// first invalid invariant encountered.
  std::string getValidationMessage() const
  {
    std::string message;
    validate(&message);
    return message;
  }

  /// Returns true when bounds and friction indices match the standard LCP form.
  ///
  /// Standard LCP solvers expect lo = 0, hi = +inf, and findex < 0. The
  /// tolerance is applied to the lower-bound zero check so callers can classify
  /// numerically canonicalized problems without open-coding the convention.
  bool isStandardLcp(double tolerance = 0.0) const
  {
    if (!hasConsistentDimensions() || !hasFiniteMatrixAndVector()) {
      return false;
    }

    const Eigen::Index n = size();
    const double tol = tolerance > 0.0 ? tolerance : 0.0;
    for (Eigen::Index i = 0; i < n; ++i) {
      if (!std::isfinite(lo[i]) || std::abs(lo[i]) > tol) {
        return false;
      }
      if (hi[i] != std::numeric_limits<double>::infinity()) {
        return false;
      }
      if (findex[i] >= 0) {
        return false;
      }
    }

    return true;
  }

  /// Returns true when the problem has explicit bounds and no findex coupling.
  ///
  /// This includes standard LCPs, which are represented as the canonical boxed
  /// form lo = 0 and hi = +inf. Use hasFrictionIndex() to distinguish contact
  /// friction rows whose bounds depend on another row's solution.
  bool isBoxedLcp() const
  {
    if (!hasConsistentDimensions() || !hasFiniteMatrixAndVector()
        || !hasValidBounds()) {
      return false;
    }

    return (findex.array() < 0).all();
  }

  /// Returns true if any row uses friction-index coupling.
  bool hasFrictionIndex() const
  {
    if (!hasConsistentDimensions() || !hasFiniteMatrixAndVector()
        || !hasValidBounds()) {
      return false;
    }

    const Eigen::Index n = size();
    bool hasCoupling = false;
    for (Eigen::Index i = 0; i < n; ++i) {
      const int ref = findex[i];
      if (ref < 0) {
        continue;
      }

      if (ref >= n || ref == i) {
        return false;
      }
      if (!std::isfinite(hi[i]) || hi[i] < 0.0) {
        return false;
      }

      hasCoupling = true;
    }

    return hasCoupling;
  }

  /// Returns the number of rows whose bounds depend on a friction index.
  ///
  /// Invalid problems and standard/boxed problems without friction-index rows
  /// return zero, so callers can use this as packet metadata after the shared
  /// problem validator has classified the form.
  Eigen::Index getFrictionIndexRowCount() const
  {
    if (!hasFrictionIndex()) {
      return 0;
    }

    return (findex.array() >= 0).count();
  }

  /// Returns the number of unique normal rows referenced by friction rows.
  ///
  /// This is the contact-count convention used by DART 7 LCP evidence packets
  /// for the ODE-style findex representation: each contact contributes one
  /// normal row, and tangential rows reference that normal row.
  Eigen::Index getFrictionIndexContactCount() const
  {
    if (!hasFrictionIndex()) {
      return 0;
    }

    const Eigen::Index n = size();
    std::vector<bool> referencedNormalRows(static_cast<std::size_t>(n), false);
    Eigen::Index count = 0;
    for (Eigen::Index i = 0; i < n; ++i) {
      const int ref = findex[i];
      if (ref < 0) {
        continue;
      }

      const auto refIndex = static_cast<std::size_t>(ref);
      if (!referencedNormalRows[refIndex]) {
        referencedNormalRows[refIndex] = true;
        ++count;
      }
    }

    return count;
  }

  /// Returns a single prioritized classification for this problem form.
  ///
  /// Standard LCPs are also boxed by storage representation; this method
  /// returns Standard first, then FrictionIndex, then Boxed, then Invalid.
  LcpProblemType getType(double standardTolerance = 0.0) const
  {
    if (isStandardLcp(standardTolerance)) {
      return LcpProblemType::Standard;
    }

    if (hasFrictionIndex()) {
      return LcpProblemType::FrictionIndex;
    }

    if (isBoxedLcp()) {
      return LcpProblemType::Boxed;
    }

    return LcpProblemType::Invalid;
  }

private:
  bool validate(std::string* message) const
  {
    const bool dimensionMismatch
        = (A.rows() != A.cols()) || (A.rows() != b.size())
          || (lo.size() != b.size()) || (hi.size() != b.size())
          || (findex.size() != b.size());
    if (dimensionMismatch) {
      if (message) {
        *message = "Matrix/vector dimensions inconsistent";
      }
      return false;
    }

    if (!A.allFinite()) {
      if (message) {
        *message = "Matrix contains non-finite values";
      }
      return false;
    }

    if (!b.allFinite()) {
      if (message) {
        *message = "Vector b contains non-finite values";
      }
      return false;
    }

    const Eigen::Index n = size();
    for (Eigen::Index i = 0; i < n; ++i) {
      const int ref = findex[i];
      if (ref >= n) {
        if (message) {
          *message = "Friction index entry out of range";
        }
        return false;
      }

      if (ref == i) {
        if (message) {
          *message = "Friction index entry cannot reference itself";
        }
        return false;
      }

      if (std::isnan(lo[i]) || std::isnan(hi[i])) {
        if (message) {
          *message = "Bounds contain NaN";
        }
        return false;
      }

      if (lo[i] == std::numeric_limits<double>::infinity()
          || hi[i] == -std::numeric_limits<double>::infinity()) {
        if (message) {
          *message = "Bounds have invalid infinity direction";
        }
        return false;
      }

      if (ref >= 0 && !std::isfinite(hi[i])) {
        if (message) {
          *message = "Friction coefficient (hi) must be finite";
        }
        return false;
      }

      if (ref >= 0 && hi[i] < 0.0) {
        if (message) {
          *message = "Friction coefficient (hi) must be non-negative";
        }
        return false;
      }

      if (std::isfinite(lo[i]) && std::isfinite(hi[i]) && lo[i] > hi[i]) {
        if (message) {
          *message = "Lower bound exceeds upper bound";
        }
        return false;
      }
    }

    return true;
  }

  /// Returns true when the matrix, bound, and findex storage match b.size().
  bool hasConsistentDimensions() const
  {
    const Eigen::Index n = size();
    return A.rows() == n && A.cols() == n && lo.size() == n && hi.size() == n
           && findex.size() == n;
  }

  /// Returns true when the dense problem data is finite.
  bool hasFiniteMatrixAndVector() const
  {
    return A.allFinite() && b.allFinite();
  }

  /// Returns true when fixed bounds are not NaN and finite ranges are ordered.
  bool hasValidBounds() const
  {
    const Eigen::Index n = size();
    for (Eigen::Index i = 0; i < n; ++i) {
      if (std::isnan(lo[i]) || std::isnan(hi[i])) {
        return false;
      }
      if (lo[i] == std::numeric_limits<double>::infinity()
          || hi[i] == -std::numeric_limits<double>::infinity()) {
        return false;
      }
      if (std::isfinite(lo[i]) && std::isfinite(hi[i]) && lo[i] > hi[i]) {
        return false;
      }
    }

    return true;
  }

public:
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd lo;
  Eigen::VectorXd hi;
  Eigen::VectorXi findex;
};

} // namespace math
} // namespace dart

#endif // DART_MATH_LCP_LCPTYPES_HPP_
