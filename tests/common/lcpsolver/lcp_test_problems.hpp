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

#ifndef DART_TESTS_LCPTESTPROBLEMS_HPP_
#define DART_TESTS_LCPTESTPROBLEMS_HPP_

#include <Eigen/Dense>

#include <random>

#include <cstdint>

namespace dart {
namespace test {

/// Flags indicating what makes an LCP problem ill-formed
enum class LCPProblemIssue : uint32_t
{
  None = 0,                       ///< Well-formed problem
  NotSymmetric = 1 << 0,          ///< Matrix A is not symmetric
  NotPositiveDefinite = 1 << 1,   ///< Matrix A has negative eigenvalues
  IllConditioned = 1 << 2,        ///< Condition number > 10000
  SingularOrNearSingular = 1 << 3 ///< Matrix is singular or nearly singular
};

/// Bitwise OR operator for combining flags
inline LCPProblemIssue operator|(LCPProblemIssue a, LCPProblemIssue b)
{
  return static_cast<LCPProblemIssue>(
      static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

/// Bitwise AND operator for checking flags
inline LCPProblemIssue operator&(LCPProblemIssue a, LCPProblemIssue b)
{
  return static_cast<LCPProblemIssue>(
      static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

/// Bitwise OR assignment operator
inline LCPProblemIssue& operator|=(LCPProblemIssue& a, LCPProblemIssue b)
{
  a = a | b;
  return a;
}

/// Check if a flag is set
inline bool hasIssue(LCPProblemIssue flags, LCPProblemIssue flag)
{
  return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(flag)) != 0;
}

/// Test problem structure for LCP benchmarks and tests
struct LCPProblem
{
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  int dimension;
  std::string name;
  LCPProblemIssue issues; ///< Flags indicating problem issues

  LCPProblem(
      int dim,
      const std::string& problemName,
      LCPProblemIssue problemIssues = LCPProblemIssue::None)
    : dimension(dim), name(problemName), issues(problemIssues)
  {
    A.resize(dim, dim);
    b.resize(dim);
  }

  /// Check if this is a well-formed problem
  bool isWellFormed() const
  {
    return issues == LCPProblemIssue::None;
  }
};

/// Collection of standard LCP test problems
class LCPTestProblems
{
public:
  /// 1D test problem
  static LCPProblem getProblem1D()
  {
    LCPProblem problem(1, "1D");
    problem.A << 1;
    problem.b << -1.5;
    return problem;
  }

  /// 2D test problem
  static LCPProblem getProblem2D()
  {
    LCPProblem problem(2, "2D");
    problem.A << 3.12, 0.1877, 0.1877, 3.254;
    problem.b << -0.00662, -0.006711;
    return problem;
  }

  /// 4D well-formed test problem (positive definite)
  static LCPProblem getProblem4D()
  {
    LCPProblem problem(4, "4D");
    // Diagonally dominant positive definite matrix
    problem.A << 5.0, 0.5, 0.3, 0.2, 0.5, 5.0, 0.4, 0.3, 0.3, 0.4, 5.0, 0.5,
        0.2, 0.3, 0.5, 5.0;
    problem.b << -0.01, -0.02, -0.03, -0.04;
    return problem;
  }

  /// 4D ill-formed test problem (original, has negative eigenvalues)
  static LCPProblem getProblem4D_IllFormed()
  {
    LCPProblem problem(
        4,
        "4D_IllFormed",
        LCPProblemIssue::NotPositiveDefinite | LCPProblemIssue::IllConditioned);
    problem.A << 3.999, 0.9985, 1.001, -2, 0.9985, 3.998, -2, 0.9995, 1.001, -2,
        4.002, 1.001, -2, 0.9995, 1.001, 4.001;
    problem.b << -0.01008, -0.009494, -0.07234, -0.07177;
    return problem;
  }

  /// 6D well-formed test problem (diagonally dominant)
  static LCPProblem getProblem6D()
  {
    LCPProblem problem(6, "6D");
    // Diagonally dominant positive definite matrix
    problem.A << 8.0, 0.5, 0.3, 0.2, 0.1, 0.1, 0.5, 8.0, 0.4, 0.3, 0.2, 0.1,
        0.3, 0.4, 8.0, 0.5, 0.3, 0.2, 0.2, 0.3, 0.5, 8.0, 0.4, 0.3, 0.1, 0.2,
        0.3, 0.4, 8.0, 0.5, 0.1, 0.1, 0.2, 0.3, 0.5, 8.0;
    problem.b << -0.01, -0.02, -0.03, -0.04, -0.05, -0.06;
    return problem;
  }

  /// 6D ill-formed test problem (original, has negative eigenvalue)
  static LCPProblem getProblem6D_IllFormed()
  {
    LCPProblem problem(6, "6D_IllFormed", LCPProblemIssue::NotPositiveDefinite);
    problem.A << 3.1360, -2.0370, 0.9723, 0.1096, -2.0370, 0.9723, -2.0370,
        3.7820, 0.8302, -0.0257, 2.4730, 0.0105, 0.9723, 0.8302, 5.1250,
        -2.2390, -1.9120, 3.4080, 0.1096, -0.0257, -2.2390, 3.1010, -0.0257,
        -2.2390, -2.0370, 2.4730, -1.9120, -0.0257, 5.4870, -0.0242, 0.9723,
        0.0105, 3.4080, -2.2390, -0.0242, 3.3860;
    problem.b << 0.1649, -0.0025, -0.0904, -0.0093, -0.0000, -0.0889;
    return problem;
  }

  /// 12D well-formed test problem (diagonally dominant)
  static LCPProblem getProblem12D()
  {
    LCPProblem problem(12, "12D");
    // Create diagonally dominant matrix
    problem.A.setZero();
    for (int i = 0; i < 12; ++i) {
      problem.A(i, i) = 10.0; // Strong diagonal
      for (int j = 0; j < 12; ++j) {
        if (i != j) {
          problem.A(i, j) = 0.3; // Small off-diagonal
        }
      }
    }
    problem.b.setConstant(-0.01);
    return problem;
  }

  /// 12D ill-formed test problem (original, has negative eigenvalues)
  static LCPProblem getProblem12D_IllFormed()
  {
    LCPProblem problem(
        12, "12D_IllFormed", LCPProblemIssue::NotPositiveDefinite);
    problem.A << 4.03, -1.014, -1.898, 1.03, -1.014, -1.898, 1, -1.014, -1.898,
        -2, -1.014, -1.898, -1.014, 4.885, -1.259, 1.888, 3.81, 2.345, -1.879,
        1.281, -2.334, 1.022, 0.206, 1.27, -1.898, -1.259, 3.2, -1.032, -0.6849,
        1.275, 1.003, 0.6657, 3.774, 1.869, 1.24, 1.85, 1.03, 1.888, -1.032,
        4.03, 1.888, -1.032, -2, 1.888, -1.032, 1, 1.888, -1.032, -1.014, 3.81,
        -0.6849, 1.888, 3.225, 1.275, -1.879, 1.85, -1.27, 1.022, 1.265, 0.6907,
        -1.898, 2.345, 1.275, -1.032, 1.275, 4.86, 1.003, -1.24, 0.2059, 1.869,
        -2.309, 3.791, 1, -1.879, 1.003, -2, -1.879, 1.003, 3.97, -1.879, 1.003,
        0.9703, -1.879, 1.003, -1.014, 1.281, 0.6657, 1.888, 1.85, -1.24,
        -1.879, 3.187, 1.234, 1.022, 3.755, -0.6714, -1.898, -2.334, 3.774,
        -1.032, -1.27, 0.2059, 1.003, 1.234, 4.839, 1.869, 2.299, 1.27, -2,
        1.022, 1.869, 1, 1.022, 1.869, 0.9703, 1.022, 1.869, 3.97, 1.022, 1.869,
        -1.014, 0.206, 1.24, 1.888, 1.265, -2.309, -1.879, 3.755, 2.299, 1.022,
        4.814, -1.25, -1.898, 1.27, 1.85, -1.032, 0.6907, 3.791, 1.003, -0.6714,
        1.27, 1.869, -1.25, 3.212;
    problem.b << -0.00981, -1.458e-10, 5.357e-10, -0.0098, -1.44e-10, 5.298e-10,
        -0.009807, -1.399e-10, 5.375e-10, -0.009807, -1.381e-10, 5.316e-10;
    return problem;
  }

  /// 24D test problem (doubled 12D for larger test)
  static LCPProblem getProblem24D()
  {
    auto problem12 = getProblem12D();
    LCPProblem problem(24, "24D");

    // Create a block diagonal matrix with two 12x12 blocks
    problem.A.setZero();
    problem.A.block(0, 0, 12, 12) = problem12.A;
    problem.A.block(12, 12, 12, 12) = problem12.A;

    // Concatenate the b vectors
    problem.b.segment(0, 12) = problem12.b;
    problem.b.segment(12, 12) = problem12.b;

    return problem;
  }

  /// 48D test problem (quadrupled 12D for large test)
  static LCPProblem getProblem48D()
  {
    auto problem12 = getProblem12D();
    LCPProblem problem(48, "48D");

    // Create a block diagonal matrix with four 12x12 blocks
    problem.A.setZero();
    for (int i = 0; i < 4; ++i) {
      problem.A.block(i * 12, i * 12, 12, 12) = problem12.A;
    }

    // Concatenate the b vectors
    for (int i = 0; i < 4; ++i) {
      problem.b.segment(i * 12, 12) = problem12.b;
    }

    return problem;
  }

  /// Get all well-formed test problems (suitable for solver testing)
  static std::vector<LCPProblem> getWellFormedProblems()
  {
    return {
        getProblem1D(),
        getProblem2D(),
        getProblem4D(),
        getProblem6D(),
        getProblem12D(),
        getProblem24D(),
        getProblem48D()};
  }

  /// Get all ill-formed test problems (for robustness testing)
  static std::vector<LCPProblem> getIllFormedProblems()
  {
    return {
        getProblem4D_IllFormed(),
        getProblem6D_IllFormed(),
        getProblem12D_IllFormed()};
  }

  /// Get all test problems (both well-formed and ill-formed)
  static std::vector<LCPProblem> getAllProblems()
  {
    auto wellFormed = getWellFormedProblems();
    auto illFormed = getIllFormedProblems();
    wellFormed.insert(wellFormed.end(), illFormed.begin(), illFormed.end());
    return wellFormed;
  }

  /// Generate a random well-formed (positive definite) LCP problem
  static LCPProblem generateRandomWellFormed(
      int dimension, unsigned int seed = 42)
  {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(0.1, 1.0);

    LCPProblem problem(
        dimension, "Random" + std::to_string(dimension) + "D_WellFormed");

    // Create a positive definite matrix using A = M^T * M + lambda * I
    // where M is a random matrix and lambda ensures positive definiteness
    Eigen::MatrixXd M(dimension, dimension);
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        M(i, j) = dist(rng);
      }
    }

    // Make it symmetric and positive definite
    problem.A = M.transpose() * M;

    // Add diagonal dominance to ensure positive definiteness
    double lambda = 2.0 * dimension;
    problem.A += lambda * Eigen::MatrixXd::Identity(dimension, dimension);

    // Generate random b vector
    for (int i = 0; i < dimension; ++i) {
      problem.b(i) = -dist(rng) * 0.1; // Negative values typical for LCP
    }

    return problem;
  }

  /// Generate a random ill-formed (non-positive definite) LCP problem
  static LCPProblem generateRandomIllFormed(
      int dimension, unsigned int seed = 42)
  {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    LCPProblem problem(
        dimension,
        "Random" + std::to_string(dimension) + "D_IllFormed",
        LCPProblemIssue::NotPositiveDefinite);

    // Create a symmetric but NOT positive definite matrix
    Eigen::MatrixXd M(dimension, dimension);
    for (int i = 0; i < dimension; ++i) {
      for (int j = i; j < dimension; ++j) {
        M(i, j) = M(j, i) = dist(rng);
      }
    }

    // Subtract identity to create negative eigenvalues
    problem.A
        = M - 0.5 * dimension * Eigen::MatrixXd::Identity(dimension, dimension);

    // Generate random b vector
    for (int i = 0; i < dimension; ++i) {
      problem.b(i) = dist(rng) * 0.1;
    }

    return problem;
  }

  /// Generate multiple random well-formed problems of various dimensions
  static std::vector<LCPProblem> generateRandomWellFormedProblems(
      const std::vector<int>& dimensions, unsigned int baseSeed = 42)
  {
    std::vector<LCPProblem> problems;
    problems.reserve(dimensions.size());

    for (size_t i = 0; i < dimensions.size(); ++i) {
      problems.push_back(generateRandomWellFormed(dimensions[i], baseSeed + i));
    }

    return problems;
  }

  /// Generate multiple random ill-formed problems of various dimensions
  static std::vector<LCPProblem> generateRandomIllFormedProblems(
      const std::vector<int>& dimensions, unsigned int baseSeed = 42)
  {
    std::vector<LCPProblem> problems;
    problems.reserve(dimensions.size());

    for (size_t i = 0; i < dimensions.size(); ++i) {
      problems.push_back(
          generateRandomIllFormed(dimensions[i], baseSeed + i * 100));
    }

    return problems;
  }
};

} // namespace test
} // namespace dart

#endif // DART_TESTS_LCPTESTPROBLEMS_HPP_
