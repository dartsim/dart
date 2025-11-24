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

#include "dart/math/lcp/pivoting/LemkeSolver.hpp"

#include <Eigen/SVD>

#include <vector>

#include <cmath>

namespace dart {
namespace math {

namespace {

// Legacy Lemke implementation (was LemkeImpl) kept locally.
int LemkeImpl(
    const Eigen::MatrixXd& M, const Eigen::VectorXd& q, Eigen::VectorXd* z)
{
  const int n = q.size();

  const double zer_tol = 1e-5;
  const double piv_tol = 1e-8;
  const int maxiter = 1000;
  int err = 0;

  if (q.minCoeff() >= 0) {
    *z = Eigen::VectorXd::Zero(n);
    return err;
  }

  *z = Eigen::VectorXd::Zero(2 * n);
  int iter = 0;
  double ratio = 0;
  int leaving = 0;
  Eigen::VectorXd Be = Eigen::VectorXd::Constant(n, 1);
  Eigen::VectorXd x = q;
  std::vector<int> bas;
  std::vector<int> nonbas;

  const int t = 2 * n;
  int entering = t;

  for (int i = 0; i < n; ++i) {
    nonbas.push_back(i);
  }

  Eigen::MatrixXd B = -Eigen::MatrixXd::Identity(n, n);

  if (!bas.empty()) {
    Eigen::MatrixXd B_copy = B;
    for (std::size_t i = 0; i < bas.size(); ++i) {
      B.col(i) = M.col(bas[i]);
    }
    for (std::size_t i = 0; i < nonbas.size(); ++i) {
      B.col(bas.size() + i) = B_copy.col(nonbas[i]);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(B);
    const double cond = svd.singularValues()(0)
                        / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond > 1e16) {
      (*z) = Eigen::VectorXd::Zero(n);
      err = 3;
      return err;
    }
    x = -B.householderQr().solve(q);
  }

  if (x.minCoeff() >= 0) {
    Eigen::VectorXd __z = Eigen::VectorXd::Zero(2 * n);
    for (std::size_t i = 0; i < bas.size(); ++i) {
      (__z).row(bas[i]) = x.row(static_cast<int>(i));
    }
    (*z) = __z.head(n);
    return err;
  }

  Eigen::VectorXd minuxX = -x;
  int lvindex;
  double tval = minuxX.maxCoeff(&lvindex);
  for (std::size_t i = 0; i < nonbas.size(); ++i) {
    bas.push_back(nonbas[i] + n);
  }
  leaving = bas[lvindex];

  bas[lvindex] = t; // pivoting in the artificial variable

  Eigen::VectorXd U = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    if (x[i] < 0)
      U[i] = 1;
  }
  Be = -(B * U);
  x += tval * U;
  x[lvindex] = tval;
  B.col(lvindex) = Be;

  for (iter = 0; iter < maxiter; ++iter) {
    if (leaving == t) {
      break;
    } else if (leaving < n) {
      entering = n + leaving;
      Be = Eigen::VectorXd::Zero(n);
      Be[leaving] = -1;
    } else {
      entering = leaving - n;
      Be = M.col(entering);
    }

    Eigen::VectorXd d = B.householderQr().solve(Be);

    std::vector<int> j;
    for (int i = 0; i < n; ++i) {
      if (d[i] > piv_tol)
        j.push_back(i);
    }
    if (j.empty()) {
      err = 2;
      break;
    }

    const std::size_t jSize = j.size();
    Eigen::VectorXd minRatio(jSize);
    for (std::size_t i = 0; i < jSize; ++i) {
      minRatio[i] = (x[j[i]] + zer_tol) / d[j[i]];
    }
    const double theta = minRatio.minCoeff();

    std::vector<int> tmpJ;
    std::vector<double> tmpd;
    for (std::size_t i = 0; i < jSize; ++i) {
      if (x[j[i]] / d[j[i]] <= theta) {
        tmpJ.push_back(j[i]);
        tmpd.push_back(d[j[i]]);
      }
    }
    j = tmpJ;

    if (std::find(j.begin(), j.end(), lvindex) != j.end()) {
      int pos = 0;
      for (std::size_t i = 0; i < j.size(); ++i) {
        if (j[i] == lvindex) {
          pos = static_cast<int>(i);
          break;
        }
      }
      j[pos] = j[j.size() - 1];
      j[j.size() - 1] = lvindex;
      tmpd[pos] = tmpd[tmpd.size() - 1];
      tmpd[tmpd.size() - 1] = d[lvindex];
      d[lvindex] = tmpd[pos];
    }

    ratio = x[lvindex] / d[lvindex];

    x = x - ratio * d;
    x[lvindex] = ratio;
    B.col(lvindex) = Be;
    bas[lvindex] = entering;
  }

  if (iter >= maxiter && leaving != t) {
    err = 1;
  }

  if (err == 0) {
    for (std::size_t i = 0; i < bas.size(); ++i) {
      if (bas[i] < z->size()) {
        (*z)[bas[i]] = x[static_cast<int>(i)];
      }
    }

    Eigen::VectorXd __z = z->head(n);
    *z = __z;
  }

  return err;
}

bool ValidateImpl(
    const Eigen::MatrixXd& M,
    const Eigen::VectorXd& z,
    const Eigen::VectorXd& q)
{
  const double threshold = 1e-4;
  const int n = z.size();

  const Eigen::VectorXd w = M * z + q;
  for (int i = 0; i < n; ++i) {
    if (w(i) < -threshold || z(i) < -threshold)
      return false;
    if (std::abs(w(i) * z(i)) > threshold)
      return false;
  }
  return true;
}

} // namespace

//==============================================================================
LemkeSolver::LemkeSolver()
{
  // Lemke is an exact solver, no need for iterations or tolerances
  mDefaultOptions.maxIterations = 0;       // Not applicable
  mDefaultOptions.validateSolution = true; // Always validate
}

//==============================================================================
LcpResult LemkeSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    const LcpOptions& options)
{
  LcpResult result;

  // Check problem dimensions
  if (A.rows() != A.cols() || A.rows() != b.size()) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Matrix dimensions inconsistent";
    return result;
  }

  // Call the legacy Lemke solver
  const int exitCode = LemkeImpl(A, b, &x);

  // Interpret exit code
  result.iterations = 1; // Pivoting methods don't have iterations in same sense
  Eigen::VectorXd w = A * x + b;
  const double complementarityError = (x.array() * w.array()).abs().maxCoeff();
  const bool feasible = (w.minCoeff() >= -1e-8) && (x.minCoeff() >= -1e-8);

  if (exitCode == 0 || feasible) {
    result.status = LcpSolverStatus::Success;

    if (options.validateSolution) {
      const bool isValid = ValidateImpl(A, x, b);
      result.validated = true;
      if (!isValid) {
        result.status = LcpSolverStatus::NumericalError;
        result.message = "Solution validation failed";
      }
    }
  } else {
    result.status = LcpSolverStatus::Failed;
    result.message = "Lemke algorithm failed (no solution found)";
  }

  result.complementarity = complementarityError;
  result.residual = w.lpNorm<Eigen::Infinity>();

  return result;
}

//==============================================================================
std::string LemkeSolver::getName() const
{
  return "Lemke";
}

//==============================================================================
std::string LemkeSolver::getCategory() const
{
  return "Pivoting";
}

} // namespace math
} // namespace dart
