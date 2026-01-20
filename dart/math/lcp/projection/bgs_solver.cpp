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

#include "dart/math/lcp/projection/bgs_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/pivoting/direct_solver.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <numeric>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace dart::math {
namespace {

constexpr int kMaxDirectBlockSize = 3;

struct BlockData
{
  std::vector<int> indices;
  Eigen::MatrixXd A;
  Eigen::VectorXd baseB;
  Eigen::VectorXd lo;
  Eigen::VectorXd hi;
  Eigen::VectorXi findex;
};

double matrixInfinityNorm(const Eigen::MatrixXd& A)
{
  if (A.size() == 0)
    return 0.0;

  return A.cwiseAbs().rowwise().sum().maxCoeff();
}

double vectorInfinityNorm(const Eigen::VectorXd& v)
{
  return v.size() > 0 ? v.cwiseAbs().maxCoeff() : 0.0;
}

bool isStandardBlock(const BlockData& block, double absTol)
{
  return (block.lo.array().abs().maxCoeff() <= absTol)
         && (block.hi.array() == std::numeric_limits<double>::infinity()).all()
         && (block.findex.array() < 0).all();
}

bool buildBlockData(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    std::span<const int> indices,
    BlockData& block,
    std::string* message)
{
  const auto m = std::ssize(indices);
  if (m == 0) {
    if (message)
      *message = "Block size must be positive";
    return false;
  }

  block.indices.assign(indices.begin(), indices.end());
  block.A.resize(m, m);
  block.baseB.resize(m);
  block.lo.resize(m);
  block.hi.resize(m);
  block.findex.resize(m);

  for (int r = 0; r < m; ++r) {
    const int globalIndex = indices[r];
    block.baseB[r] = b[globalIndex];
    block.lo[r] = lo[globalIndex];
    block.hi[r] = hi[globalIndex];

    const int frictionIndex = findex[globalIndex];
    if (frictionIndex < 0) {
      block.findex[r] = -1;
    } else {
      auto it = std::ranges::find(indices, frictionIndex);
      if (it == indices.end()) {
        if (message)
          *message = "Block partition must include friction index";
        return false;
      }
      block.findex[r] = static_cast<int>(std::distance(indices.begin(), it));
    }

    for (int c = 0; c < m; ++c)
      block.A(r, c) = A(globalIndex, indices[c]);
  }

  return true;
}

bool buildBlocks(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    const BgsSolver::Parameters& params,
    std::vector<BlockData>& blocks,
    std::string* message)
{
  const auto n = std::ssize(b);
  blocks.clear();
  if (n == 0)
    return true;

  if (!params.blockSizes.empty()) {
    int total = 0;
    for (const int size : params.blockSizes) {
      if (size <= 0) {
        if (message)
          *message = "Block sizes must be positive";
        return false;
      }
      total += size;
    }

    if (total != n) {
      if (message)
        *message = "Block sizes must sum to problem dimension";
      return false;
    }

    int offset = 0;
    for (const int size : params.blockSizes) {
      std::vector<int> indices;
      indices.reserve(static_cast<std::size_t>(size));
      for (int i = 0; i < size; ++i)
        indices.push_back(offset + i);
      offset += size;

      BlockData block;
      if (!buildBlockData(
              A,
              b,
              lo,
              hi,
              findex,
              std::span<const int>{indices},
              block,
              message))
        return false;
      blocks.push_back(std::move(block));
    }

    return true;
  }

  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);

  auto findRoot = [&](int v) {
    while (parent[v] != v) {
      parent[v] = parent[parent[v]];
      v = parent[v];
    }
    return v;
  };

  auto unite = [&](int a, int b) {
    const int rootA = findRoot(a);
    const int rootB = findRoot(b);
    if (rootA != rootB)
      parent[rootB] = rootA;
  };

  for (int i = 0; i < n; ++i) {
    const int frictionIndex = findex[i];
    if (frictionIndex >= 0)
      unite(i, frictionIndex);
  }

  std::vector<std::vector<int>> groups(n);
  for (int i = 0; i < n; ++i)
    groups[findRoot(i)].push_back(i);

  for (const auto& indices : groups) {
    if (indices.empty())
      continue;
    BlockData block;
    if (!buildBlockData(
            A,
            b,
            lo,
            hi,
            findex,
            std::span<const int>{indices},
            block,
            message))
      return false;
    blocks.push_back(std::move(block));
  }

  return true;
}

} // namespace

//==============================================================================
BgsSolver::BgsSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void BgsSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const BgsSolver::Parameters& BgsSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult BgsSolver::solve(
    const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
{
  LcpResult result;

  std::string problemMessage;
  if (!detail::validateProblem(problem, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  const auto& A = problem.A;
  const auto& b = problem.b;
  const auto& lo = problem.lo;
  const auto& hi = problem.hi;
  const auto& findex = problem.findex;

  const auto n = std::ssize(b);
  if (n == 0) {
    x.resize(0);
    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual = 0.0;
    result.complementarity = 0.0;
    result.validated = options.validateSolution;
    return result;
  }

  if (x.size() != n || !options.warmStart || !x.allFinite())
    x = Eigen::VectorXd::Zero(n);

  const int maxIterations = std::max(
      1,
      (options.maxIterations > 0) ? options.maxIterations
                                  : mDefaultOptions.maxIterations);
  const double absTol = (options.absoluteTolerance > 0.0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double relTol = (options.relativeTolerance > 0.0)
                            ? options.relativeTolerance
                            : mDefaultOptions.relativeTolerance;
  const double compTolOpt = (options.complementarityTolerance > 0.0)
                                ? options.complementarityTolerance
                                : mDefaultOptions.complementarityTolerance;

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;

  std::vector<BlockData> blocks;
  if (!buildBlocks(A, b, lo, hi, findex, *params, blocks, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  if (blocks.empty()) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "No blocks configured";
    return result;
  }

  Eigen::VectorXd w;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  double tol = 0.0;
  double compTol = 0.0;
  double residual = 0.0;
  double complementarity = 0.0;

  auto updateMetrics = [&]() -> bool {
    w = A * x - b;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = boundsMessage;
      return false;
    }

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    tol = std::max(absTol, relTol * scale);
    compTol = std::max(compTolOpt, relTol * scale);
    residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
    complementarity
        = detail::complementarityInfinityNorm(x, w, loEff, hiEff, compTol);
    return true;
  };

  if (!updateMetrics())
    return result;

  bool converged = (residual <= tol && complementarity <= compTol);
  int iterationsUsed = 0;

  DirectSolver directSolver;
  DantzigSolver blockSolver;
  LcpOptions blockOptions = blockSolver.getDefaultOptions();
  blockOptions.absoluteTolerance = absTol;
  blockOptions.relativeTolerance = relTol;
  blockOptions.complementarityTolerance = compTolOpt;
  blockOptions.validateSolution = false;
  blockOptions.warmStart = true;
  blockOptions.customOptions = nullptr;

  for (int iter = 0; iter < maxIterations && !converged; ++iter) {
    iterationsUsed = iter + 1;

    for (const auto& block : blocks) {
      const auto m = std::ssize(block.indices);
      Eigen::VectorXd xBlock(m);
      for (int r = 0; r < m; ++r)
        xBlock[r] = x[block.indices[r]];

      Eigen::VectorXd AxBlock(m);
      for (int r = 0; r < m; ++r) {
        const int globalIndex = block.indices[r];
        AxBlock[r] = A.row(globalIndex).dot(x);
      }

      const Eigen::VectorXd AxSelf = block.A * xBlock;
      const Eigen::VectorXd bEff = block.baseB - (AxBlock - AxSelf);

      LcpProblem subProblem(block.A, bEff, block.lo, block.hi, block.findex);
      const bool useDirect
          = (m <= kMaxDirectBlockSize) && isStandardBlock(block, absTol);
      const LcpResult blockResult
          = useDirect ? directSolver.solve(subProblem, xBlock, blockOptions)
                      : blockSolver.solve(subProblem, xBlock, blockOptions);
      if (blockResult.status == LcpSolverStatus::InvalidProblem
          || blockResult.status == LcpSolverStatus::NumericalError) {
        result.status = blockResult.status;
        result.message = blockResult.message;
        return result;
      }

      for (int r = 0; r < m; ++r)
        x[block.indices[r]] = xBlock[r];
    }

    if (!updateMetrics())
      return result;

    if (residual <= tol && complementarity <= compTol)
      converged = true;
  }

  result.iterations = iterationsUsed;
  result.residual = residual;
  result.complementarity = complementarity;

  if (converged) {
    result.status = LcpSolverStatus::Success;
  } else {
    result.status = LcpSolverStatus::MaxIterations;
  }

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double validationTol = std::max(tol, compTol);
    std::string validationMessage;
    const bool valid = detail::validateSolution(
        x, w, loEff, hiEff, validationTol, &validationMessage);
    result.validated = true;
    if (!valid) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = validationMessage.empty() ? "Solution validation failed"
                                                 : validationMessage;
    }
  }

  return result;
}

//==============================================================================
std::string BgsSolver::getName() const
{
  return "BGS";
}

//==============================================================================
std::string BgsSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
