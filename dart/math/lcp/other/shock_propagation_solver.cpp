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

#include "dart/math/lcp/other/shock_propagation_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/pivoting/direct_solver.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <algorithm>
#include <iterator>
#include <limits>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace dart::math {
namespace {

constexpr int kMaxDirectBlockSize = 3;
constexpr int kMaxFrictionIndexExactFastPathSize = 192;

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
  if (A.size() == 0) {
    return 0.0;
  }

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

bool solveUnconstrainedFeasibleBlock(
    const BlockData& block,
    const Eigen::VectorXd& bEff,
    const double absTol,
    Eigen::VectorXd& xBlock)
{
  const auto m = std::ssize(block.indices);
  if (m == 0 || m > kMaxDirectBlockSize || (block.findex.array() >= 0).any()
      || !bEff.allFinite()) {
    return false;
  }

  const Eigen::FullPivLU<Eigen::MatrixXd> lu(block.A);
  if (!lu.isInvertible()) {
    return false;
  }

  Eigen::VectorXd candidate = lu.solve(bEff);
  if (!candidate.allFinite()) {
    return false;
  }

  for (int i = 0; i < candidate.size(); ++i) {
    if (candidate[i] < block.lo[i] - absTol
        || candidate[i] > block.hi[i] + absTol) {
      return false;
    }
    candidate[i] = std::clamp(candidate[i], block.lo[i], block.hi[i]);
  }

  xBlock = std::move(candidate);
  return true;
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
    if (message) {
      *message = "Block size must be positive";
    }
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
        if (message) {
          *message = "Block partition must include friction index";
        }
        return false;
      }
      block.findex[r] = static_cast<int>(std::distance(indices.begin(), it));
    }

    for (int c = 0; c < m; ++c) {
      block.A(r, c) = A(globalIndex, indices[c]);
    }
  }

  return true;
}

bool buildBlocks(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    const ShockPropagationSolver::Parameters& params,
    std::vector<BlockData>& blocks,
    std::string* message)
{
  const auto n = std::ssize(b);
  blocks.clear();
  if (n == 0) {
    return true;
  }

  if (!params.blockSizes.empty()) {
    int total = 0;
    for (const int size : params.blockSizes) {
      if (size <= 0) {
        if (message) {
          *message = "Block sizes must be positive";
        }
        return false;
      }
      total += size;
    }

    if (total != n) {
      if (message) {
        *message = "Block sizes must sum to problem dimension";
      }
      return false;
    }

    int offset = 0;
    for (const int size : params.blockSizes) {
      const auto indexRange = std::views::iota(offset, offset + size);
      std::vector<int> indices(indexRange.begin(), indexRange.end());
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
              message)) {
        return false;
      }
      blocks.push_back(std::move(block));
    }

    return true;
  }

  const auto problemIndices = std::views::iota(0, static_cast<int>(n));
  std::vector<int> parent(problemIndices.begin(), problemIndices.end());

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
    if (rootA != rootB) {
      parent[rootB] = rootA;
    }
  };

  for (int i = 0; i < n; ++i) {
    const int frictionIndex = findex[i];
    if (frictionIndex >= 0) {
      unite(i, frictionIndex);
    }
  }

  std::vector<std::vector<int>> groups(n);
  for (int i = 0; i < n; ++i) {
    groups[findRoot(i)].push_back(i);
  }

  for (const auto& indices : groups) {
    if (indices.empty()) {
      continue;
    }
    BlockData block;
    if (!buildBlockData(
            A,
            b,
            lo,
            hi,
            findex,
            std::span<const int>{indices},
            block,
            message)) {
      return false;
    }
    blocks.push_back(std::move(block));
  }

  return true;
}

bool buildLayerOrder(
    std::span<const BlockData> blocks,
    const ShockPropagationSolver::Parameters& params,
    std::vector<std::vector<int>>& layers,
    std::string* message)
{
  const auto numBlocks = std::ssize(blocks);
  layers.clear();

  if (numBlocks == 0) {
    if (message) {
      *message = "No blocks configured";
    }
    return false;
  }

  if (params.layers.empty()) {
    const auto blockIndices = std::views::iota(0, static_cast<int>(numBlocks));
    layers.emplace_back(blockIndices.begin(), blockIndices.end());
    return true;
  }

  std::vector<int> used(numBlocks, 0);
  for (const auto& layer : params.layers) {
    if (layer.empty()) {
      if (message) {
        *message = "Layer must include at least one block";
      }
      return false;
    }
    for (const int blockIndex : layer) {
      if (blockIndex < 0 || blockIndex >= numBlocks) {
        if (message) {
          *message = "Layer block index out of range";
        }
        return false;
      }
      if (used[blockIndex]++) {
        if (message) {
          *message = "Block index appears in multiple layers";
        }
        return false;
      }
    }
  }

  for (int i = 0; i < numBlocks; ++i) {
    if (used[i] == 0) {
      if (message) {
        *message = "Layers must cover all blocks";
      }
      return false;
    }
  }

  layers = params.layers;
  return true;
}

bool validateBlockLayerStructure(
    const int problemSize,
    const ShockPropagationSolver::Parameters& params,
    std::string* message)
{
  int numBlocks = problemSize;
  if (!params.blockSizes.empty()) {
    int total = 0;
    numBlocks = static_cast<int>(std::ssize(params.blockSizes));
    for (const int size : params.blockSizes) {
      if (size <= 0) {
        if (message) {
          *message = "Block sizes must be positive";
        }
        return false;
      }
      total += size;
    }

    if (total != problemSize) {
      if (message) {
        *message = "Block sizes must sum to problem dimension";
      }
      return false;
    }
  }

  if (params.layers.empty()) {
    return true;
  }

  std::vector<int> used(static_cast<std::size_t>(numBlocks), 0);
  for (const auto& layer : params.layers) {
    if (layer.empty()) {
      if (message) {
        *message = "Layer must include at least one block";
      }
      return false;
    }

    for (const int blockIndex : layer) {
      if (blockIndex < 0 || blockIndex >= numBlocks) {
        if (message) {
          *message = "Layer block index out of range";
        }
        return false;
      }
      if (used[static_cast<std::size_t>(blockIndex)]++) {
        if (message) {
          *message = "Block index appears in multiple layers";
        }
        return false;
      }
    }
  }

  for (const int count : used) {
    if (count == 0) {
      if (message) {
        *message = "Layers must cover all blocks";
      }
      return false;
    }
  }

  return true;
}

} // namespace

//==============================================================================
ShockPropagationSolver::ShockPropagationSolver()
{
  mDefaultOptions.maxIterations = 1;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void ShockPropagationSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const ShockPropagationSolver::Parameters&
ShockPropagationSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult ShockPropagationSolver::solve(
    const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
{
  LcpResult result;

  const auto& A = problem.A;
  const auto& b = problem.b;
  const auto& lo = problem.lo;
  const auto& hi = problem.hi;
  const auto& findex = problem.findex;

  const auto n = std::ssize(b);
  if (n == 0) {
    std::string problemMessage;
    if (!detail::validateProblem(problem, &problemMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = problemMessage;
      return result;
    }

    x.resize(0);
    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual = 0.0;
    result.complementarity = 0.0;
    result.validated = options.validateSolution;
    return result;
  }

  const bool resetInitialGuess
      = x.size() != n || !options.warmStart || !x.allFinite();

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
  bool blockLayerStructureValidated = false;
  std::string problemMessage;
  std::vector<BlockData> blocks;
  bool blocksBuilt = false;

  auto completeExactFastPath = [&](const Eigen::VectorXd& fastW) -> bool {
    Eigen::VectorXd loEff;
    Eigen::VectorXd hiEff;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = boundsMessage;
      return true;
    }

    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual
        = detail::naturalResidualInfinityNorm(x, fastW, loEff, hiEff);
    result.complementarity = detail::complementarityInfinityNorm(
        x, fastW, loEff, hiEff, compTolOpt);
    result.validated = options.validateSolution;
    return true;
  };

  auto tryExactFastPath = [&]() -> bool {
    Eigen::VectorXd fastW;
    if (options.warmStart) {
      return false;
    }

    const double validationTolerance = std::max(absTol, compTolOpt);
    if (problem.isStandardLcp(absTol)) {
      const double strictInteriorTolerance = std::max(0.0, absTol);
      bool strictInteriorFastPath = false;
      Eigen::LLT<Eigen::MatrixXd> exactFactorization(A);
      if (exactFactorization.info() == Eigen::Success) {
        Eigen::VectorXd candidate = exactFactorization.solve(b);
        if (candidate.allFinite()
            && candidate.minCoeff() > strictInteriorTolerance) {
          fastW = A * candidate - b;
          if (detail::validateSolution(
                  candidate, fastW, lo, hi, validationTolerance)) {
            x = std::move(candidate);
            strictInteriorFastPath = true;
          }
        }
      }

      if (!strictInteriorFastPath
          && !detail::trySolveStrictInteriorStandardLcp(
              problem, absTol, validationTolerance, x, &fastW)) {
        return false;
      }
    } else if (problem.isBoxedLcp()) {
      if (!detail::trySolveProjectedActiveSetBoxedLcp(
              problem, absTol, validationTolerance, x, &fastW)) {
        return false;
      }
    } else if (problem.hasFrictionIndex()) {
      if (problem.size() > kMaxFrictionIndexExactFastPathSize) {
        return false;
      }
      if (!detail::trySolveInteriorFrictionIndexLcp(
              problem, absTol, validationTolerance, x, &fastW)) {
        return false;
      }
    } else {
      return false;
    }

    return completeExactFastPath(fastW);
  };

  const bool canTryExactFastPath
      = n > 0 && !options.warmStart
        && (problem.isStandardLcp(absTol) || problem.isBoxedLcp()
            || problem.hasFrictionIndex());
  if (canTryExactFastPath
      && !validateBlockLayerStructure(
          static_cast<int>(n), *params, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }
  blockLayerStructureValidated = canTryExactFastPath;

  if (canTryExactFastPath && problem.hasFrictionIndex()
      && options.customOptions != nullptr
      && (!params->blockSizes.empty() || !params->layers.empty())) {
    if (!buildBlocks(A, b, lo, hi, findex, *params, blocks, &problemMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = problemMessage;
      return result;
    }
    blocksBuilt = true;
  }

  if (canTryExactFastPath && tryExactFastPath()) {
    return result;
  }

  if (!detail::validateProblem(problem, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  if (!blockLayerStructureValidated && !options.warmStart
      && (problem.isStandardLcp(absTol) || problem.isBoxedLcp()
          || problem.hasFrictionIndex())
      && !validateBlockLayerStructure(
          static_cast<int>(n), *params, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  if (!options.warmStart && problem.hasFrictionIndex()
      && options.customOptions != nullptr && !blocksBuilt
      && (!params->blockSizes.empty() || !params->layers.empty())) {
    if (!buildBlocks(A, b, lo, hi, findex, *params, blocks, &problemMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = problemMessage;
      return result;
    }
    blocksBuilt = true;
  }

  if (!blocksBuilt
      && !buildBlocks(A, b, lo, hi, findex, *params, blocks, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  if (resetInitialGuess) {
    x = Eigen::VectorXd::Zero(n);
  }

  std::vector<std::vector<int>> layers;
  if (!buildLayerOrder(
          std::span<const BlockData>{blocks},
          *params,
          layers,
          &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
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

  if (!updateMetrics()) {
    return result;
  }

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

    for (const auto& layer : layers) {
      for (const int blockIndex : layer) {
        const auto& block = blocks[blockIndex];
        const auto m = std::ssize(block.indices);
        Eigen::VectorXd xBlock(m);
        for (int r = 0; r < m; ++r) {
          xBlock[r] = x[block.indices[r]];
        }

        Eigen::VectorXd AxBlock(m);
        for (int r = 0; r < m; ++r) {
          const int globalIndex = block.indices[r];
          AxBlock[r] = A.row(globalIndex).dot(x);
        }

        const Eigen::VectorXd AxSelf = block.A * xBlock;
        const Eigen::VectorXd bEff = block.baseB - (AxBlock - AxSelf);

        if (solveUnconstrainedFeasibleBlock(block, bEff, absTol, xBlock)) {
          for (int r = 0; r < m; ++r) {
            x[block.indices[r]] = xBlock[r];
          }
          continue;
        }

        LcpProblem subProblem(block.A, bEff, block.lo, block.hi, block.findex);
        const double validationTolerance = std::max(absTol, compTolOpt);
        if (detail::trySolveInteriorFrictionIndexLcp(
                subProblem, absTol, validationTolerance, xBlock)) {
          for (int r = 0; r < m; ++r) {
            x[block.indices[r]] = xBlock[r];
          }
          continue;
        }

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

        for (int r = 0; r < m; ++r) {
          x[block.indices[r]] = xBlock[r];
        }
      }
    }

    if (!updateMetrics()) {
      return result;
    }

    if (residual <= tol && complementarity <= compTol) {
      converged = true;
    }
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
std::string ShockPropagationSolver::getName() const
{
  return "ShockPropagation";
}

//==============================================================================
std::string ShockPropagationSolver::getCategory() const
{
  return "Other";
}

} // namespace dart::math
