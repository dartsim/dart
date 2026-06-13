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

#pragma once

#include <dart/math/lcp/lcp_solver.hpp>
#include <dart/math/lcp/pivoting/dantzig/lcp.hpp>

#include <dart/common/stl_allocator.hpp>

#include <vector>

namespace dart::math {

/// Wrapper around the legacy Dantzig BLCP solver exposing the modern
/// `LcpSolver` interface.
class DART_API DantzigSolver : public LcpSolver
{
public:
  /// Reusable work storage for repeated same-shape Dantzig solves.
  struct DART_API Scratch
  {
    using DoubleAllocator = dart::common::StlAllocator<double>;
    using IntAllocator = dart::common::StlAllocator<int>;

    Scratch() = default;

    explicit Scratch(dart::common::MemoryAllocator& allocator)
      : Adata(DoubleAllocator{allocator}),
        xdata(DoubleAllocator{allocator}),
        wdata(DoubleAllocator{allocator}),
        bdata(DoubleAllocator{allocator}),
        loData(DoubleAllocator{allocator}),
        hiData(DoubleAllocator{allocator}),
        findexData(IntAllocator{allocator}),
        lcp(allocator)
    {
    }

    std::vector<double, DoubleAllocator> Adata;
    std::vector<double, DoubleAllocator> xdata;
    std::vector<double, DoubleAllocator> wdata;
    std::vector<double, DoubleAllocator> bdata;
    std::vector<double, DoubleAllocator> loData;
    std::vector<double, DoubleAllocator> hiData;
    std::vector<int, IntAllocator> findexData;
    Eigen::VectorXd w;
    Eigen::VectorXd loEff;
    Eigen::VectorXd hiEff;
    DantzigLcpScratch<double> lcp;

    void clear() noexcept;
  };

  DantzigSolver();
  ~DantzigSolver() override = default;

  using LcpSolver::solve;

  LcpResult solve(
      const LcpProblem& problem,
      Eigen::VectorXd& x,
      const LcpOptions& options) override;

  LcpResult solve(
      const LcpProblem& problem,
      Eigen::VectorXd& x,
      Scratch& scratch,
      const LcpOptions& options);

  LcpResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const Eigen::VectorXd& lo,
      const Eigen::VectorXd& hi,
      const Eigen::VectorXi& findex,
      Eigen::VectorXd& x,
      Scratch& scratch,
      const LcpOptions& options);

  std::string getName() const override;
  std::string getCategory() const override;
};

} // namespace dart::math
