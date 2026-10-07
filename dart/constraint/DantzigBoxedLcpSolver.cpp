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

#include "dart/constraint/DantzigBoxedLcpSolver.hpp"

#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"

#include <limits>
#include <stdexcept>

namespace dart {
namespace constraint {

namespace {

::dart::lcpsolver::dantzig::DantzigLcpScratch<double>& dantzigScratch()
{
  static thread_local ::dart::lcpsolver::dantzig::DantzigLcpScratch<double>
      scratch;
  return scratch;
}

} // namespace

//==============================================================================
const std::string& DantzigBoxedLcpSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DantzigBoxedLcpSolver::getStaticType()
{
  static const std::string type = "DantzigBoxedLcpSolver";
  return type;
}

//==============================================================================
bool DantzigBoxedLcpSolver::solve(
    int n,
    double* A,
    double* x,
    double* b,
    int /*nub*/,
    double* lo,
    double* hi,
    int* findex,
    bool earlyTermination)
{
  auto& scratch = dantzigScratch();
  if (n > 0 && static_cast<std::size_t>(n) > scratch.stateCapacity)
    reserve(static_cast<std::size_t>(n));
  return ::dart::lcpsolver::dantzig::solveLcpWithScratch<double>(
      n, A, x, b, nullptr, 0, lo, hi, findex, scratch, earlyTermination);
}

//==============================================================================
void DantzigBoxedLcpSolver::reserve(std::size_t n)
{
  if (n == 0u)
    return;

  auto& scratch = dantzigScratch();
  if (n <= scratch.stateCapacity)
    return;

  // Keep the logical solve size unchanged and grow only retained storage.
  const auto maxRows
      = static_cast<std::size_t>(std::numeric_limits<int>::max() - 3);
  if (n > maxRows)
    throw std::length_error("Dantzig scratch row count exceeds padded stride");
  std::size_t rowCapacity = 1u;
  while (rowCapacity < n) {
    if (rowCapacity > maxRows / 2u) {
      rowCapacity = n;
      break;
    }
    rowCapacity *= 2u;
  }
  n = rowCapacity;
  const auto nskip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));
  const auto nskipSize = static_cast<std::size_t>(nskip);
  if (n > scratch.L.max_size() / nskipSize)
    throw std::length_error("Dantzig scratch matrix size exceeds capacity");
  const auto ldltRemoveTmpScalars = n + 2u * nskipSize;

  scratch.L.reserve(n * nskipSize);
  scratch.d.reserve(n);
  scratch.w.reserve(n);
  scratch.deltaW.reserve(n);
  scratch.deltaX.reserve(n);
  scratch.dell.reserve(n);
  scratch.ell.reserve(n);
  scratch.p.reserve(n);
  scratch.C.reserve(n);
  scratch.ldltRemoveTmp.reserve(ldltRemoveTmpScalars);
  scratch.rowPointers.reserve(n);
  scratch.reserveState(n);
}

#if DART_BUILD_MODE_DEBUG
//==============================================================================
bool DantzigBoxedLcpSolver::canSolve(int /*n*/, const double* /*A*/)
{
  // TODO(JS): Not implemented.
  return true;
}
#endif

} // namespace constraint
} // namespace dart
