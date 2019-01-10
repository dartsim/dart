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

#ifndef DART_CONSTRAINT_DANTZIGLCPSOLVER_HPP_
#define DART_CONSTRAINT_DANTZIGLCPSOLVER_HPP_

#include <cstddef>

#include "dart/config.hpp"
#include "dart/constraint/LCPSolver.hpp"

namespace dart {
namespace constraint {

/// \deprecated This header has been deprecated in DART 6.7. Please include
/// DantzigBoxedLcpSolver.hpp intead.
///
/// DantzigLCPSolver is a LCP solver that uses ODE's implementation of Dantzig
/// algorithm
class DantzigLCPSolver : public LCPSolver
{
public:
  /// Constructor
  explicit DantzigLCPSolver(double _timestep);

  /// Constructor
  virtual ~DantzigLCPSolver();

  // Documentation inherited
  void solve(ConstrainedGroup* _group) override;

#ifndef NDEBUG
private:
  /// Return true if the matrix is symmetric
  bool isSymmetric(std::size_t _n, double* _A);

  /// Return true if the diagonla block of matrix is symmetric
  bool isSymmetric(std::size_t _n, double* _A, std::size_t _begin, std::size_t _end);

  /// Print debug information
  void print(std::size_t _n, double* _A, double* _x, double* _lo, double* _hi,
             double* _b, double* w, int* _findex);
#endif
};

} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_DANTZIGLCPSOLVER_HPP_

