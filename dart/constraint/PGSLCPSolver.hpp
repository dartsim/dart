/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_CONSTRAINT_PGSLCPSOLVER_HPP_
#define DART_CONSTRAINT_PGSLCPSOLVER_HPP_

#include <cstddef>

#include "dart/config.hpp"
#include "dart/constraint/LCPSolver.hpp"

namespace dart {
namespace constraint {

/// PGSLCPSolver
class PGSLCPSolver : public LCPSolver
{
public:
  /// Constructor
  explicit PGSLCPSolver(double _timestep);

  /// Constructor
  virtual ~PGSLCPSolver();

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

struct PGSOption
{
  int itermax;
  double sor_w;
  double eps_ea;
  double eps_res;
  double eps_div;

  void setDefault();
};

bool solvePGS(int n, int nskip, int /*nub*/, double* A,
                            double* x, double * b,
                            double * lo, double * hi, int * findex,
                            PGSOption * option);


} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_PGSLCPSOLVER_HPP_

