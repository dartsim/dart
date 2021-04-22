/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_OPTIMIZATION_NLOPT_NLOPTSOLVER_HPP_
#define DART_OPTIMIZATION_NLOPT_NLOPTSOLVER_HPP_

#include <nlopt.hpp>

#include "dart/common/Deprecated.hpp"
#include "dart/optimization/Solver.hpp"

namespace dart {
namespace optimization {

class Problem;

/// NloptSolver is a nonlinear programming solver that provides many unlerlying
/// algorithms through nlopt (an third-party library:
/// https://nlopt.readthedocs.io/).
///
/// The algorithms falls into four categories:
///   (1) Global derivative-free
///   (2) Global gradient-based
///   (3) Local derivative-free
///   (4) Local gradient-based,
/// which can be specified by NloptSolver::Algorithm. The element of NloptSolver
/// are mostly of the form NLOPT_{G,L}{N,D}_xxxx, where G/L denotes global/local
/// optimization and N/D denotes derivative-free/gradient-based algorithms,
/// respectively. For the details, please see:
/// https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/
class NloptSolver : public Solver
{
public:
  enum Algorithm
  {
    GN_DIRECT = 0,
    GN_DIRECT_L,
    GN_DIRECT_L_RAND,
    GN_DIRECT_NOSCAL,
    GN_DIRECT_L_NOSCAL,
    GN_DIRECT_L_RAND_NOSCAL,
    GN_ORIG_DIRECT,
    GN_ORIG_DIRECT_L,
    GD_STOGO,
    GD_STOGO_RAND,
    LD_LBFGS_NOCEDAL,
    LD_LBFGS,
    LN_PRAXIS,
    LD_VAR1,
    LD_VAR2,
    LD_TNEWTON,
    LD_TNEWTON_RESTART,
    LD_TNEWTON_PRECOND,
    LD_TNEWTON_PRECOND_RESTART,
    GN_CRS2_LM,
    GN_MLSL,
    GD_MLSL,
    GN_MLSL_LDS,
    GD_MLSL_LDS,
    LD_MMA,
    LN_COBYLA,
    LN_NEWUOA,
    LN_NEWUOA_BOUND,
    LN_NELDERMEAD,
    LN_SBPLX,
    LN_AUGLAG,
    LD_AUGLAG,
    LN_AUGLAG_EQ,
    LD_AUGLAG_EQ,
    LN_BOBYQA,
    GN_ISRES,
    AUGLAG,
    AUGLAG_EQ,
    G_MLSL,
    G_MLSL_LDS,
    LD_SLSQP,
    LD_CCSAQ,
    GN_ESCH,
    NUM_ALGORITHMS ///< Not an algorithm, just the number of them
  };

  /// Default Constructor
  DART_DEPRECATED(6.9)
  NloptSolver(const Solver::Properties& properties, nlopt::algorithm alg);

  /// Default Constructor
  NloptSolver(
      const Solver::Properties& properties = Solver::Properties(),
      Algorithm alg = LN_COBYLA);

  /// Alternative Constructor
  DART_DEPRECATED(6.9)
  NloptSolver(std::shared_ptr<Problem> problem, nlopt::algorithm alg);

  /// Alternative Constructor
  NloptSolver(std::shared_ptr<Problem> problem, Algorithm alg = LN_COBYLA);

  /// Destructor
  ~NloptSolver() override;

  // Documentation inherited
  bool solve() override;

  // Documentation inherited
  Eigen::VectorXd getLastConfiguration() const;

  // Documentation inherited
  std::string getType() const override;

  // Documentation inherited
  std::shared_ptr<Solver> clone() const override;

  /// Copy the Properties of another NloptSolver
  void copy(const NloptSolver& other);

  /// Copy the Properties of another NloptSolver
  NloptSolver& operator=(const NloptSolver& other);

  /// Set the algorithm that is to be used by the nlopt solver
  DART_DEPRECATED(6.9)
  void setAlgorithm(nlopt::algorithm alg);

  /// Set the algorithm that is to be used by the nlopt solver
  void setAlgorithm(Algorithm alg);

  /// Get the algorithm that is to be used by the nlopt solver
  DART_DEPRECATED(6.9)
  nlopt::algorithm getAlgorithm() const;

  /// Get the algorithm that is to be used by the nlopt solver
  Algorithm getAlgorithm2() const;
  // TODO(JS): Rename to getAlgorithm2() once getAlgorithm() is removed in
  // DART 7

private:
  /// Converts nlopt::algorithm to NloptSolver::Algorithm
  static nlopt::algorithm convertAlgorithm(Algorithm algorithm);

  /// Converts NloptSolver::Algorithm to nlopt::algorithm
  static Algorithm convertAlgorithm(nlopt::algorithm algorithm);

  /// Wrapping function for nlopt callback function, nlopt_func
  static double _nlopt_func(
      unsigned n,
      const double* x,
      double* gradient, // nullptr if not needed
      void* func_data);

  /// Wrapping function for nlopt callback function, nlopt_mfunc
  static void _nlopt_mfunc(
      unsigned m,
      double* result,
      unsigned n,
      const double* x,
      double* gradient, // nullptr if not needed
      void* func_data);

  /// NLOPT data structure
  std::unique_ptr<nlopt::opt> mOpt;

  /// Algorithm to be used by the nlopt::opt
  nlopt::algorithm mAlg;

  /// Optimization parameters
  std::vector<double> mX;

  /// Optimum value of the objective function
  double mMinF;
};

} // namespace optimization
} // namespace dart

#endif // DART_OPTIMIZATION_NLOPT_NLOPTSOLVER_HPP_
