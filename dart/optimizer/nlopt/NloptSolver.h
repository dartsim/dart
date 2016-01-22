/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_OPTIMIZER_NLOPT_NLOPTSOLVER_H_
#define DART_OPTIMIZER_NLOPT_NLOPTSOLVER_H_

#include <nlopt.hpp>

#include "dart/common/Deprecated.h"
#include "dart/optimizer/Solver.h"

namespace dart {
namespace optimizer {

class Problem;

/// \brief class NloptSolver
class NloptSolver : public Solver
{
public:

  /// Default Constructor
  NloptSolver(const Solver::Properties& _properties = Solver::Properties(),
              nlopt::algorithm _alg = nlopt::LN_COBYLA);

  /// Alternative Constructor
  NloptSolver(std::shared_ptr<Problem> _problem,
              nlopt::algorithm _alg = nlopt::LN_COBYLA);

  /// Destructor
  virtual ~NloptSolver();

  // Documentation inherited
  virtual bool solve() override;

  // Documentation inherited
  Eigen::VectorXd getLastConfiguration() const;

  // Documentation inherited
  virtual std::string getType() const override;

  // Documentation inherited
  virtual std::shared_ptr<Solver> clone() const override;

  /// Copy the Properties of another NloptSolver
  void copy(const NloptSolver& _other);

  /// Copy the Properties of another NloptSolver
  NloptSolver& operator=(const NloptSolver& _other);

  /// Set the algorithm that is to be used by the nlopt solver
  void setAlgorithm(nlopt::algorithm _alg);

  /// Get the algorithm that is to be used by the nlopt solver
  nlopt::algorithm getAlgorithm() const;

  /// Set number of maximum evaluations
  ///
  /// Deprecated: Please use setNumMaxIterations() instead
  DEPRECATED(5.0)
  virtual void setNumMaxEvaluations(size_t _numVal);

  /// Get number of maximum evaluations
  ///
  /// Deprecated: Please use getNumMaxIterations() instead
  DEPRECATED(5.0)
  virtual size_t getNumEvaluationMax() const;

private:
  /// \brief Wrapping function for nlopt callback function, nlopt_func
  static double _nlopt_func(unsigned _n,
                            const double* _x,
                            double* _gradient,  // nullptr if not needed
                            void* _func_data);

  /// \brief Wrapping function for nlopt callback function, nlopt_mfunc
  static void _nlopt_mfunc(unsigned _m,
                           double* _result,
                           unsigned _n,
                           const double* _x,
                           double* _gradient,  // nullptr if not needed
                           void* _func_data);

  /// \brief NLOPT data structure
  std::unique_ptr<nlopt::opt> mOpt;

  /// Algorithm to be used by the nlopt::opt
  nlopt::algorithm mAlg;

  /// \brief Optimization parameters
  std::vector<double> mX;

  /// \brief Optimum value of the objective function
  double mMinF;
};

}  // namespace optimizer
}  // namespace dart

#endif  // DART_OPTIMIZER_NLOPT_NLOPTSOLVER_H_

