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

#include "dart/optimizer/nlopt/NloptSolver.h"

#include <Eigen/Dense>

#include "dart/optimizer/Problem.h"
#include "dart/optimizer/Function.h"

namespace dart {
namespace optimizer {

//==============================================================================
NloptSolver::NloptSolver(std::shared_ptr<Problem> _problem,
                         nlopt_algorithm _alg)
  : Solver(_problem),
    mMinF(0.0)
{
  assert(_problem);
  assert(_problem->getObjective());

  // Problem dimension
  int dim = _problem->getDimension();

  // Create nlopt object
  mOpt = nlopt_create(_alg, dim);

  // Set relative tolerance on function value
  nlopt_set_ftol_rel(mOpt, 1e-8);

  // Set initial guess for x
  mX = mProperties.mProblem->getInitialGuess();

  // Set tolerance for x
  nlopt_set_xtol_rel(mOpt, 1e-9);

  // Set lower/upper bounds
  nlopt_set_lower_bounds(mOpt, mProperties.mProblem->getLowerBounds().data());
  nlopt_set_upper_bounds(mOpt, mProperties.mProblem->getUpperBounds().data());

  // Set objective function
  FunctionPtr obj = mProperties.mProblem->getObjective();
  nlopt_set_min_objective(mOpt, NloptSolver::_nlopt_func, obj.get());

  // Add equality constraint functions
  for (size_t i = 0; i < mProperties.mProblem->getNumEqConstraints(); ++i)
  {
    FunctionPtr fn = mProperties.mProblem->getEqConstraint(i);
    nlopt_add_equality_constraint(
          mOpt, NloptSolver::_nlopt_func, fn.get(), 1e-8);
  }

  // Add inequality constraint functions
  for (size_t i = 0; i < mProperties.mProblem->getNumIneqConstraints(); ++i)
  {
    FunctionPtr fn = mProperties.mProblem->getIneqConstraint(i);
    nlopt_add_inequality_constraint(
          mOpt, NloptSolver::_nlopt_func, fn.get(), 1e-8);
  }
}

//==============================================================================
NloptSolver::~NloptSolver()
{
  nlopt_destroy(mOpt);
}

//==============================================================================
void NloptSolver::setNumMaxEvaluations(size_t _numVal)
{
  nlopt_set_maxeval(mOpt, _numVal);
}

//==============================================================================
size_t NloptSolver::getNumEvaluationMax() const
{
  return nlopt_get_maxeval(mOpt);
}

//==============================================================================
bool NloptSolver::solve()
{
  // Optimize
  nlopt_result result = nlopt_optimize(mOpt, mX.data(), &mMinF);

  // Negative result means failure. For the detail, see nlopt.hpp
  if (result < 0)
    return false;

  // Store optimal and optimum values
  mProperties.mProblem->setOptimumValue(mMinF);
  Eigen::VectorXd minX = Eigen::VectorXd::Zero(getProblem()->getDimension());
  for (size_t i = 0; i < getProblem()->getDimension(); ++i)
    minX[i] = mX[i];
  getProblem()->setOptimalSolution(minX);

  return true;
}

//==============================================================================
std::string NloptSolver::getType() const
{
  return "NloptSolver";
}

//==============================================================================
double NloptSolver::_nlopt_func(unsigned _n,
                                const double* _x,
                                double* _gradient,
                                void* _func_data)
{
  Function* fn = static_cast<Function*>(_func_data);

  Eigen::Map<const Eigen::VectorXd> x(_x, _n);

  if (_gradient)
  {
    Eigen::Map<Eigen::VectorXd> grad(_gradient, _n);
    fn->evalGradient(x, grad);
  }

  return fn->eval(x);
}

//==============================================================================
void NloptSolver::_nlopt_mfunc(unsigned _m,
                               double* _result,
                               unsigned _n,
                               const double* _x,
                               double* _gradient,
                               void* _func_data)
{
  Eigen::Map<const Eigen::VectorXd> x(_x, _n);
  Eigen::Map<Eigen::VectorXd> f(_result, _m);
  Eigen::Map<Eigen::MatrixXd> grad(_gradient, 0, 0);
  if (_gradient)
    new (&grad) Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor>(_gradient, _m, _n);

  return (*static_cast<MultiFunction*>(_func_data))(x, f, grad);
}

}  // namespace optimizer
}  // namespace dart
