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

#include <memory>

#include "dart/optimizer/nlopt/NloptSolver.hpp"

#include <Eigen/Dense>

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/optimizer/Problem.hpp"
#include "dart/optimizer/Function.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
NloptSolver::NloptSolver(const Solver::Properties& _properties,
                         nlopt::algorithm _alg)
  : Solver(_properties),
    mOpt(nullptr),
    mAlg(_alg),
    mMinF(0.0)
{
  // Do nothing
}

//==============================================================================
NloptSolver::NloptSolver(std::shared_ptr<Problem> _problem,
                         nlopt::algorithm _alg)
  : Solver(_problem),
    mOpt(nullptr),
    mAlg(_alg),
    mMinF(0.0)
{
  // Do nothing
}

//==============================================================================
NloptSolver::~NloptSolver()
{
  // Do nothing
}

//==============================================================================
static std::vector<double> convertToStd(const Eigen::VectorXd& v)
{
  return std::vector<double>(v.data(), v.data() + v.size());
}

//==============================================================================
static Eigen::VectorXd convertToEigen(const std::vector<double>& _v)
{
  Eigen::VectorXd result(_v.size());
  for(std::size_t i=0; i<_v.size(); ++i)
    result[i] = _v[i];

  return result;
}

//==============================================================================
bool NloptSolver::solve()
{
  // Allocate a new nlopt::opt structure if needed
  std::size_t dimension = mProperties.mProblem->getDimension();
  if(nullptr == mOpt
     || mOpt->get_dimension() != dimension
     || mOpt->get_algorithm() != mAlg)
  {
    mOpt = common::make_unique<nlopt::opt>(mAlg, dimension);
  }
  else
  {
    mOpt->remove_equality_constraints();
    mOpt->remove_inequality_constraints();
  }

  const std::shared_ptr<Problem>& problem = mProperties.mProblem;

  mOpt->set_maxeval(mProperties.mNumMaxIterations);
  mOpt->set_xtol_rel(mProperties.mTolerance);
  mOpt->set_lower_bounds(convertToStd(problem->getLowerBounds()));
  mOpt->set_upper_bounds(convertToStd(problem->getUpperBounds()));

  // Set up the nlopt::opt
  mOpt->set_min_objective(NloptSolver::_nlopt_func,
                          problem->getObjective().get());

  for(std::size_t i=0; i<problem->getNumEqConstraints(); ++i)
  {
    FunctionPtr fn = problem->getEqConstraint(i);
    try
    {
      mOpt->add_equality_constraint(NloptSolver::_nlopt_func, fn.get(),
                                    mProperties.mTolerance);
    }
    catch(const std::invalid_argument& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an equality constraint to an Nlopt solver. "
            << "Check whether your algorithm [" << nlopt::algorithm_name(mAlg)
            << "] (" << mAlg << ") supports equality constraints!\n";
      assert(false);
    }
    catch(const std::exception& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an equality constraint to the Nlopt solver. "
            << "This might be a bug in DART; please report this!\n";
      assert(false);
    }
  }

  for(std::size_t i=0; i<problem->getNumIneqConstraints(); ++i)
  {
    FunctionPtr fn = problem->getIneqConstraint(i);
    try
    {
      mOpt->add_inequality_constraint(NloptSolver::_nlopt_func, fn.get(),
                                      mProperties.mTolerance);
    }
    catch(const std::invalid_argument& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an inequality constraint to an Nlopt solver. "
            << "Check whether your algorithm [" << nlopt::algorithm_name(mAlg)
            << "] (" << mAlg << ") supports inequality constraints!\n";
      assert(false);
    }
    catch(const std::exception& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an inequality constraint to the Nlopt solver. "
            << "This might be a bug in DART; please report this!\n";
      assert(false);
    }
  }

  // Optimize
  mX = convertToStd(problem->getInitialGuess());
  nlopt::result result = mOpt->optimize(mX, mMinF);

  // If the result is not in this range, then it failed
  if ( !(nlopt::SUCCESS <= result && result <= nlopt::XTOL_REACHED) )
    return false;

  // Store optimal and optimum values
  problem->setOptimumValue(mMinF);
  problem->setOptimalSolution(convertToEigen(mX));

  return true;
}

//==============================================================================
Eigen::VectorXd NloptSolver::getLastConfiguration() const
{
  return convertToEigen(mX);
}

//==============================================================================
std::string NloptSolver::getType() const
{
  return "NloptSolver";
}

//==============================================================================
std::shared_ptr<Solver> NloptSolver::clone() const
{
  return std::make_shared<NloptSolver>(getSolverProperties(), getAlgorithm());
}

//==============================================================================
void NloptSolver::copy(const NloptSolver& _other)
{
  setProperties(_other.getSolverProperties());
  setAlgorithm(_other.getAlgorithm());
}

//==============================================================================
NloptSolver& NloptSolver::operator=(const NloptSolver& _other)
{
  copy(_other);
  return *this;
}

//==============================================================================
void NloptSolver::setAlgorithm(nlopt::algorithm _alg)
{
  mAlg = _alg;
}

//==============================================================================
nlopt::algorithm NloptSolver::getAlgorithm() const
{
  return mAlg;
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
    fn->evalGradient(static_cast<const Eigen::VectorXd&>(x), grad);
  }

  return fn->eval(static_cast<const Eigen::VectorXd&>(x));
  // TODO(MXG): Remove the static_casts once the old eval() functions are
  // removed and there is no longer ambiguity.
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
