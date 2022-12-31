/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/optimizer/nlopt/NloptSolver.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/Problem.hpp"

#include <Eigen/Dense>

#include <memory>

namespace dart {
namespace optimizer {

//==============================================================================
NloptSolver::NloptSolver(
    const Solver::Properties& properties, nlopt::algorithm alg)
  : NloptSolver(properties, convertAlgorithm(alg))
{
  // Do nothing
}

//==============================================================================
NloptSolver::NloptSolver(
    const Solver::Properties& properties, NloptSolver::Algorithm alg)
  : Solver(properties), mOpt(nullptr), mAlg(convertAlgorithm(alg)), mMinF(0.0)
{
  // Do nothing
}

//==============================================================================
NloptSolver::NloptSolver(std::shared_ptr<Problem> problem, nlopt::algorithm alg)
  : NloptSolver(std::move(problem), convertAlgorithm(alg))
{
  // Do nothing
}

//==============================================================================
NloptSolver::NloptSolver(
    std::shared_ptr<Problem> problem, NloptSolver::Algorithm alg)
  : Solver(std::move(problem)),
    mOpt(nullptr),
    mAlg(convertAlgorithm(alg)),
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
static Eigen::VectorXd convertToEigen(const std::vector<double>& v)
{
  Eigen::VectorXd result(v.size());
  for (std::size_t i = 0; i < v.size(); ++i)
    result[static_cast<Eigen::Index>(i)] = v[i];

  return result;
}

//==============================================================================
bool NloptSolver::solve()
{
  // Allocate a new nlopt::opt structure if needed
  std::size_t dimension = mProperties.mProblem->getDimension();
  if (nullptr == mOpt || mOpt->get_dimension() != dimension
      || mOpt->get_algorithm() != mAlg)
  {
    mOpt = std::make_unique<nlopt::opt>(mAlg, dimension);
  }
  else
  {
    mOpt->remove_equality_constraints();
    mOpt->remove_inequality_constraints();
  }

  const std::shared_ptr<Problem>& problem = mProperties.mProblem;

  mOpt->set_maxeval(static_cast<int>(mProperties.mNumMaxIterations));
  mOpt->set_xtol_rel(mProperties.mTolerance);
  mOpt->set_lower_bounds(convertToStd(problem->getLowerBounds()));
  mOpt->set_upper_bounds(convertToStd(problem->getUpperBounds()));

  // Set up the nlopt::opt
  mOpt->set_min_objective(
      NloptSolver::_nlopt_func, problem->getObjective().get());

  for (std::size_t i = 0; i < problem->getNumEqConstraints(); ++i)
  {
    FunctionPtr fn = problem->getEqConstraint(i);
    try
    {
      mOpt->add_equality_constraint(
          NloptSolver::_nlopt_func, fn.get(), mProperties.mTolerance);
    }
    catch (const std::invalid_argument& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an equality constraint to an Nlopt solver. "
            << "Check whether your algorithm [" << nlopt::algorithm_name(mAlg)
            << "] (" << mAlg << ") supports equality constraints!\n";
      assert(false);
    }
    catch (const std::exception& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an equality constraint to the Nlopt solver. "
            << "This might be a bug in DART; please report this!\n";
      assert(false);
    }
  }

  for (std::size_t i = 0; i < problem->getNumIneqConstraints(); ++i)
  {
    FunctionPtr fn = problem->getIneqConstraint(i);
    try
    {
      mOpt->add_inequality_constraint(
          NloptSolver::_nlopt_func, fn.get(), mProperties.mTolerance);
    }
    catch (const std::invalid_argument& e)
    {
      dterr << "[NloptSolver::solve] Encountered exception [" << e.what()
            << "] while adding an inequality constraint to an Nlopt solver. "
            << "Check whether your algorithm [" << nlopt::algorithm_name(mAlg)
            << "] (" << mAlg << ") supports inequality constraints!\n";
      assert(false);
    }
    catch (const std::exception& e)
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
  if (!(nlopt::SUCCESS <= result && result <= nlopt::XTOL_REACHED))
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
  return std::make_shared<NloptSolver>(getSolverProperties(), getAlgorithm2());
}

//==============================================================================
void NloptSolver::copy(const NloptSolver& other)
{
  setProperties(other.getSolverProperties());
  setAlgorithm(other.getAlgorithm2());
}

//==============================================================================
NloptSolver& NloptSolver::operator=(const NloptSolver& other)
{
  copy(other);
  return *this;
}

//==============================================================================
void NloptSolver::setAlgorithm(nlopt::algorithm alg)
{
  setAlgorithm(convertAlgorithm(alg));
}

//==============================================================================
void NloptSolver::setAlgorithm(NloptSolver::Algorithm alg)
{
  mAlg = convertAlgorithm(alg);
}

//==============================================================================
nlopt::algorithm NloptSolver::getAlgorithm() const
{
  return convertAlgorithm(getAlgorithm2());
}

//==============================================================================
NloptSolver::Algorithm NloptSolver::getAlgorithm2() const
{
  return convertAlgorithm(mAlg);
}

//==============================================================================
#define NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(alg_name)                          \
  case alg_name:                                                               \
    return nlopt::algorithm::alg_name;

//==============================================================================
#define NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(alg_name)                          \
  case nlopt::algorithm::alg_name:                                             \
    return alg_name;

//==============================================================================
nlopt::algorithm NloptSolver::convertAlgorithm(NloptSolver::Algorithm algorithm)
{
  switch (algorithm)
  {
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_DIRECT)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_DIRECT_L)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_DIRECT_L_RAND)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_DIRECT_NOSCAL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_DIRECT_L_NOSCAL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_DIRECT_L_RAND_NOSCAL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_ORIG_DIRECT)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_ORIG_DIRECT_L)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GD_STOGO)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GD_STOGO_RAND)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_LBFGS_NOCEDAL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_LBFGS)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_PRAXIS)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_VAR1)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_VAR2)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_TNEWTON)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_TNEWTON_RESTART)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_TNEWTON_PRECOND)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_TNEWTON_PRECOND_RESTART)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_CRS2_LM)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_MLSL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GD_MLSL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_MLSL_LDS)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GD_MLSL_LDS)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_MMA)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_COBYLA)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_NEWUOA)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_NEWUOA_BOUND)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_NELDERMEAD)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_SBPLX)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_AUGLAG)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_AUGLAG)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_AUGLAG_EQ)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_AUGLAG_EQ)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LN_BOBYQA)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_ISRES)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(AUGLAG)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(AUGLAG_EQ)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(G_MLSL)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(G_MLSL_LDS)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_SLSQP)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(LD_CCSAQ)
    NLOPTSOLVER_ALGORITHM_DART_TO_NLOPT(GN_ESCH)
    default:
      dtwarn << "[NloptSolver] Attempt to convert unsupported algorithm '"
             << algorithm << "'. Use nlopt::algorithm::LN_COBYLA instead. \n";
      return nlopt::algorithm::LN_COBYLA;
  }
}

//==============================================================================
NloptSolver::Algorithm NloptSolver::convertAlgorithm(nlopt::algorithm algorithm)
{
  switch (algorithm)
  {
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_DIRECT)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_DIRECT_L)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_DIRECT_L_RAND)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_DIRECT_NOSCAL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_DIRECT_L_NOSCAL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_DIRECT_L_RAND_NOSCAL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_ORIG_DIRECT)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_ORIG_DIRECT_L)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GD_STOGO)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GD_STOGO_RAND)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_LBFGS_NOCEDAL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_LBFGS)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_PRAXIS)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_VAR1)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_VAR2)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_TNEWTON)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_TNEWTON_RESTART)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_TNEWTON_PRECOND)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_TNEWTON_PRECOND_RESTART)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_CRS2_LM)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_MLSL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GD_MLSL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_MLSL_LDS)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GD_MLSL_LDS)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_MMA)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_COBYLA)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_NEWUOA)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_NEWUOA_BOUND)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_NELDERMEAD)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_SBPLX)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_AUGLAG)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_AUGLAG)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_AUGLAG_EQ)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_AUGLAG_EQ)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LN_BOBYQA)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_ISRES)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(AUGLAG)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(AUGLAG_EQ)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(G_MLSL)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(G_MLSL_LDS)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_SLSQP)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(LD_CCSAQ)
    NLOPTSOLVER_ALGORITHM_NLOPT_TO_DART(GN_ESCH)
    default:
      dtwarn << "[NloptSolver] Attempt to convert unsupported algorithm '"
             << algorithm
             << "'. Use NloptSolver::Algorithm::LN_COBYLA instead. \n";
      return LN_COBYLA;
  }
}

//==============================================================================
double NloptSolver::_nlopt_func(
    unsigned n, const double* x, double* gradient, void* func_data)
{
  Function* fn = static_cast<Function*>(func_data);

  Eigen::Map<const Eigen::VectorXd> mapX(x, n);

  if (gradient)
  {
    Eigen::Map<Eigen::VectorXd> grad(gradient, n);
    fn->evalGradient(mapX, grad);
  }

  return fn->eval(mapX);
}

//==============================================================================
void NloptSolver::_nlopt_mfunc(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* gradient,
    void* func_data)
{
  Eigen::Map<const Eigen::VectorXd> mapX(x, n);
  Eigen::Map<Eigen::VectorXd> f(result, m);
  Eigen::Map<Eigen::MatrixXd> grad(gradient, 0, 0);
  if (gradient)
    new (&grad) Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor>(gradient, m, n);

  return (*static_cast<MultiFunction*>(func_data))(mapX, f, grad);
}

} // namespace optimizer
} // namespace dart
