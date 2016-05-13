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

#include "dart/optimizer/ipopt/IpoptSolver.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/Problem.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
IpoptSolver::IpoptSolver(const Solver::Properties& _properties)
  : Solver(_properties)
{
  mNlp = new DartTNLP(this);
  mIpoptApp = IpoptApplicationFactory();
}

//==============================================================================
IpoptSolver::IpoptSolver(std::shared_ptr<Problem> _problem)
  : Solver(_problem)
{
  assert(_problem);

  // Create a new instance of nlp (use a SmartPtr, not raw)
  mNlp = new DartTNLP(this);

  // Create a new instance of IpoptApplication (use a SmartPtr, not raw). We are
  // using the factory, since this allows us to compile this with an Ipopt
  // Windows DLL.
  mIpoptApp = IpoptApplicationFactory();
  mIpoptApp->Options()->SetStringValue("mu_strategy", "adaptive");
  mIpoptApp->Options()->SetStringValue("hessian_approximation",
                                       "limited-memory");
}

//==============================================================================
IpoptSolver::~IpoptSolver()
{
  // Do nothing
}

//==============================================================================
bool IpoptSolver::solve()
{
  // Change some options
  // Note: The following choices are only examples, they might not be
  //       suitable for your optimization problem.
  mIpoptApp->Options()->SetNumericValue("tol", getTolerance());
  mIpoptApp->Options()->SetStringValue("output_file", getResultFileName());

  std::size_t freq = mProperties.mIterationsPerPrint;
  if(freq > 0)
  {
    mIpoptApp->Options()->SetNumericValue("print_frequency_iter", freq);
  }
  else
  {
    mIpoptApp->Options()->SetNumericValue(
          "print_frequency_iter", std::numeric_limits<int>::infinity());
  }

  // Intialize the IpoptApplication and process the options
  Ipopt::ApplicationReturnStatus init_status = mIpoptApp->Initialize();
  if (init_status != Ipopt::Solve_Succeeded)
  {
    dterr << "[IpoptSolver::solve] Error during ipopt initialization.\n";
    assert(false);
    return false;
  }

  // Ask Ipopt to solve the problem
  Ipopt::ApplicationReturnStatus status = mIpoptApp->OptimizeTNLP(mNlp);

  if (status == Ipopt::Solve_Succeeded)
    return true;
  else
    return false;
}

//==============================================================================
std::string IpoptSolver::getType() const
{
  return "IpoptSolver";
}

//==============================================================================
std::shared_ptr<Solver> IpoptSolver::clone() const
{
  std::shared_ptr<IpoptSolver> newSolver(
        new IpoptSolver(getSolverProperties(), mIpoptApp->clone()));

  return newSolver;
}

//==============================================================================
const Ipopt::SmartPtr<Ipopt::IpoptApplication>& IpoptSolver::getApplication()
{
  return mIpoptApp;
}

//==============================================================================
Ipopt::SmartPtr<const Ipopt::IpoptApplication> IpoptSolver::getApplication() const
{
  return mIpoptApp;
}

//==============================================================================
IpoptSolver::IpoptSolver(const Properties& _properties,
                         const Ipopt::SmartPtr<Ipopt::IpoptApplication>& _app)
  : Solver(_properties)
{
  mIpoptApp = _app;
}

//==============================================================================
DartTNLP::DartTNLP(IpoptSolver* _solver)
  : Ipopt::TNLP(),
    mSolver(_solver)
{
  assert(_solver && "Null pointer is not allowed.");
}

//==============================================================================
DartTNLP::~DartTNLP()
{
  // Do nothing
}

//==============================================================================
bool DartTNLP::get_nlp_info(Ipopt::Index& n,
                            Ipopt::Index& m,
                            Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag,
                            Ipopt::TNLP::IndexStyleEnum& index_style)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  // Set the number of decision variables
  n = problem->getDimension();

  // Set the total number of constraints
  m = problem->getNumEqConstraints() + problem->getNumIneqConstraints();

  // Set the number of entries in the constraint Jacobian
  nnz_jac_g = n * m;

  // Set the number of entries in the Hessian
  nnz_h_lag = n * n * m;

  // use the C style indexing (0-based)
  index_style = Ipopt::TNLP::C_STYLE;

  return true;
}

//==============================================================================
bool DartTNLP::get_bounds_info(Ipopt::Index n,
                               Ipopt::Number* x_l,
                               Ipopt::Number* x_u,
                               Ipopt::Index m,
                               Ipopt::Number* g_l,
                               Ipopt::Number* g_u)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  assert(static_cast<std::size_t>(n) == problem->getDimension());
  assert(static_cast<std::size_t>(m) == problem->getNumEqConstraints()
         + problem->getNumIneqConstraints());
  DART_UNUSED(m);

  // lower and upper bounds
  for (Ipopt::Index i = 0; i < n; i++)
  {
    x_l[i] = problem->getLowerBounds()[i];
    x_u[i] = problem->getUpperBounds()[i];
  }

  // Add inequality constraint functions
  std::size_t idx = 0;
  for (std::size_t i = 0; i < problem->getNumEqConstraints(); ++i)
  {
    g_l[idx] = g_u[idx] = 0.0;
    ++idx;
  }

  for (std::size_t i = 0; i < problem->getNumIneqConstraints(); ++i)
  {
    // Ipopt interprets any number greater than nlp_upper_bound_inf as
    // infinity. The default value of nlp_upper_bound_inf and
    // nlp_lower_bound_inf is 1e+19 and can be changed through ipopt options.
    g_l[idx] = -std::numeric_limits<double>::infinity();
    g_u[idx] =  0;
    idx++;
  }

  return true;
}

//==============================================================================
bool DartTNLP::get_starting_point(Ipopt::Index n,
                                  bool init_x,
                                  Ipopt::Number* x,
                                  bool init_z,
                                  Ipopt::Number* /*z_L*/,
                                  Ipopt::Number* /*z_U*/,
                                  Ipopt::Index /*m*/,
                                  bool init_lambda,
                                  Ipopt::Number* /*lambda*/)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  // If init_x is true, this method must provide an initial value for x.
  if (init_x)
  {
    for (int i = 0; i < n; ++i)
      x[i] = problem->getInitialGuess()[i];
  }

  // If init_z is true, this method must provide an initial value for the bound
  // multipliers z^L and z^U
  if (init_z)
  {
    // TODO(JS): Not implemented yet.
    dterr << "Initializing lower/upper bounds for z is not supported yet. "
          << "Ignored here.\n";
  }

  // If init_lambda is true, this method must provide an initial value for the
  // constraint multipliers, lambda.
  if (init_lambda)
  {
    // TODO(JS): Not implemented yet.
    dterr << "Initializing lambda is not supported yet. "
          << "Ignored here.\n";
  }

  return true;
}

//==============================================================================
bool DartTNLP::eval_f(Ipopt::Index _n,
                      const Ipopt::Number* _x,
                      bool /*_new_x*/,
                      Ipopt::Number& _obj_value)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  Eigen::Map<const Eigen::VectorXd> x(_x, _n);
  mObjValue = problem->getObjective()->eval(
        static_cast<const Eigen::VectorXd&>(x));
  // TODO(MXG): Remove this static cast once the
  // Eigen::Map<const Eigen::VectorXd>& version of the function is removed

  _obj_value = mObjValue;

  return true;
}

//==============================================================================
bool DartTNLP::eval_grad_f(Ipopt::Index _n,
                           const Ipopt::Number* _x,
                           bool /*_new_x*/,
                           Ipopt::Number* _grad_f)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  Eigen::Map<const Eigen::VectorXd> x(_x, _n);
  Eigen::Map<Eigen::VectorXd> grad(_grad_f, _n);
  problem->getObjective()->evalGradient(
        static_cast<const Eigen::VectorXd&>(x), grad);

  return true;
}

//==============================================================================
bool DartTNLP::eval_g(Ipopt::Index _n,
                      const Ipopt::Number* _x,
                      bool _new_x,
                      Ipopt::Index _m,
                      Ipopt::Number* _g)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  assert(static_cast<std::size_t>(_m) == problem->getNumEqConstraints()
                                    + problem->getNumIneqConstraints());
  DART_UNUSED(_m);

  // TODO(JS):
  if (_new_x)
  {
  }

  Eigen::Map<const Eigen::VectorXd> x(_x, _n);
  std::size_t idx = 0;

  // Evaluate function values for equality constraints
  for (std::size_t i = 0; i < problem->getNumEqConstraints(); ++i)
  {
    _g[idx] = problem->getEqConstraint(i)->eval(
          static_cast<const Eigen::VectorXd&>(x));
    idx++;
  }

  // Evaluate function values for inequality constraints
  for (std::size_t i = 0; i < problem->getNumIneqConstraints(); ++i)
  {
    _g[idx] = problem->getIneqConstraint(i)->eval(
          static_cast<const Eigen::VectorXd&>(x));
    idx++;
  }

  return true;
}

//==============================================================================
bool DartTNLP::eval_jac_g(Ipopt::Index _n,
                          const Ipopt::Number* _x,
                          bool /*_new_x*/,
                          Ipopt::Index _m,
                          Ipopt::Index /*_nele_jac*/,
                          Ipopt::Index* _iRow,
                          Ipopt::Index* _jCol,
                          Ipopt::Number* _values)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  // If the iRow and jCol arguments are not nullptr, then IPOPT wants you to fill
  // in the sparsity structure of the Jacobian (the row and column indices
  // only). At this time, the x argument and the values argument will be nullptr.

  if (nullptr == _values)
  {
    // return the structure of the Jacobian

    // Assume the gradient is dense
    std::size_t idx = 0;
    for (int i = 0; i < _m; ++i)
    {
      for (int j = 0; j < _n; ++j)
      {
        _iRow[idx] = i;
        _jCol[idx] = j;
        ++idx;
      }
    }
  }
  else
  {
    // return the values of the Jacobian of the constraints
    std::size_t idx = 0;
    Eigen::Map<const Eigen::VectorXd> x(_x, _n);
    Eigen::Map<Eigen::VectorXd> grad(nullptr, 0);

    // Evaluate function values for equality constraints
    for (std::size_t i = 0; i < problem->getNumEqConstraints(); ++i)
    {
      new (&grad)Eigen::Map<Eigen::VectorXd>(_values + idx, _n);
      problem->getEqConstraint(i)->evalGradient(
            static_cast<const Eigen::VectorXd&>(x), grad);
      idx += _n;
    }

    // Evaluate function values for inequality constraints
    for (std::size_t i = 0; i < problem->getNumIneqConstraints(); ++i)
    {
      new (&grad)Eigen::Map<Eigen::VectorXd>(_values + idx, _n);
      problem->getIneqConstraint(i)->evalGradient(
            static_cast<const Eigen::VectorXd&>(x), grad);
      idx += _n;
    }
  }

  return true;
}

//==============================================================================
bool DartTNLP::eval_h(Ipopt::Index _n,
                      const Ipopt::Number* _x,
                      bool _new_x,
                      Ipopt::Number _obj_factor,
                      Ipopt::Index _m,
                      const Ipopt::Number* _lambda,
                      bool _new_lambda,
                      Ipopt::Index _nele_hess,
                      Ipopt::Index* _iRow,
                      Ipopt::Index* _jCol,
                      Ipopt::Number* _values)
{
  // TODO(JS): Not implemented yet.
  dterr << "[DartTNLP::eval_h] Not implemented yet.\n";

  return TNLP::eval_h(_n, _x, _new_x, _obj_factor, _m, _lambda, _new_lambda,
                      _nele_hess, _iRow, _jCol, _values);
}

//==============================================================================
void DartTNLP::finalize_solution(Ipopt::SolverReturn /*_status*/,
                                 Ipopt::Index _n,
                                 const Ipopt::Number* _x,
                                 const Ipopt::Number* /*_z_L*/,
                                 const Ipopt::Number* /*_z_U*/,
                                 Ipopt::Index /*_m*/,
                                 const Ipopt::Number* /*_g*/,
                                 const Ipopt::Number* /*_lambda*/,
                                 Ipopt::Number _obj_value,
                                 const Ipopt::IpoptData* /*_ip_data*/,
                                 Ipopt::IpoptCalculatedQuantities* /*_ip_cq*/)
{
  const std::shared_ptr<Problem>& problem = mSolver->getProblem();

  // Store optimal and optimum values
  problem->setOptimumValue(_obj_value);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(_n);
  for (int i = 0; i < _n; ++i)
    x[i] = _x[i];
  problem->setOptimalSolution(x);
}

}  // namespace optimizer
}  // namespace dart
