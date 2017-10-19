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

#ifndef DART_OPTIMIZER_IPOPT_IPOPTSOLVER_HPP_
#define DART_OPTIMIZER_IPOPT_IPOPTSOLVER_HPP_

//------------------------------------------------------------------------------
// Workaround for bug:
// (see https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=684062)
// This is fixed at IpOpt 3.11.4-1
#define HAVE_CSTDDEF
#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>
#undef HAVE_CSTDDEF
//------------------------------------------------------------------------------

#include <memory>

#include "dart/optimizer/Solver.hpp"

namespace dart {
namespace optimizer {

class Problem;
class DartTNLP;

/// \brief class IpoptSolver
class IpoptSolver : public Solver
{
public:

  /// Default constructor
  IpoptSolver(const Solver::Properties& _properties = Solver::Properties());

  /// Alternative Constructor
  explicit IpoptSolver(std::shared_ptr<Problem> _problem);

  /// Destructor
  virtual ~IpoptSolver();

  // Documentation inherited
  bool solve() override;

  // Documentation inherited
  std::string getType() const override;

  // Documentation inherited
  std::shared_ptr<Solver> clone() const override;

  /// Get the application interface for this IpoptSolver
  const Ipopt::SmartPtr<Ipopt::IpoptApplication>& getApplication();

  /// Get a const application interface for this IpoptSolver
  Ipopt::SmartPtr<const Ipopt::IpoptApplication> getApplication() const;

private:

  /// Constructor used during cloning
  IpoptSolver(const Properties& _properties,
              const Ipopt::SmartPtr<Ipopt::IpoptApplication>& _app);

  /// IPOPT nonlinear programming problem
  Ipopt::SmartPtr<Ipopt::TNLP> mNlp;

  /// Main application class for making calls to Ipopt
  Ipopt::SmartPtr<Ipopt::IpoptApplication> mIpoptApp;
};

/// class DartTNLP
class DartTNLP : public Ipopt::TNLP
{
public:

  friend class IpoptSolver;

  /// \brief
  virtual ~DartTNLP();

  //------------------------- Ipopt::TNLP --------------------------------------
  /// \brief Method to return some info about the nlp
  bool get_nlp_info(Ipopt::Index& n,
                    Ipopt::Index& m,
                    Ipopt::Index& nnz_jac_g,
                    Ipopt::Index& nnz_h_lag,
                    Ipopt::TNLP::IndexStyleEnum& index_style) override;

  /// \brief Method to return the bounds for my problem
  bool get_bounds_info(Ipopt::Index n,
                       Ipopt::Number* x_l,
                       Ipopt::Number* x_u,
                       Ipopt::Index m,
                       Ipopt::Number* g_l,
                       Ipopt::Number* g_u) override;

  /// \brief Method to return the starting point for the algorithm
  bool get_starting_point(Ipopt::Index n,
                          bool init_x,
                          Ipopt::Number* x,
                          bool init_z,
                          Ipopt::Number* z_L,
                          Ipopt::Number* z_U,
                          Ipopt::Index m,
                          bool init_lambda,
                          Ipopt::Number* lambda) override;

  /// \brief Method to return the objective value
  bool eval_f(Ipopt::Index _n,
              const Ipopt::Number* _x,
              bool _new_x,
              Ipopt::Number&
              _obj_value) override;

  /// \brief Method to return the gradient of the objective
  bool eval_grad_f(Ipopt::Index _n,
                   const Ipopt::Number* _x,
                   bool _new_x,
                   Ipopt::Number* _grad_f) override;

  /// \brief Method to return the constraint residuals
  bool eval_g(Ipopt::Index _n,
              const Ipopt::Number* _x,
              bool _new_x,
              Ipopt::Index _m,
              Ipopt::Number* _g) override;

  /// \brief Method to return:
  ///        1) The structure of the jacobian (if "values" is nullptr)
  ///        2) The values of the jacobian (if "values" is not nullptr)
  bool eval_jac_g(Ipopt::Index _n,
                  const Ipopt::Number* _x,
                  bool _new_x,
                  Ipopt::Index _m,
                  Ipopt::Index _nele_jac,
                  Ipopt::Index* _iRow,
                  Ipopt::Index* _jCol,
                  Ipopt::Number* _values) override;

  /// \brief Method to return:
  ///        1) The structure of the hessian of the lagrangian (if "values" is
  ///           nullptr)
  ///        2) The values of the hessian of the lagrangian (if "values" is not
  ///           nullptr)
  bool eval_h(Ipopt::Index _n,
              const Ipopt::Number* _x,
              bool _new_x,
              Ipopt::Number _obj_factor,
              Ipopt::Index _m,
              const Ipopt::Number* _lambda,
              bool _new_lambda,
              Ipopt::Index _nele_hess,
              Ipopt::Index* _iRow,
              Ipopt::Index* _jCol,
              Ipopt::Number* _values) override;

  /// \brief This method is called when the algorithm is complete so the TNLP
  ///        can store/write the solution
  void finalize_solution(Ipopt::SolverReturn _status,
                         Ipopt::Index _n,
                         const Ipopt::Number* _x,
                         const Ipopt::Number* _z_L,
                         const Ipopt::Number* _z_U,
                         Ipopt::Index _m,
                         const Ipopt::Number* _g,
                         const Ipopt::Number* _lambda,
                         Ipopt::Number _obj_value,
                         const Ipopt::IpoptData* _ip_data,
                         Ipopt::IpoptCalculatedQuantities* _ip_cq) override;

private:

  /// \brief
  explicit DartTNLP(IpoptSolver* _solver);

  /// \brief DART optimization problem
  IpoptSolver* mSolver;

  /// \brief Objective value
  Ipopt::Number mObjValue;

  /// \brief Objective gradient
  Eigen::VectorXd mObjGradient;

  /// \brief Objective Hessian
  Eigen::MatrixXd mObjHessian;
};

}  // namespace optimizer
}  // namespace dart

#endif  // DART_OPTIMIZER_IPOPT_IPOPTSOLVER_HPP_
