/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_OPTIMIZER_PROBLEM_H
#define DART_OPTIMIZER_PROBLEM_H

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace dart {
namespace optimizer {

class Function;

/// \brief class Problem
class Problem
{
public:
  /// \brief Constructor
  explicit Problem(size_t _dim);

  /// \brief Destructor
  virtual ~Problem();

  //--------------------------- Problem Setting --------------------------------
  /// \brief Set dimension
  void setDimension(size_t _dim);

  /// \brief Get dimension
  size_t getDimension() const;

  /// \brief Set initial guess for opimization parameters
  void setInitialGuess(const Eigen::VectorXd& _initGuess);

  /// \brief Set initial guess for opimization parameters
  const Eigen::VectorXd& getInitialGuess() const;

  /// \brief Set lower bounds for optimization parameters
  void setLowerBounds(const Eigen::VectorXd& _lb);

  /// \brief Get lower bounds for optimization parameters
  const Eigen::VectorXd& getLowerBounds() const;

  /// \brief Set upper bounds for optimization parameters
  void setUpperBounds(const Eigen::VectorXd& _ub);

  /// \brief Get upper bounds for optimization parameters
  const Eigen::VectorXd& getUpperBounds() const;

  /// \brief Set minimum objective function
  void setObjective(Function* _obj);

  /// \brief Get objective function
  Function* getObjective() const;

  /// \brief Add equality constraint
  void addEqConstraint(Function* _eqConst);

  /// \brief Add inequality constraint
  void addIneqConstraint(Function* _ineqConst);

  /// \brief Get number of equality constraints
  size_t getNumEqConstraints() const;

  /// \brief Get number of inequality constraints
  size_t getNumIneqConstraints() const;

  /// \brief Get equality constraint
  Function* getEqConstraint(size_t _idx) const;

  /// \brief Get inequality constraint
  Function* getIneqConstraint(size_t _idx) const;

  /// \brief Remove equality constraint
  void removeEqConstraint(Function* _eqConst);

  /// \brief Remove inequality constraint
  void removeIneqConstraint(Function* _ineqConst);

  /// \brief Remove all equality constraints
  void removeAllEqConstraints();

  /// \brief Remove all inequality constraints
  void removeAllIneqConstraints();

  //------------------------------ Result --------------------------------------
  /// \brief Set optimum value of the objective function. This function called
  ///        by Solver.
  void setOptimumValue(double _val);

  /// \brief Get optimum value of the objective function
  double getOptimumValue() const;

  /// \brief Set optimal solution. This function called by Solver.
  void setOptimalSolution(const Eigen::VectorXd& _optParam);

  /// \brief Get optimal solution
  const Eigen::VectorXd& getOptimalSolution();

protected:
  /// \brief Dimension of this problem
  size_t mDimension;

  /// \brief Initial guess for optimization parameters
  Eigen::VectorXd mInitialGuess;

  /// \brief Lower bounds for optimization parameters
  Eigen::VectorXd mLowerBounds;

  /// \brief Upper bounds for optimization parameters
  Eigen::VectorXd mUpperBounds;

  /// \brief Objective function
  Function* mObjective;

  /// \brief Equality constraint functions
  std::vector<Function*> mEqConstraints;

  /// \brief Inequality constraint functions
  std::vector<Function*> mIneqConstraints;

  /// \brief Optimal objective value
  double mOptimumValue;

  /// \brief Optimal solution
  Eigen::VectorXd mOptimalSolution;
};

} // namespace optimizer
} // namespace dart

#endif // #ifndef DART_OPTIMIZER_PROBLEM_H

