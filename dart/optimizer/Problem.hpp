/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_OPTIMIZER_PROBLEM_HPP_
#define DART_OPTIMIZER_PROBLEM_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "dart/optimizer/Function.hpp"

namespace dart {
namespace optimizer {

/// \brief class Problem
class Problem
{
public:

  /// \brief Constructor
  explicit Problem(std::size_t _dim = 0);

  /// \brief Destructor
  virtual ~Problem() = default;

  //--------------------------- Problem Setting --------------------------------
  /// \brief Set dimension. Note: Changing the dimension will clear out the
  /// initial guess and any seeds that have been added.
  void setDimension(std::size_t _dim);

  /// \brief Get dimension
  std::size_t getDimension() const;

  /// \brief Set initial guess for opimization parameters
  void setInitialGuess(const Eigen::VectorXd& _initGuess);

  /// \brief Set initial guess for opimization parameters
  const Eigen::VectorXd& getInitialGuess() const;

  /// \brief Add a seed for the Solver to use as a hint for the neighborhood of
  /// the solution.
  void addSeed(const Eigen::VectorXd& _seed);

  /// \brief Get a mutable reference of the seed for the specified index. If an
  /// out-of-bounds index is provided a warning will print, and a reference to
  /// the initial guess will be returned instead.
  Eigen::VectorXd& getSeed(std::size_t _index);

  /// \brief An immutable version of getSeed(std::size_t)
  const Eigen::VectorXd& getSeed(std::size_t _index) const;

  /// \brief Get a mutable reference to the full vector of seeds that this
  /// Problem currently contains
  std::vector<Eigen::VectorXd>& getSeeds();

  /// \brief An immutable version of getSeeds()
  const std::vector<Eigen::VectorXd>& getSeeds() const;

  /// \brief Clear the seeds that this Problem currently contains
  void clearAllSeeds();

  /// \brief Set lower bounds for optimization parameters
  void setLowerBounds(const Eigen::VectorXd& _lb);

  /// \brief Get lower bounds for optimization parameters
  const Eigen::VectorXd& getLowerBounds() const;

  /// \brief Set upper bounds for optimization parameters
  void setUpperBounds(const Eigen::VectorXd& _ub);

  /// \brief Get upper bounds for optimization parameters
  const Eigen::VectorXd& getUpperBounds() const;

  /// \brief Set minimum objective function
  void setObjective(FunctionPtr _obj);

  /// \brief Get objective function
  FunctionPtr getObjective() const;

  /// \brief Add equality constraint
  void addEqConstraint(FunctionPtr _eqConst);

  /// \brief Add inequality constraint. Inequality constraints must evaluate
  /// to LESS THAN or equal to zero (within some tolerance) to be satisfied.
  void addIneqConstraint(FunctionPtr _ineqConst);

  /// \brief Get number of equality constraints
  std::size_t getNumEqConstraints() const;

  /// \brief Get number of inequality constraints
  std::size_t getNumIneqConstraints() const;

  /// \brief Get equality constraint
  FunctionPtr getEqConstraint(std::size_t _idx) const;

  /// \brief Get inequality constraint
  FunctionPtr getIneqConstraint(std::size_t _idx) const;

  /// \brief Remove equality constraint
  void removeEqConstraint(FunctionPtr _eqConst);

  /// \brief Remove inequality constraint
  void removeIneqConstraint(FunctionPtr _ineqConst);

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
  std::size_t mDimension;

  /// \brief Initial guess for optimization parameters
  Eigen::VectorXd mInitialGuess;

  /// \brief Additional guess hints for the Solver.
  std::vector<Eigen::VectorXd> mSeeds;

  /// \brief Lower bounds for optimization parameters
  Eigen::VectorXd mLowerBounds;

  /// \brief Upper bounds for optimization parameters
  Eigen::VectorXd mUpperBounds;

  /// \brief Objective function
  FunctionPtr mObjective;

  /// \brief Equality constraint functions
  std::vector<FunctionPtr> mEqConstraints;

  /// \brief Inequality constraint functions
  std::vector<FunctionPtr> mIneqConstraints;

  /// \brief Optimal objective value
  double mOptimumValue;

  /// \brief Optimal solution
  Eigen::VectorXd mOptimalSolution;
};

} // namespace optimizer
} // namespace dart

#endif // #ifndef DART_OPTIMIZER_PROBLEM_HPP_

