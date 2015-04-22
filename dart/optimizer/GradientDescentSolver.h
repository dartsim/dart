/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_OPTIMIZER_GRADIENTDESCENTSOLVER_H_
#define DART_OPTIMIZER_GRADIENTDESCENTSOLVER_H_

#include "dart/optimizer/Solver.h"

namespace dart {
namespace optimizer {

/// DefaultSolver is a Solver extension which is native to DART (rather than
/// relying on third-party libraries). It uses stochastic gradient descent and
/// softened constraints (i.e. constraint functions are worked into the
/// objective function and assigned weights) to solve nonlinear problems.
class GradientDescentSolver : public Solver
{
public:

  static const std::string Type;

  struct UniqueProperties
  {
    double mStepMultiplier;

    double mMaxStepLength;

    size_t mMaxAttempts;

    double mDefaultConstraintWeight;

    Eigen::VectorXd mConstraintWeights;

    UniqueProperties(
        double _stepMultiplier = 0.1,
        double _maxStepLength = INFINITY,
        size_t _maxAttempts = 1,
        double _defaultConstraintWeight = 1.0,
        Eigen::VectorXd _constraintWeights = Eigen::VectorXd() );
  };

  struct Properties : Solver::Properties, UniqueProperties
  {
    Properties(
        const Solver::Properties& _solverProperties = Solver::Properties(),
        const UniqueProperties& _descentProperties = UniqueProperties() );
  };

  explicit GradientDescentSolver(const Properties& _properties = Properties());

  explicit GradientDescentSolver(std::shared_ptr<Problem> _problem);

  virtual ~GradientDescentSolver();

  virtual bool solve() override;

  virtual std::string getType() const override;

  // Documentation inherited
  virtual std::shared_ptr<Solver> clone() const override;

  /// Set the Properties of this GradientDescentSolver
  void setProperties(const Properties& _properties);

  /// Set the Properties of this GradientDescentSolver
  void setProperties(const UniqueProperties& _properties);

  /// Get the Properties of this GradientDescentSolver
  Properties getGradientDescentProperties() const;

  /// Copy the Properties of another GradientDescentSolver
  void copy(const GradientDescentSolver& _other);

  /// Copy the Properties of another GradientDescentSolver
  GradientDescentSolver& operator=(const GradientDescentSolver& _other);

  void setStepMultiplier(double _newMultiplier);

  double getStepMultiplier() const;

  void setMaxStepLength(double _newLength);

  double getMaxStepLength() const;

  void setMaxAttempts(size_t _maxAttempts);

  size_t getMaxAttempts() const;

  void setDefaultConstraintWeight(double _newDefault);

  double getDefaultConstraintWeight() const;

  Eigen::VectorXd& getConstraintWeights();

  const Eigen::VectorXd& getConstraintWeights() const;

protected:

  UniqueProperties mGradientP;

};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_GRADIENTDESCENTSOLVER_H_
