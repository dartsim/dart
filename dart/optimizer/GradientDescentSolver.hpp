/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_OPTIMIZER_GRADIENTDESCENTSOLVER_HPP_
#define DART_OPTIMIZER_GRADIENTDESCENTSOLVER_HPP_

#include <random>

#include "dart/optimizer/Solver.hpp"

namespace dart {
namespace optimizer {

/// DefaultSolver is a Solver extension which is native to DART (rather than
/// relying on third-party libraries). It uses randomized gradient descent and
/// softened constraints (i.e. constraint functions are added into the
/// objective function and assigned weights) to solve nonlinear problems. Note
/// that this is not a good option for Problems with difficult constraint
/// functions that need to be solved exactly.
class GradientDescentSolver : public Solver
{
public:

  static const std::string Type;

  struct UniqueProperties
  {
    /// Value of the fixed step size
    double mStepSize;

    /// Number of attempts to make before quitting. Each attempt will start from
    /// the next seed provided by the problem. Once there are no more seeds,
    /// random starting configurations will be used.
    ///
    /// Set this to 0 to keep trying until a solution is found (the program will
    /// need to be interrupted in order to stop if no solution is being found).
    std::size_t mMaxAttempts;

    /// The number of steps between random perturbations being applied to the
    /// configuration. Set this to 0 to never apply randomized perturbations.
    std::size_t mPerturbationStep;

    /// The random perturbation works as follows: A random point in the domain
    /// of the Problem is selected, and then a random step size between 0 and
    /// mMaxPerturbationFactor is selected. The configuration will take a step
    /// of that random step size towards the random point.
    ///
    /// A maximum value of 1.0 is recommended for mMaxPerturbationFactor. A
    /// smaller value will result in smaller randomized perturbations. A value
    /// significantly larger than 1.0 could bias the configuration towards the
    /// boundary of the Problem domain.
    double mMaxPerturbationFactor;

    /// The largest permittable change in value when randomizing a configuration
    double mMaxRandomizationStep;

    /// This is the weight that will be applied to any constraints that do not
    /// have a corresponding weight specified by mEqConstraintWeights or by
    /// mIneqConstraintWeights
    double mDefaultConstraintWeight;

    /// Vector of weights that should be applied to the equality constraints.
    /// If there are fewer components in this vector than there are equality
    /// constraints in the Problem, then the remaining equality constraints will
    /// be assigned a weight of mDefaultConstraintWeight.
    Eigen::VectorXd mEqConstraintWeights;

    /// Vector of weights that should be applied to the inequality constraints.
    /// If there are fewer components in this vector than there are inequality
    /// constraints in the Problem, then the remaining inequality constraints
    /// will be assigned a weight of mDefaultConstraintWeight.
    Eigen::VectorXd mIneqConstraintWeights;

    UniqueProperties(
        double _stepMultiplier = 0.1,
        std::size_t _maxAttempts = 1,
        std::size_t _perturbationStep = 0,
        double _maxPerturbationFactor = 1.0,
        double _maxRandomizationStep = 1e10,
        double _defaultConstraintWeight = 1.0,
        Eigen::VectorXd _eqConstraintWeights = Eigen::VectorXd(),
        Eigen::VectorXd _ineqConstraintWeights = Eigen::VectorXd() );
  };

  struct Properties : Solver::Properties, UniqueProperties
  {
    Properties(
        const Solver::Properties& _solverProperties = Solver::Properties(),
        const UniqueProperties& _descentProperties = UniqueProperties() );
  };

  /// Default constructor
  explicit GradientDescentSolver(const Properties& _properties = Properties());

  /// Alternative constructor
  explicit GradientDescentSolver(std::shared_ptr<Problem> _problem);

  /// Destructor
  virtual ~GradientDescentSolver();

  // Documentation inherited
  bool solve() override;

  /// Get the last configuration that was used by the Solver
  Eigen::VectorXd getLastConfiguration() const;

  // Documentation inherited
  std::string getType() const override;

  // Documentation inherited
  std::shared_ptr<Solver> clone() const override;

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

  /// Set the multiplier for the step size
  void setStepSize(double _newMultiplier);

  /// Get the multiplier for the step size
  double getStepSize() const;

  /// Set the maximum number of solving attempts before quitting. Each attempt
  /// will use getNumMaxIterations() steps. When a new attempt is started, it
  /// will use the next seed in the list of seeds. If we've reached the end of
  /// the list of seeds, the attempt will start from a randomized configuration.
  void setMaxAttempts(std::size_t _maxAttempts);

  /// Get the maximum number of solving attempts.
  std::size_t getMaxAttempts() const;

  /// Set the number of steps that will be taken before applying a randomized
  /// perturbation.
  void setPerturbationStep(std::size_t _step);

  /// Get UniqueProperties::mPerturbationStep
  std::size_t getPerturbationStep() const;

  /// Set UniqueProperties::mPerturbationFactor
  void setMaxPerturbationFactor(double _factor);

  /// Get UniqueProperties::mPerturbationFactor
  double getMaxPerturbationFactor() const;

  /// Set UniqueProperties::mDefaultConstraintWeight
  void setDefaultConstraintWeight(double _newDefault);

  /// Get UniqueProperties::mDefaultConstraintWeight
  double getDefaultConstraintWeight() const;

  /// Set UniqueProperties::mEqConstraintWeights
  Eigen::VectorXd& getEqConstraintWeights();

  /// Get UniqueProperties::mEqConstraintWeights
  const Eigen::VectorXd& getEqConstraintWeights() const;

  /// Set UniqueProperties::mIneqConstraintWeights
  Eigen::VectorXd& getIneqConstraintWeights();

  /// Get UniqueProperties::mIneqConstraintWeights
  const Eigen::VectorXd& getIneqConstraintWeights() const;

  /// Randomize the configuration based on this Solver's settings
  void randomizeConfiguration(Eigen::VectorXd& _x);

  /// Clamp the configuration to the limits of the Problem
  void clampToBoundary(Eigen::VectorXd& _x);

  /// Get the number of iterations used in the last attempt to solve the problem
  std::size_t getLastNumIterations() const;

protected:

  /// GradientDescentSolver properties
  UniqueProperties mGradientP;

  /// The last number of iterations performed by this Solver
  std::size_t mLastNumIterations;

  /// Randomization device
  std::random_device mRD;

  /// Mersenne twister method
  std::mt19937 mMT;

  /// Distribution
  std::uniform_real_distribution<double> mDistribution;

  /// Cache to track the costs of equality constraints
  Eigen::VectorXd mEqConstraintCostCache;

  /// Cache to track the costs of inequality constraints
  Eigen::VectorXd mIneqConstraintCostCache;

  /// The last config reached by this Solver
  Eigen::VectorXd mLastConfig;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_GRADIENTDESCENTSOLVER_HPP_
