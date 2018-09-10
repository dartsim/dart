/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_OPTIMIZER_MCMCMULTIOBJECTIVESOLVER_HPP_
#define DART_OPTIMIZER_MCMCMULTIOBJECTIVESOLVER_HPP_

#include "dart/optimizer/MultiObjectiveSolver.hpp"

namespace dart {
namespace optimizer {

class MultiObjectiveProblem;

struct FitnessEvaluator
{
  virtual void evaluate(const Population& pop, Eigen::VectorXd& fitness) const = 0;
};

struct HyperVolumeFitnessEvaluator : FitnessEvaluator
{
  using BoolArray = Eigen::Matrix<bool, Eigen::Dynamic, 1>;
  mutable BoolArray isSolutionInParetoFront;
  mutable Eigen::MatrixXd dominanceMatrix;

  void evaluate(const Population& pop, Eigen::VectorXd& dominanceFitness) const override;
};

/// Implementation of Markov chain Monte Carlo method for multi-objective
/// optimization problems.
class MomcmcSolver : public MultiObjectiveSolver
{
public:
  struct UniqueProperties
  {
    /// Simulated temparature
    double mTemp;

    /// Lower bound for the simulated temparature.
    double mMinTemp;

    /// Upper bound for the simulated temparature.
    double mMaxTemp;

    /// Desired acceptance rate, which is used for controlling the simulated
    /// temperature. The temperature will be increased by 10% if the acceptance
    /// rate (AR) of the samples in a new population is lower than the desired
    /// AR, and it will be decreased by 10% if the AR is greater than the
    /// desired AR. Otherwise, the simulated temperature will be unchanged.
    double mDesiredAR;

    /// Constructor
    UniqueProperties(
        double temperature = 1.0,
        double minTemperature = 0.0,
        double maxTemperature = 1000.0,
        double desiredAcceptanceRate = 0.01);
  };

  struct Properties : MultiObjectiveSolver::Properties, UniqueProperties
  {
    Properties(
        const MultiObjectiveSolver::Properties& parentProperties
        = MultiObjectiveSolver::Properties(),
        const UniqueProperties& uniqueProperties = UniqueProperties());
  };

  /// Default Constructor
  MomcmcSolver(const Properties& properties = Properties());

  /// Alternative Constructor
  MomcmcSolver(std::shared_ptr<MultiObjectiveProblem> problem);

  /// Destructor
  ~MomcmcSolver() override;

  // Documentation inherited
  bool solve(std::size_t numEvolutions = 1u) override;

  // Documentation inherited
  std::string getType() const override;

  // Documentation inherited
  std::shared_ptr<MultiObjectiveSolver> clone() const override;

  /// Sets the Properties of this McmcMultiObjectiveSolver
  void setProperties(const Properties& properties);

  /// Sets the Properties of this McmcMultiObjectiveSolver
  void setProperties(const UniqueProperties& properties);

  /// Returns the Properties of this GradientDescentSolver
  Properties getGradientDescentProperties() const;

  /// Copies the Properties of another McmcMultiObjectiveSolver
  void copy(const MomcmcSolver& other);

  /// Copies the Properties of another McmcMultiObjectiveSolver
  MomcmcSolver& operator=(const MomcmcSolver& other);

protected:
  /// McmcMultiObjectiveSolver properties
  UniqueProperties mUniqueP;

  /// Distribution
  std::uniform_real_distribution<double> mDistribution;

private:
  // using BoolArray = Eigen::Array<bool, Eigen::Dynamic, 1>;
  using BoolArray = Eigen::Matrix<bool, Eigen::Dynamic, 1>;

  void computeDominance(
      const Population& pop,
      BoolArray& isSolutionInParetoFront,
      Eigen::MatrixXd& popDominanceMatrix);

  void computeDominanceFitness(
      const BoolArray& isSolutionInParetoFront,
      const Eigen::MatrixXd& dominanceMatrix,
      Eigen::VectorXd& dominanceFitness);

  bool goodToAcceptNewSample(
      double oldDominanceFitness,
      double newDominanceFitness,
      double temperature,
      double randomVar);

  void increaseTemperature();
  void decreaseTemperature();

  void evolution(Population& pop, Population& newPop);

  void randomEvolution(const Population& pop, Population& newPop);
  // void differentialEvolution(Population& pop, Population& newPop);

  BoolArray mPopNotDominated;
  Eigen::MatrixXd mDominanceMatrix;
  Eigen::VectorXd mDominanceFitnesses;

  std::vector<Population> mNewPopulations;
  BoolArray mNewPopNotDominated;
  Eigen::MatrixXd mNewDominanceMatrix;
  Eigen::VectorXd mNewDominanceFitnesses;

  HyperVolumeFitnessEvaluator mFitnessEvaluator;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_MCMCMULTIOBJECTIVESOLVER_HPP_
