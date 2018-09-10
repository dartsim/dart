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

#include "dart/optimizer/McmcMultiObjectiveSolver.hpp"

#include <memory>

#include <Eigen/Dense>

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"
#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimizer {

namespace {

//==============================================================================
enum class PairRelationType
{
  DOMINATING = 0,
  DOMINATED,
  INDIFFERENT
};

//==============================================================================
PairRelationType compare(const Eigen::VectorXd& a1, const Eigen::VectorXd& a2)
{
  const Eigen::VectorXd delta = a2 - a1;

  if ((delta.array() >= 0.0).all())
    return PairRelationType::DOMINATING;
  else if ((delta.array() <= 0.0).all())
    return PairRelationType::DOMINATED;

  return PairRelationType::INDIFFERENT;
}

//==============================================================================
double calcAcceptanceProbLog(double prevFitness, double nextFitness, double mT)
{
  assert(mT > 0.0);
  return (prevFitness - nextFitness) / mT;
}

} // (anonymous) namespace

//==============================================================================
MomcmcSolver::UniqueProperties::UniqueProperties(
    double temperature,
    double minTemperature,
    double maxTemperature,
    double desiredAcceptanceRate)
  : mTemp(temperature),
    mMinTemp(minTemperature),
    mMaxTemp(maxTemperature),
    mDesiredAR(desiredAcceptanceRate)
{
  // Do nothing
}

//==============================================================================
MomcmcSolver::Properties::Properties(
    const MultiObjectiveSolver::Properties& parentProperties,
    const MomcmcSolver::UniqueProperties& uniqueProperties)
  : MultiObjectiveSolver::Properties(parentProperties),
    UniqueProperties(uniqueProperties)
{
  // Do nothing
}

//==============================================================================
MomcmcSolver::MomcmcSolver(const Properties& properties)
  : MultiObjectiveSolver(properties), mUniqueP(properties)
{
  // Do nothing
}

//==============================================================================
MomcmcSolver::MomcmcSolver(std::shared_ptr<MultiObjectiveProblem> problem)
  : MultiObjectiveSolver(std::move(problem))
{
  // Do nothing
}

//==============================================================================
MomcmcSolver::~MomcmcSolver()
{
  // Do nothing
}

//==============================================================================
bool MomcmcSolver::solve(std::size_t numEvolutions)
{
  // TODO(JS): Improve performace by re-using prob, archi, and populations.

  const std::shared_ptr<MultiObjectiveProblem>& prob = mProperties.mProblem;
  if (!prob)
    return true;

  const auto numPopulations = getNumPopulations();
  const auto numSolutions = getNumSolutions();

  mPopulations.resize(numPopulations, {*prob, numSolutions});
  mNewPopulations.resize(numPopulations, {*prob, numSolutions});
  // TODO(JS): Use a flag of whether to evolve from the existing population
  // or random populations

  for (std::size_t i = 0u; i < numPopulations; ++i)
  {
    Population& pop = mPopulations[i];
    Population& newPop = mNewPopulations[i];

    pop.setRandom(*prob, getNumSolutions());

    for (std::size_t j = 0u; j < numEvolutions; ++j)
    {
      computeDominance(pop, mPopNotDominated, mDominanceMatrix);
      computeDominanceFitness(
          mPopNotDominated, mDominanceMatrix, mDominanceFitnesses);
      evolution(pop, newPop);
    }
  }

  return true;
}

//==============================================================================
std::string MomcmcSolver::getType() const
{
  return "McmcMultiObjectiveSolver";
}

//==============================================================================
std::shared_ptr<MultiObjectiveSolver> MomcmcSolver::clone() const
{
  return std::make_shared<MomcmcSolver>(getSolverProperties());
}

//==============================================================================
void MomcmcSolver::setProperties(const Properties& properties)
{
  MultiObjectiveSolver::setProperties(properties);
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void MomcmcSolver::setProperties(const UniqueProperties& properties)
{
  mUniqueP = properties;
}

//==============================================================================
MomcmcSolver::Properties MomcmcSolver::getGradientDescentProperties() const
{
  return Properties(getSolverProperties(), mUniqueP);
}

//==============================================================================
void MomcmcSolver::copy(const MomcmcSolver& other)
{
  setProperties(other.getSolverProperties());
}

//==============================================================================
MomcmcSolver& MomcmcSolver::operator=(const MomcmcSolver& other)
{
  copy(other);
  return *this;
}

//==============================================================================
void MomcmcSolver::computeDominance(
    const Population& pop,
    BoolArray& isSolutionInParetoFront,
    Eigen::MatrixXd& popDominanceMatrix)
{
  const auto numSolutions = getNumSolutions();

  isSolutionInParetoFront.resize(static_cast<int>(numSolutions));
  popDominanceMatrix.resize(
      static_cast<int>(numSolutions), static_cast<int>(numSolutions));

  // Initiall we assume all the samples are not dominated.
  isSolutionInParetoFront.setConstant(true); // N x 1
  assert(
      static_cast<std::size_t>(isSolutionInParetoFront.size()) == numSolutions);
  popDominanceMatrix.setZero(); // N x N
  assert(static_cast<std::size_t>(popDominanceMatrix.rows()) == numSolutions);
  assert(static_cast<std::size_t>(popDominanceMatrix.cols()) == numSolutions);

  Eigen::VectorXd iFitness;
  Eigen::VectorXd jFitness;
  for (auto i = 0u; i < numSolutions - 1u; ++i)
  {
    iFitness = pop.getFitness(i);

    for (auto j = i + 1u; j < numSolutions; ++j)
    {
      jFitness = pop.getFitness(j);

      const auto relationType = compare(iFitness, jFitness);

      // iObjs is dominated by jObjs.
      if (PairRelationType::DOMINATED == relationType)
      {
        isSolutionInParetoFront[i] = false;
        const auto dominanceVal = (iFitness - jFitness).norm();
        popDominanceMatrix(j, i) = -dominanceVal;
        popDominanceMatrix(i, j) = dominanceVal;
      }
      // currObjs is dominating nextObjs.
      else if (PairRelationType::DOMINATING == relationType)
      {
        isSolutionInParetoFront[j] = false;
        const auto dominanceVal = (iFitness - jFitness).norm();
        popDominanceMatrix(i, j) = -dominanceVal;
        popDominanceMatrix(j, i) = dominanceVal;
      }
    }
  }

  static Eigen::RowVectorXd zeroVector;
  zeroVector.setZero(static_cast<int>(numSolutions));

  // TODO(JS): We could make this optional.
  // Discard hyper volume, which is positive, between samples that both of them
  // are not in the Pareto front.
  for (auto i = 0; i < static_cast<int>(numSolutions); ++i)
  {
    if (isSolutionInParetoFront[i])
      continue;

    popDominanceMatrix.row(i) = popDominanceMatrix.row(i).cwiseMax(zeroVector);
  }
}

//==============================================================================
void MomcmcSolver::computeDominanceFitness(
    const BoolArray& isSolutionInParetoFront,
    const Eigen::MatrixXd& dominanceMatrix,
    Eigen::VectorXd& dominanceFitness)
{
  const auto numSolutions = getNumSolutions();
  static Eigen::VectorXd zeroVector
      = Eigen::VectorXd::Zero(static_cast<int>(numSolutions));

  dominanceFitness.resize(static_cast<int>(numSolutions));

  for (auto i = 0u; i < numSolutions; ++i)
  {
    const Eigen::VectorXd dominanceRow = dominanceMatrix.row(i);
    if (isSolutionInParetoFront[i])
    {
      // Number of elements with negative value is the number of samples
      // dominated by i-th sample.
      const auto dominanceCount = (dominanceRow.array() < 0.0).count();

      // dominanceRow should only contain zero or negative.
      assert((dominanceRow.array() > 0.0).count() == 0);

      // Simply, the strength value is the fitness.
      dominanceFitness[i] = double(dominanceCount) / double(numSolutions);

      assert(dominanceFitness[i] < 1.0);
      assert(dominanceFitness[i] >= 0.0);
    }
    else
    {
      //      const Eigen::VectorXd dominating =
      //      dominanceRow.cwiseMax(zeroVector);
      assert((dominanceRow.array() >= 0.0).all());
      const double elementsDominating = dominanceRow.sum();
      dominanceFitness[i] = 1.0 + elementsDominating;
      // Consider only summing up the distance between i-th element against
      // samples in notDominated set dominate i-th element.

      assert(dominanceFitness[i] >= 1.0);
    }
  }
}

#define DART_MOMCMC_EPS 1e-9

//==============================================================================
bool MomcmcSolver::goodToAcceptNewSample(
    double oldDominanceFitness,
    double newDominanceFitness,
    double temperature,
    double randomVar)
{
  if (temperature < DART_MOMCMC_EPS)
  {
    if (oldDominanceFitness < newDominanceFitness)
      return false;
    else
      return true;
  }

  const double acceptProbLog = calcAcceptanceProbLog(
      oldDominanceFitness, newDominanceFitness, temperature);

  if (acceptProbLog > std::log(randomVar))
    return true;
  else
    return false;
}

//==============================================================================
void MomcmcSolver::increaseTemperature()
{
  MomcmcSolver::mUniqueP.mTemp *= 1.1;

  const auto& min = MomcmcSolver::mUniqueP.mMinTemp;
  const auto& max = MomcmcSolver::mUniqueP.mMaxTemp;
  MomcmcSolver::mUniqueP.mTemp
      = math::clip(MomcmcSolver::mUniqueP.mTemp, min, max);
}

//==============================================================================
void MomcmcSolver::decreaseTemperature()
{
  MomcmcSolver::mUniqueP.mTemp *= 0.9;

  const auto& min = MomcmcSolver::mUniqueP.mMinTemp;
  const auto& max = MomcmcSolver::mUniqueP.mMaxTemp;
  MomcmcSolver::mUniqueP.mTemp
      = math::clip(MomcmcSolver::mUniqueP.mTemp, min, max);
}

//==============================================================================
void MomcmcSolver::evolution(Population& pop, Population& newPop)
{
  const auto numSolutions = getNumSolutions();

  static Population tempPop;

  int acceptedCount = 0;
  Eigen::VectorXd theta(numSolutions);
  for (auto i = 0u; i < getNumIterationsPerEvolution(); ++i)
  {
    randomEvolution(pop, newPop);

    // TODO(JS): This is wrong!!
    computeDominance(newPop, mNewPopNotDominated, mNewDominanceMatrix);
    computeDominanceFitness(
        mNewPopNotDominated, mNewDominanceMatrix, mNewDominanceFitnesses);

    acceptedCount = 0;
    theta = math::Random::uniform<Eigen::VectorXd>(
        static_cast<int>(numSolutions), 0.0, 1.0);
    assert((theta.array() <= 1.0).all());
    assert((theta.array() >= 0.0).all());

    for (std::size_t j = 0; j < numSolutions; ++j)
    {
      tempPop = pop;
      tempPop.set(j, newPop.getSolution(j), newPop.getFitness(j));

      computeDominance(tempPop, mNewPopNotDominated, mNewDominanceMatrix);
      computeDominanceFitness(
          mNewPopNotDominated, mNewDominanceMatrix, mNewDominanceFitnesses);

      const auto domFit = mDominanceFitnesses[static_cast<int>(j)];
      const auto newDomFit = mNewDominanceFitnesses[static_cast<int>(j)];
      const auto temp = MomcmcSolver::mUniqueP.mTemp;
      const auto randomVar = theta[static_cast<int>(j)];

      if (goodToAcceptNewSample(domFit, newDomFit, temp, randomVar))
      {
        acceptedCount++;
      }
      else
      {
        newPop.set(j, pop.getSolution(j), pop.getFitness(j));
        mNewDominanceFitnesses[j] = mDominanceFitnesses[j];
      }
    }

//    const auto sum = mDominanceFitnesses.sum();

    pop = newPop;

    computeDominance(pop, mPopNotDominated, mDominanceMatrix);
    computeDominanceFitness(
        mPopNotDominated, mDominanceMatrix, mDominanceFitnesses);

//    mDominanceFitnesses = mNewDominanceFitnesses;

    const double AR = double(acceptedCount) / double(numSolutions);
    if (AR < MomcmcSolver::mUniqueP.mDesiredAR)
      increaseTemperature();
    else if (AR > MomcmcSolver::mUniqueP.mDesiredAR)
      decreaseTemperature();

    std::cout << i << ": ";
    std::cout << "sum: " << mDominanceFitnesses.sum() << " (AR) " << AR
              << " (T) " << MomcmcSolver::mUniqueP.mTemp << "\n";
  }
}

//==============================================================================
void MomcmcSolver::randomEvolution(const Population& pop, Population& newPop)
{
  auto prob = getProblem();
  newPop.setRandom(*prob, pop.getNumSolutions());
}

//==============================================================================
void HyperVolumeFitnessEvaluator::evaluate(
    const Population& pop, Eigen::VectorXd& dominanceFitness) const
{
  const auto numSolutions = pop.getNumSolutions();

  isSolutionInParetoFront.resize(static_cast<int>(numSolutions));
  dominanceMatrix.resize(
        static_cast<int>(numSolutions), static_cast<int>(numSolutions));

  // Initiall we assume all the samples are not dominated.
  isSolutionInParetoFront.setConstant(true); // N x 1
  assert(
        static_cast<std::size_t>(isSolutionInParetoFront.size()) == numSolutions);
  dominanceMatrix.setZero(); // N x N
  assert(static_cast<std::size_t>(dominanceMatrix.rows()) == numSolutions);
  assert(static_cast<std::size_t>(dominanceMatrix.cols()) == numSolutions);

  Eigen::VectorXd iFitness;
  Eigen::VectorXd jFitness;
  for (auto i = 0u; i < numSolutions - 1u; ++i)
  {
    iFitness = pop.getFitness(i);

    for (auto j = i + 1u; j < numSolutions; ++j)
    {
      jFitness = pop.getFitness(j);

      const auto relationType = compare(iFitness, jFitness);

      // iObjs is dominated by jObjs.
      if (PairRelationType::DOMINATED == relationType)
      {
        isSolutionInParetoFront[i] = false;
        const auto dominanceVal = (iFitness - jFitness).norm();
        dominanceMatrix(j, i) = -dominanceVal;
        dominanceMatrix(i, j) = dominanceVal;
      }
      // currObjs is dominating nextObjs.
      else if (PairRelationType::DOMINATING == relationType)
      {
        isSolutionInParetoFront[j] = false;
        const auto dominanceVal = (iFitness - jFitness).norm();
        dominanceMatrix(i, j) = -dominanceVal;
        dominanceMatrix(j, i) = dominanceVal;
      }
    }
  }

  static Eigen::RowVectorXd zeroVector;
  zeroVector.setZero(static_cast<int>(numSolutions));

  // TODO(JS): We could make this optional.
  // Discard hyper volume, which is positive, between samples that both of them
  // are not in the Pareto front.
  for (auto i = 0; i < static_cast<int>(numSolutions); ++i)
  {
    if (isSolutionInParetoFront[i])
      continue;

    dominanceMatrix.row(i) = dominanceMatrix.row(i).cwiseMax(zeroVector);
  }

  dominanceFitness.resize(static_cast<int>(numSolutions));

  for (auto i = 0u; i < numSolutions; ++i)
  {
    const Eigen::VectorXd dominanceRow = dominanceMatrix.row(i);
    if (isSolutionInParetoFront[i])
    {
      // Number of elements with negative value is the number of samples
      // dominated by i-th sample.
      const auto dominanceCount = (dominanceRow.array() < 0.0).count();

      // dominanceRow should only contain zero or negative.
      assert((dominanceRow.array() > 0.0).count() == 0);

      // Simply, the strength value is the fitness.
      dominanceFitness[i] = double(dominanceCount) / double(numSolutions);

      assert(dominanceFitness[i] < 1.0);
      assert(dominanceFitness[i] >= 0.0);
    }
    else
    {
      //      const Eigen::VectorXd dominating =
      //      dominanceRow.cwiseMax(zeroVector);
      assert((dominanceRow.array() >= 0.0).all());
      const double elementsDominating = dominanceRow.sum();
      dominanceFitness[i] = 1.0 + elementsDominating;
      // Consider only summing up the distance between i-th element against
      // samples in notDominated set dominate i-th element.

      assert(dominanceFitness[i] >= 1.0);
    }
  }
}

} // namespace optimizer
} // namespace dart
