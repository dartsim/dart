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

#include "dart/optimizer/Population.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Random.hpp"

namespace dart {
namespace optimizer {

namespace {

//==============================================================================
bool isValidX(const MultiObjectiveProblem& problem, const Eigen::VectorXd& x)
{
  if (problem.getSolutionDimension() != static_cast<std::size_t>(x.size()))
  {
    dtwarn << "[Population] Attempting to add an incompatible decision vector. "
           << "The dimension of the decision vector '" << x.size()
           << "' should be '" << problem.getSolutionDimension() << "'.\n";
    return false;
  }

  return true;
}

//==============================================================================
bool isValidF(const MultiObjectiveProblem& problem, const Eigen::VectorXd& f)
{
  if (problem.getFitnessDimension() != static_cast<std::size_t>(f.size()))
  {
    dtwarn << "[Population] Attempting to add an incompatible decision vector. "
           << "The dimension of the decision vector '" << f.size()
           << "' should be '" << problem.getFitnessDimension() << "'.\n";
    return false;
  }

  return true;
}

} // (anonymous) namespace

//==============================================================================
Population::Population(
    std::shared_ptr<MultiObjectiveProblem> problem, std::size_t populationSize)
  : mProblem(std::move(problem))
{
  // TODO(JS): Take the argument type of problem by const reference of
  // MultiObjectiveProblem and clone it once MultiObjectiveProblem::clone()
  // is added.

  assert(mProblem);

  const int xSize = static_cast<int>(mProblem->getSolutionDimension());
  const int fSize = static_cast<int>(mProblem->getFitnessDimension());

  mPopulation.resize(xSize, static_cast<int>(populationSize));
  mFitness.resize(fSize, static_cast<int>(populationSize));

  const Eigen::VectorXd& lb = mProblem->getLowerBounds();
  const Eigen::VectorXd& ub = mProblem->getUpperBounds();

  for (std::size_t i = 0u; i < populationSize; ++i)
    set(i, math::Random::uniform(lb, ub));
}

//==============================================================================
MultiObjectiveProblem* Population::getProblem()
{
  return mProblem.get();
}

//==============================================================================
const MultiObjectiveProblem* Population::getProblem() const
{
  return mProblem.get();
}

//==============================================================================
void Population::pushBack(const Eigen::VectorXd& x)
{
  if (!isValidX(*mProblem, x))
    return;

  const Eigen::VectorXd f = mProblem->evaluateFitness(x);
  pushBack(x, f);
}

//==============================================================================
void Population::pushBack(const Eigen::VectorXd& x, const Eigen::VectorXd& f)
{
  if (!isValidX(*mProblem, x))
    return;

  if (!isValidF(*mProblem, f))
    return;

  mPopulation.conservativeResize(Eigen::NoChange, mPopulation.cols() + 1);
  mPopulation.rightCols(1) = x;

  mFitness.conservativeResize(Eigen::NoChange, mFitness.cols() + 1);
  mFitness.rightCols(1) = f;
}

//==============================================================================
void Population::set(std::size_t index, const Eigen::VectorXd& x)
{
  isValidX(*mProblem, x);

  set(index, x, mProblem->evaluateFitness(x));
}

//==============================================================================
void Population::set(
    std::size_t index, const Eigen::VectorXd& x, const Eigen::VectorXd& f)
{
  if (!isValidX(*mProblem, x))
    return;

  if (!isValidF(*mProblem, f))
    return;

  mPopulation.col(static_cast<int>(index)) = x;
  mFitness.col(static_cast<int>(index)) = f;
}

//==============================================================================
std::size_t Population::getSize() const
{
  assert(mPopulation.cols() == mFitness.cols());
  return static_cast<std::size_t>(mPopulation.cols());
}

//==============================================================================
Eigen::VectorXd Population::getDecisionVector(std::size_t index) const
{
  return mPopulation.col(static_cast<int>(index));
}

//==============================================================================
Eigen::VectorXd Population::getFitnessVector(std::size_t index) const
{
  return mFitness.col(static_cast<int>(index));
}

//==============================================================================
std::ostream& Population::print(std::ostream& os) const
{
  os << getProblem() << "\n";
  os << "Population size: " << getSize() << "\n\n";
  os << "List of individuals: \n";

  for (std::size_t i = 0u; i < getSize(); ++i)
  {
    os << "#" << i << ":\n";
    os << "\tDecision vector:\t"
       << mPopulation.col(static_cast<int>(i)).transpose() << "\n";
    os << "\tDecision vector:\t"
       << mFitness.col(static_cast<int>(i)).transpose() << "\n";
  }

  // TODO(JS): Print champions if the dimension is one.

  return os;
}

} // namespace optimizer
} // namespace dart
