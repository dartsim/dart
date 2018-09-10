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
bool isValidX(std::size_t solutionDimension, const Eigen::VectorXd& x)
{
  if (solutionDimension != static_cast<std::size_t>(x.size()))
  {
    dtwarn << "[Population] Attempting to add an incompatible solution. "
           << "The dimension of the solution '" << x.size() << "' should be '"
           << solutionDimension << "'.\n";
    return false;
  }

  return true;
}

//==============================================================================
bool isValidF(std::size_t fitnessDimension, const Eigen::VectorXd& f)
{
  if (fitnessDimension != static_cast<std::size_t>(f.size()))
  {
    dtwarn << "[Population] Attempting to add an incompatible solution. "
           << "The dimension of the fitness '" << f.size() << "' should be '"
           << fitnessDimension << "'.\n";
    return false;
  }

  return true;
}

} // (anonymous) namespace

//==============================================================================
Population::Population(
    const MultiObjectiveProblem& problem, std::size_t numSolutions)
{
  setRandom(problem, numSolutions);
}

//==============================================================================
Population::Population(
    std::size_t solutionDimension,
    std::size_t fitnessDimension,
    std::size_t numSolutions)
{
  resize(solutionDimension, fitnessDimension, numSolutions);
}

//==============================================================================
void Population::pushBack(const Eigen::VectorXd& x, const Eigen::VectorXd& f)
{
  if (!isValidX(mSolutionDimension, x))
    return;

  if (!isValidF(mFitnessDimension, f))
    return;

  mSolutions.conservativeResize(Eigen::NoChange, mSolutions.cols() + 1);
  mSolutions.rightCols(1) = x;

  mFitnesses.conservativeResize(Eigen::NoChange, mFitnesses.cols() + 1);
  mFitnesses.rightCols(1) = f;
}

//==============================================================================
void Population::set(
    std::size_t index, const Eigen::VectorXd& x, const Eigen::VectorXd& f)
{
  if (!isValidX(mSolutionDimension, x))
    return;

  if (!isValidF(mFitnessDimension, f))
    return;

  // TODO(JS): check validity of index

  mSolutions.col(static_cast<int>(index)) = x;
  mFitnesses.col(static_cast<int>(index)) = f;
}

//==============================================================================
void Population::setRandom(
    const MultiObjectiveProblem& prob, std::size_t numSolutions)
{
  const auto solutionDimension = prob.getSolutionDimension();
  const auto fitnessDimension = prob.getFitnessDimension();
  resize(solutionDimension, fitnessDimension, numSolutions);

  for (std::size_t i = 0u; i < numSolutions; ++i)
  {
    const Eigen::VectorXd solution
        = math::Random::uniform(prob.getLowerBounds(), prob.getUpperBounds());
    const Eigen::VectorXd fitness = prob.evaluateFitness(solution);
    set(i, solution, fitness);
  }
}

//==============================================================================
void Population::setRandom(const MultiObjectiveProblem& prob)
{
  setRandom(prob, mNumSolutions);
}

//==============================================================================
std::size_t Population::getNumSolutions() const
{
  assert(mSolutions.cols() == mFitnesses.cols());
  assert(static_cast<std::size_t>(mSolutions.cols()) == mNumSolutions);
  return mNumSolutions;
}

//==============================================================================
Eigen::VectorXd Population::getSolution(std::size_t index) const
{
  return mSolutions.col(static_cast<int>(index));
}

//==============================================================================
Eigen::VectorXd Population::getFitness(std::size_t index) const
{
  return mFitnesses.col(static_cast<int>(index));
}

//==============================================================================
std::ostream& Population::print(std::ostream& os) const
{
  //  os << getProblem() << "\n";
  os << "Population size: " << getNumSolutions() << "\n\n";
  os << "List of individuals: \n";

  for (int i = 0u; i < static_cast<int>(getNumSolutions()); ++i)
  {
    os << "#" << i << ":\n";
    os << "\tSolution:\t" << mSolutions.col(static_cast<int>(i)).transpose()
       << "\n";
    os << "\tFitness :\t" << mFitnesses.col(static_cast<int>(i)).transpose()
       << "\n";
  }

  // TODO(JS): Print champions if the dimension is one.

  return os;
}

//==============================================================================
void Population::resize(
    std::size_t solutionDimension,
    std::size_t fitnessDimension,
    std::size_t numSolutions)
{
  mSolutionDimension = solutionDimension;
  mFitnessDimension = fitnessDimension;
  mNumSolutions = numSolutions;

  mSolutions.resize(
      static_cast<int>(mSolutionDimension), static_cast<int>(mNumSolutions));
  mFitnesses.resize(
      static_cast<int>(mFitnessDimension), static_cast<int>(mNumSolutions));
}

} // namespace optimizer
} // namespace dart
