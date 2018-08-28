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

#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"
#include "dart/optimizer/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
Population::Population(
    std::shared_ptr<MultiObjectiveProblem> problem, std::size_t populationSize)
  : mProblem(std::move(problem)), mMT(math::RandomDevice::next())
{
  const int xSize = mProblem ? static_cast<int>(mProblem->getDimension()) : 0;
  const int fSize
      = mProblem ? static_cast<int>(mProblem->getFitnessDimension()) : 0;

  mPopulation.resize(xSize, static_cast<int>(populationSize));
  mFitnesses.resize(fSize, static_cast<int>(populationSize));

  if (mProblem)
    return;

  const Eigen::VectorXd lb = mProblem->getLowerBounds();
  const Eigen::VectorXd ub = mProblem->getUpperBounds();

  for (std::size_t i = 0u; i < populationSize; ++i)
  {
    const Eigen::VectorXd randVec = math::randomVectorXd(mMT, lb, ub);
    set(i, randVec);
  }
}

//==============================================================================
std::shared_ptr<MultiObjectiveProblem> Population::getProblem()
{
  return mProblem;
}

//==============================================================================
std::shared_ptr<const MultiObjectiveProblem> Population::getProblem() const
{
  return mProblem;
}

//==============================================================================
void Population::pushBack(const Eigen::VectorXd& x)
{
  // TODO(JS): Validity check

  const Eigen::VectorXd f = mProblem->evaluateFitness(x);
  pushBack(x, f);
}

//==============================================================================
void Population::pushBack(const Eigen::VectorXd& x, const Eigen::VectorXd& f)
{
  // TODO(JS): Validity check

  mPopulation.conservativeResize(Eigen::NoChange, mPopulation.cols() + 1);
  mPopulation.rightCols(1) = x;

  mFitnesses.conservativeResize(Eigen::NoChange, mFitnesses.cols() + 1);
  mFitnesses.rightCols(1) = f;
}

//==============================================================================
void Population::set(std::size_t index, const Eigen::VectorXd& x)
{
  set(index, x, mProblem->evaluateFitness(x));
}

//==============================================================================
void Population::set(
    std::size_t index, const Eigen::VectorXd& x, const Eigen::VectorXd& f)
{
  mPopulation.col(static_cast<int>(index)) = x;
  mFitnesses.col(static_cast<int>(index)) = f;
}

//==============================================================================
std::size_t Population::getSize() const
{
  assert(mPopulation.cols() == mFitnesses.cols());
  return static_cast<std::size_t>(mPopulation.cols());
}

//==============================================================================
const Eigen::MatrixXd& Population::getDecisionVectors() const
{
  return mPopulation;
}

//==============================================================================
Eigen::VectorXd Population::getDecisionVector(std::size_t index) const
{
  return mPopulation.col(static_cast<int>(index));
}

//==============================================================================
const Eigen::MatrixXd& Population::getFitnessVectors() const
{
  return mFitnesses;
}

//==============================================================================
Eigen::VectorXd Population::getFitnessVector(std::size_t index) const
{
  return mFitnesses.col(static_cast<int>(index));
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
    os << "\tDecision vector:\t" << getDecisionVectors().col(i).transpose()
       << "\n";
    os << "\tDecision vector:\t" << getFitnessVectors().col(i).transpose()
       << "\n";
  }

  // TODO(JS): Print champions if the dimension is one.

  return os;
}

} // namespace optimizer
} // namespace dart
