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

#include "dart/optimizer/MultiObjectiveSolver.hpp"

#include <fstream>
#include "dart/common/Console.hpp"
#include "dart/optimizer/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
MultiObjectiveSolver::Properties::Properties(
    std::shared_ptr<MultiObjectiveProblem> problem,
    std::size_t numPopulations,
    std::size_t populationSize,
    double tolerance,
    std::size_t numMaxIterations,
    std::size_t iterationsPerPrint,
    std::ostream* ostream,
    bool printFinalResult)
  : mProblem(std::move(problem)),
    mNumPopulations(numPopulations),
    mPopulationSize(populationSize),
    mTolerance(tolerance),
    mIterationsPerEvolution(numMaxIterations),
    mIterationsPerPrint(iterationsPerPrint),
    mOutStream(ostream),
    mPrintFinalResult(printFinalResult)
{
  // Do nothing
}

//==============================================================================
MultiObjectiveSolver::MultiObjectiveSolver(const Properties& properties)
{
  setProperties(properties);
}

//==============================================================================
void MultiObjectiveSolver::setProperties(
    const MultiObjectiveSolver::Properties& properties)
{
  mProperties.mProblem = properties.mProblem;
  setPopulationSize(properties.mPopulationSize);
  setNumPopulations(properties.mNumPopulations);
  mProperties.mTolerance = properties.mTolerance;
  mProperties.mIterationsPerEvolution = properties.mIterationsPerEvolution;
  mProperties.mIterationsPerPrint = properties.mIterationsPerPrint;
  mProperties.mOutStream = properties.mOutStream;
  mProperties.mPrintFinalResult = properties.mPrintFinalResult;
}

//==============================================================================
const MultiObjectiveSolver::Properties&
MultiObjectiveSolver::getSolverProperties() const
{
  return mProperties;
}

//==============================================================================
void MultiObjectiveSolver::setProblem(
    std::shared_ptr<MultiObjectiveProblem> problem)
{
  mProperties.mProblem = std::move(problem);
}

//==============================================================================
std::shared_ptr<MultiObjectiveProblem> MultiObjectiveSolver::getProblem() const
{
  return mProperties.mProblem;
}

//==============================================================================
void MultiObjectiveSolver::setPopulationSize(std::size_t size)
{
  mProperties.mPopulationSize = size;
}

//==============================================================================
std::size_t MultiObjectiveSolver::getPopulationSize() const
{
  return mProperties.mPopulationSize;
}

//==============================================================================
void MultiObjectiveSolver::setNumPopulations(std::size_t size)
{
  mProperties.mNumPopulations = size;
  mPopulations.resize(size);
}

//==============================================================================
std::size_t MultiObjectiveSolver::getNumPopulations() const
{
  assert(mPopulations.size() == mProperties.mNumPopulations);
  return mProperties.mNumPopulations;
}

//==============================================================================
const Population& MultiObjectiveSolver::getPopulation(std::size_t index) const
{
  return mPopulations[index];
}

//==============================================================================
const std::vector<Population>& MultiObjectiveSolver::getPopulations() const
{
  return mPopulations;
}

//==============================================================================
void MultiObjectiveSolver::setNumIterationsPerEvolution(
    std::size_t maxIterations)
{
  mProperties.mIterationsPerEvolution = maxIterations;
}

//==============================================================================
std::size_t MultiObjectiveSolver::getNumIterationsPerEvolution() const
{
  return mProperties.mIterationsPerEvolution;
}

} // namespace optimizer
} // namespace dart
