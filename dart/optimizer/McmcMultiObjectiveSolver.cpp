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
#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
McmcMultiObjectiveSolver::UniqueProperties::UniqueProperties()
{
  // Do nothing
}

//==============================================================================
McmcMultiObjectiveSolver::Properties::Properties(
    const MultiObjectiveSolver::Properties& solverProperties,
    const McmcMultiObjectiveSolver::UniqueProperties& uniqueProperties)
  : MultiObjectiveSolver::Properties(solverProperties),
    UniqueProperties(uniqueProperties)
{
  // Do nothing
}

//==============================================================================
McmcMultiObjectiveSolver::McmcMultiObjectiveSolver(
    const Properties& properties)
  : MultiObjectiveSolver(properties), mMcmcMultiObjectiveSolverP(properties)
{
  // Do nothing
}

//==============================================================================
McmcMultiObjectiveSolver::McmcMultiObjectiveSolver(
    std::shared_ptr<MultiObjectiveProblem> problem)
  : MultiObjectiveSolver(std::move(problem))
{
  // Do nothing
}

//==============================================================================
McmcMultiObjectiveSolver::~McmcMultiObjectiveSolver()
{
  // Do nothing
}

//==============================================================================
bool McmcMultiObjectiveSolver::solve(std::size_t /*numEvolutions*/)
{
  // TODO(JS): Improve performace by re-using prob, archi, and populations.

  const std::shared_ptr<MultiObjectiveProblem>& prob = mProperties.mProblem;
  if (!prob)
    return true;

  const auto numPopulations = getNumPopulations();
  const auto populationSize = getPopulationSize();

  mPopulations.resize(numPopulations, {nullptr});
  for (auto& population : mPopulations)
  {
    population;
  }

  return true;
}

//==============================================================================
std::string McmcMultiObjectiveSolver::getType() const
{
  return "McmcMultiObjectiveSolver";
}

//==============================================================================
std::shared_ptr<MultiObjectiveSolver> McmcMultiObjectiveSolver::clone() const
{
  return std::make_shared<McmcMultiObjectiveSolver>(getSolverProperties());
}

//==============================================================================
void McmcMultiObjectiveSolver::setProperties(const Properties& properties)
{
  MultiObjectiveSolver::setProperties(properties);
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void McmcMultiObjectiveSolver::setProperties(
    const UniqueProperties& properties)
{
  mMcmcMultiObjectiveSolverP = properties;
}

//==============================================================================
McmcMultiObjectiveSolver::Properties
McmcMultiObjectiveSolver::getGradientDescentProperties() const
{
  return Properties(getSolverProperties(), mMcmcMultiObjectiveSolverP);
}

//==============================================================================
void McmcMultiObjectiveSolver::copy(const McmcMultiObjectiveSolver& other)
{
  setProperties(other.getSolverProperties());
}

//==============================================================================
McmcMultiObjectiveSolver& McmcMultiObjectiveSolver::operator=(
    const McmcMultiObjectiveSolver& other)
{
  copy(other);
  return *this;
}

} // namespace optimizer
} // namespace dart
