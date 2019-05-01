/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "dart/optimizer/pagmo/PagmoMultiObjectiveSolver.hpp"

#include <memory>
#include <Eigen/Dense>
#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/math/Constants.hpp"
#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/MultiObjectiveProblem.hpp"
#include "dart/optimizer/pagmo/PagmoMultiObjectiveProblemAdaptor.hpp"
#include "dart/optimizer/pagmo/PagmoUtils.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
PagmoMultiObjectiveSolver::UniqueProperties::UniqueProperties(
    Algorithm algorithm)
  : mAlgorithm(algorithm)
{
  // Do nothing
}

//==============================================================================
PagmoMultiObjectiveSolver::Properties::Properties(
    const MultiObjectiveSolver::Properties& solverProperties,
    const PagmoMultiObjectiveSolver::UniqueProperties& uniqueProperties)
  : MultiObjectiveSolver::Properties(solverProperties),
    UniqueProperties(uniqueProperties)
{
  // Do nothing
}

//==============================================================================
PagmoMultiObjectiveSolver::PagmoMultiObjectiveSolver(
    const Properties& properties)
  : MultiObjectiveSolver(properties), mPagmoMultiObjectiveSolverP(properties)
{
  // Do nothing
}

//==============================================================================
PagmoMultiObjectiveSolver::PagmoMultiObjectiveSolver(
    std::shared_ptr<MultiObjectiveProblem> problem)
  : MultiObjectiveSolver(std::move(problem))
{
  // Do nothing
}

//==============================================================================
PagmoMultiObjectiveSolver::~PagmoMultiObjectiveSolver()
{
  // Do nothing
}

//==============================================================================
#ifdef PAGMO_WITH_NLOPT
static pagmo::algorithm createNloptCobyla(
    const PagmoMultiObjectiveSolver::Properties& properties)
{
  pagmo::nlopt nlopt("cobyla");
  nlopt.set_maxeval(properties.mIterationsPerEvolution);
  pagmo::algorithm alg(nlopt);

  return alg;
}
#endif

//==============================================================================
static pagmo::algorithm createMoead(
    const PagmoMultiObjectiveSolver::Properties& properties)
{
  pagmo::algorithm alg(pagmo::moead(properties.mIterationsPerEvolution));

  return alg;
}

//==============================================================================
static pagmo::algorithm createNsga2(
    const PagmoMultiObjectiveSolver::Properties& properties)
{
  pagmo::algorithm alg(pagmo::nsga2(properties.mIterationsPerEvolution));

  return alg;
}

//==============================================================================
static pagmo::algorithm createPagmoAlgorithm(
    const PagmoMultiObjectiveSolver::Properties& properties)
{
  switch (properties.mAlgorithm)
  {
#ifdef PAGMO_WITH_NLOPT
    case PagmoMultiObjectiveSolver::Algorithm::Local_nlopt_COBYLA:
    {
      return createNloptCobyla(properties);
    }
#endif
    case PagmoMultiObjectiveSolver::Algorithm::Global_MOEAD:
    {
      return createMoead(properties);
    }
    case PagmoMultiObjectiveSolver::Algorithm::Global_NSGA2:
    {
      return createNsga2(properties);
    }
  }

  return pagmo::algorithm(pagmo::null_algorithm());
}

//==============================================================================
bool PagmoMultiObjectiveSolver::solve(std::size_t numEvolutions)
{
  // TODO(JS): Improve performace by re-using prob, archi, and populations.

  const std::shared_ptr<MultiObjectiveProblem>& prob = mProperties.mProblem;
  if (!prob)
    return true;

  pagmo::problem pagmoProb(
      PagmoMultiObjectiveProblemAdaptor(mProperties.mProblem));
  const pagmo::algorithm algo = createPagmoAlgorithm(mProperties);

  pagmo::archipelago archi = pagmo::archipelago(
      getNumPopulations(), algo, pagmoProb, getPopulationSize());
  archi.evolve(numEvolutions);
  archi.wait();

  mPopulations.clear();
  mPopulations.reserve(archi.size());
  for (std::size_t i = 0u; i < archi.size(); ++i)
  {
    mPopulations.emplace_back(
        PagmoTypes::convertPopulation(archi[i].get_population(), prob));
  }

  return true;
}

//==============================================================================
std::string PagmoMultiObjectiveSolver::getType() const
{
  return "PagmoMultiObjectiveSolver";
}

//==============================================================================
std::shared_ptr<MultiObjectiveSolver> PagmoMultiObjectiveSolver::clone() const
{
  return std::make_shared<PagmoMultiObjectiveSolver>(getSolverProperties());
}

//==============================================================================
void PagmoMultiObjectiveSolver::setProperties(const Properties& properties)
{
  MultiObjectiveSolver::setProperties(properties);
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void PagmoMultiObjectiveSolver::setProperties(
    const UniqueProperties& properties)
{
  setAlgorithm(properties.mAlgorithm);
}

//==============================================================================
PagmoMultiObjectiveSolver::Properties
PagmoMultiObjectiveSolver::getGradientDescentProperties() const
{
  return Properties(getSolverProperties(), mPagmoMultiObjectiveSolverP);
}

//==============================================================================
void PagmoMultiObjectiveSolver::copy(const PagmoMultiObjectiveSolver& other)
{
  setProperties(other.getSolverProperties());
}

//==============================================================================
PagmoMultiObjectiveSolver& PagmoMultiObjectiveSolver::operator=(
    const PagmoMultiObjectiveSolver& other)
{
  copy(other);
  return *this;
}

//==============================================================================
void PagmoMultiObjectiveSolver::setAlgorithm(Algorithm alg)
{
  mPagmoMultiObjectiveSolverP.mAlgorithm = alg;
}

//==============================================================================
PagmoMultiObjectiveSolver::Algorithm PagmoMultiObjectiveSolver::getAlgorithm()
    const
{
  return mPagmoMultiObjectiveSolverP.mAlgorithm;
}

} // namespace optimizer
} // namespace dart
