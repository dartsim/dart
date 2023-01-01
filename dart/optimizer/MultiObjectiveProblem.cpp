/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/optimizer/MultiObjectiveProblem.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/optimizer/Function.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace dart {
namespace optimizer {

//==============================================================================
template <typename T>
static T getVectorObjectIfAvailable(std::size_t idx, const std::vector<T>& vec)
{
  // TODO: Should we have an out-of-bounds assertion or throw here?
  if (idx < vec.size())
    return vec[idx];

  return nullptr;
}

//==============================================================================
MultiObjectiveProblem::MultiObjectiveProblem(
    std::size_t dim, std::size_t integerDim)
{
  setSolutionDimension(dim, integerDim);
}

//==============================================================================
void MultiObjectiveProblem::setSolutionDimension(
    std::size_t dim, std::size_t integerDim)
{
  if (dim == mDimension && integerDim == mIntegerDimension)
    return;

  mDimension = dim;
  mIntegerDimension = integerDim;

  const double inf = std::numeric_limits<double>::infinity();
  const auto dimension = static_cast<Eigen::VectorXd::Index>(dim);

  setLowerBounds(Eigen::VectorXd::Constant(dimension, -inf));
  setUpperBounds(Eigen::VectorXd::Constant(dimension, inf));
}

//==============================================================================
std::size_t MultiObjectiveProblem::getSolutionDimension() const
{
  return mDimension;
}

//==============================================================================
std::size_t MultiObjectiveProblem::getDoubleDimension() const
{
  return getSolutionDimension() - getIntegerDimension();
}

//==============================================================================
void MultiObjectiveProblem::setIntegerDimension(std::size_t dim)
{
  mIntegerDimension = dim;
}

//==============================================================================
std::size_t MultiObjectiveProblem::getIntegerDimension() const
{
  return mIntegerDimension;
}

//==============================================================================
void MultiObjectiveProblem::setLowerBounds(const Eigen::VectorXd& lb)
{
  assert(static_cast<std::size_t>(lb.size()) == mDimension && "Invalid size.");
  mLowerBounds = lb;
}

//==============================================================================
const Eigen::VectorXd& MultiObjectiveProblem::getLowerBounds() const
{
  return mLowerBounds;
}

//==============================================================================
void MultiObjectiveProblem::setUpperBounds(const Eigen::VectorXd& ub)
{
  assert(static_cast<std::size_t>(ub.size()) == mDimension && "Invalid size.");
  mUpperBounds = ub;
}

//==============================================================================
const Eigen::VectorXd& MultiObjectiveProblem::getUpperBounds() const
{
  return mUpperBounds;
}

//==============================================================================
std::size_t MultiObjectiveProblem::getEqConstraintDimension() const
{
  return 0u;
}

//==============================================================================
std::size_t MultiObjectiveProblem::getIneqConstraintDimension() const
{
  return 0u;
}

//==============================================================================
Eigen::VectorXd MultiObjectiveProblem::evaluateEqConstraints(
    const Eigen::VectorXd& /*x*/) const
{
  return Eigen::VectorXd::Zero(0);
}

//==============================================================================
Eigen::VectorXd MultiObjectiveProblem::evaluateIneqConstraints(
    const Eigen::VectorXd& /*x*/) const
{
  return Eigen::VectorXd::Zero(0);
}

//==============================================================================
std::size_t MultiObjectiveProblem::getFitnessDimension() const
{
  const std::size_t objDim = getObjectiveDimension();
  const std::size_t eqDim = getEqConstraintDimension();
  const std::size_t ineqDim = getIneqConstraintDimension();

  return objDim + eqDim + ineqDim;
}

//==============================================================================
Eigen::VectorXd MultiObjectiveProblem::evaluateFitness(
    const Eigen::VectorXd& x) const
{
  const int objDim = static_cast<int>(getObjectiveDimension());
  const int eqDim = static_cast<int>(getEqConstraintDimension());
  const int ineqDim = static_cast<int>(getIneqConstraintDimension());
  const int totalDim = objDim + eqDim + ineqDim;

  Eigen::VectorXd f(totalDim);

  f.head(objDim) = evaluateObjectives(x);
  f.segment(objDim, eqDim) = evaluateEqConstraints(x);
  f.tail(ineqDim) = evaluateIneqConstraints(x);

  return f;
}

//==============================================================================
std::ostream& MultiObjectiveProblem::print(std::ostream& os) const
{
  os << "MultiObjectiveProblem\n";
  os << "\tParameter dimension:\t" << getSolutionDimension() << "\n";

  return os;
}

} // namespace optimizer
} // namespace dart
