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

#include "dart/optimizer/GenericMultiObjectiveProblem.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

#include "dart/common/Console.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/optimizer/Function.hpp"

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
GenericMultiObjectiveProblem::GenericMultiObjectiveProblem(
    std::size_t dim, std::size_t integerDim)
  : MultiObjectiveProblem(dim, integerDim),
    mObjectiveDimension(0u),
    mEqConstraintDimension(0u),
    mIneqConstraintDimension(0u)
{
  setSolutionDimension(dim);
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getObjectiveDimension() const
{
  return mObjectiveDimension;
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getNumObjectiveFunctions() const
{
  return mObjectiveFunctions.size();
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getNumEqualityConstraintFunctions()
    const
{
  return mEqConstraintFunctions.size();
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getNumIneqConstraintFunctions() const
{
  return mIneqConstraintFunctions.size();
}

//==============================================================================
void GenericMultiObjectiveProblem::setObjectiveFunctions(
    const std::vector<FunctionPtr>& objectives)
{
  mObjectiveFunctions = objectives;
  mObjectiveDimension = mObjectiveFunctions.size();
}

//==============================================================================
void GenericMultiObjectiveProblem::addObjectiveFunction(FunctionPtr objective)
{
  assert(objective && "nullptr pointer is not allowed.");
  mObjectiveFunctions.emplace_back(std::move(objective));
  mObjectiveDimension = mObjectiveFunctions.size();
}

//==============================================================================
const std::vector<FunctionPtr>&
GenericMultiObjectiveProblem::getObjectiveFunctions() const
{
  return mObjectiveFunctions;
}

//==============================================================================
void GenericMultiObjectiveProblem::addEqConstraintFunction(FunctionPtr eqConst)
{
  assert(eqConst);
  mEqConstraintFunctions.push_back(eqConst);
  mEqConstraintDimension = mEqConstraintFunctions.size();
}

//==============================================================================
void GenericMultiObjectiveProblem::addIneqConstraintFunction(
    FunctionPtr ineqConst)
{
  assert(ineqConst);
  mIneqConstraintFunctions.push_back(ineqConst);
  mIneqConstraintDimension = mIneqConstraintFunctions.size();
}

//==============================================================================
FunctionPtr GenericMultiObjectiveProblem::getEqConstraintFunction(
    std::size_t index) const
{
  assert(index < mEqConstraintFunctions.size());
  return getVectorObjectIfAvailable<FunctionPtr>(index, mEqConstraintFunctions);
}

//==============================================================================
FunctionPtr GenericMultiObjectiveProblem::getIneqConstraintFunction(
    std::size_t index) const
{
  assert(index < mIneqConstraintFunctions.size());
  return getVectorObjectIfAvailable<FunctionPtr>(
      index, mIneqConstraintFunctions);
}

//==============================================================================
void GenericMultiObjectiveProblem::removeObjectiveFunction(FunctionPtr function)
{
  mObjectiveFunctions.erase(
      std::remove(
          mObjectiveFunctions.begin(), mObjectiveFunctions.end(), function),
      mObjectiveFunctions.end());
  mObjectiveDimension = mObjectiveFunctions.size();
}

//==============================================================================
void GenericMultiObjectiveProblem::removeEqConstraintFunction(
    FunctionPtr eqConst)
{
  mEqConstraintFunctions.erase(
      std::remove(
          mEqConstraintFunctions.begin(),
          mEqConstraintFunctions.end(),
          eqConst),
      mEqConstraintFunctions.end());
  mEqConstraintDimension = mEqConstraintFunctions.size();
}

//==============================================================================
void GenericMultiObjectiveProblem::removeIneqConstraintFunction(
    FunctionPtr ineqConst)
{
  mIneqConstraintFunctions.erase(
      std::remove(
          mIneqConstraintFunctions.begin(),
          mIneqConstraintFunctions.end(),
          ineqConst),
      mIneqConstraintFunctions.end());
  mIneqConstraintDimension = mIneqConstraintFunctions.size();
}

//==============================================================================
void GenericMultiObjectiveProblem::removeAllObjectiveFunctions()
{
  mObjectiveFunctions.clear();
  mObjectiveDimension = mObjectiveFunctions.size();
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getEqConstraintDimension() const
{
  return mEqConstraintDimension;
}

//==============================================================================
void GenericMultiObjectiveProblem::removeAllEqConstraintFunctions()
{
  mEqConstraintFunctions.clear();
  mEqConstraintDimension = mEqConstraintFunctions.size();
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getIneqConstraintDimension() const
{
  return mIneqConstraintDimension;
}

//==============================================================================
void GenericMultiObjectiveProblem::removeAllIneqConstraintFunctions()
{
  mIneqConstraintFunctions.clear();
  mIneqConstraintDimension = mIneqConstraintFunctions.size();
}

//==============================================================================
static Eigen::VectorXd computeFunctions(
    const std::vector<FunctionPtr>& functions,
    std::size_t dimension,
    const Eigen::VectorXd& x)
{
  Eigen::VectorXd val(dimension);

  std::size_t index = 0u;
  for (const FunctionPtr& function : functions)
  {
    const std::size_t size = 1u; // TODO(JS): Update this once Function can
                                 // return vector
    // objectives.segment(index, size) = objective->evaluate(x);
    val[static_cast<int>(index)] = function->eval(x);
    index += size;
  }

  return val;
}

//==============================================================================
Eigen::VectorXd GenericMultiObjectiveProblem::evaluateObjectives(
    const Eigen::VectorXd& x) const
{
  return computeFunctions(mObjectiveFunctions, getObjectiveDimension(), x);
}

//==============================================================================
Eigen::VectorXd GenericMultiObjectiveProblem::evaluateEqConstraints(
    const Eigen::VectorXd& x) const
{
  return computeFunctions(
      mEqConstraintFunctions, getEqConstraintDimension(), x);
}

//==============================================================================
Eigen::VectorXd GenericMultiObjectiveProblem::evaluateIneqConstraints(
    const Eigen::VectorXd& x) const
{
  return computeFunctions(
      mIneqConstraintFunctions, getIneqConstraintDimension(), x);
}

//==============================================================================
std::size_t GenericMultiObjectiveProblem::getFitnessDimension() const
{
  const std::size_t objDim = getObjectiveDimension();
  const std::size_t eqDim = getEqConstraintDimension();
  const std::size_t ineqDim = getIneqConstraintDimension();

  return objDim + eqDim + ineqDim;
}

//==============================================================================
Eigen::VectorXd GenericMultiObjectiveProblem::evaluateFitness(
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

} // namespace optimizer
} // namespace dart
