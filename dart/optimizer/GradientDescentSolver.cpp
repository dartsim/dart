/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/optimizer/GradientDescentSolver.h"

namespace dart {
namespace optimizer {

//==============================================================================
const std::string GradientDescentSolver::Type = "GradientDescentSolver";

//==============================================================================
GradientDescentSolver::UniqueProperties::UniqueProperties(
    double _stepMultiplier,
    double _maxStepLength,
    size_t _maxAttempts,
    double _defaultConstraintWeight,
    Eigen::VectorXd _constraintWeights)
  : mStepMultiplier(_stepMultiplier),
    mMaxStepLength(_maxStepLength),
    mMaxAttempts(_maxAttempts),
    mDefaultConstraintWeight(_defaultConstraintWeight),
    mConstraintWeights(_constraintWeights)
{
  // Do nothing
}

//==============================================================================
GradientDescentSolver::Properties::Properties(
    const Solver::Properties& _solverProperties,
    const UniqueProperties& _descentProperties)
  : Solver::Properties(_solverProperties),
    UniqueProperties(_descentProperties)
{
  // Do nothing
}

//==============================================================================
GradientDescentSolver::GradientDescentSolver(const Properties& _properties)
  : Solver(_properties),
    mGradientP(_properties)
{
  // Do nothing
}

//==============================================================================
GradientDescentSolver::GradientDescentSolver(std::shared_ptr<Problem> _problem)
  : Solver(_problem)
{
  // Do nothing
}

//==============================================================================
GradientDescentSolver::~GradientDescentSolver()
{
  // Do nothing
}

//==============================================================================
bool GradientDescentSolver::solve()
{
  // TODO
}

//==============================================================================
std::string GradientDescentSolver::getType() const
{
  return Type;
}

//==============================================================================
std::shared_ptr<Solver> GradientDescentSolver::clone() const
{
  return std::make_shared<GradientDescentSolver>(
        getGradientDescentProperties());
}

//==============================================================================
void GradientDescentSolver::setProperties(const Properties& _properties)
{
  Solver::setProperties(_properties);
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void GradientDescentSolver::setProperties(const UniqueProperties& _properties)
{
  setStepMultiplier(_properties.mStepMultiplier);
  setMaxStepLength(_properties.mMaxStepLength);
  setMaxAttempts(_properties.mMaxAttempts);
  setDefaultConstraintWeight(_properties.mDefaultConstraintWeight);
  getConstraintWeights() = _properties.mConstraintWeights;
}

//==============================================================================
GradientDescentSolver::Properties
GradientDescentSolver::getGradientDescentProperties() const
{
  return GradientDescentSolver::Properties(getSolverProperties(), mGradientP);
}

//==============================================================================
void GradientDescentSolver::copy(const GradientDescentSolver& _other)
{
  if(this == &_other)
    return;

  setProperties(_other.getGradientDescentProperties());
}

//==============================================================================
GradientDescentSolver& GradientDescentSolver::operator=(
    const GradientDescentSolver& _other)
{
  copy(_other);
  return *this;
}

//==============================================================================
void GradientDescentSolver::setStepMultiplier(double _newMultiplier)
{
  mGradientP.mStepMultiplier = _newMultiplier;
}

//==============================================================================
double GradientDescentSolver::getStepMultiplier() const
{
  return mGradientP.mStepMultiplier;
}

//==============================================================================
void GradientDescentSolver::setMaxStepLength(double _newLength)
{
  mGradientP.mMaxStepLength = _newLength;
}

//==============================================================================
double GradientDescentSolver::getMaxStepLength() const
{
  return mGradientP.mMaxStepLength;
}

//==============================================================================
void GradientDescentSolver::setMaxAttempts(size_t _maxAttempts)
{
  mGradientP.mMaxAttempts = _maxAttempts;
}

//==============================================================================
size_t GradientDescentSolver::getMaxAttempts() const
{
  return mGradientP.mMaxAttempts;
}

//==============================================================================
void GradientDescentSolver::setDefaultConstraintWeight(double _newDefault)
{
  mGradientP.mDefaultConstraintWeight = _newDefault;
}

//==============================================================================
double GradientDescentSolver::getDefaultConstraintWeight() const
{
  return mGradientP.mDefaultConstraintWeight;
}

//==============================================================================
Eigen::VectorXd& GradientDescentSolver::getConstraintWeights()
{
  return mGradientP.mConstraintWeights;
}

//==============================================================================
const Eigen::VectorXd& GradientDescentSolver::getConstraintWeights() const
{
  return mGradientP.mConstraintWeights;
}

} // namespace optimizer
} // namespace dart
