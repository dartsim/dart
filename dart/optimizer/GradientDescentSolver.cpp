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

#include "dart/common/Console.h"
#include "dart/optimizer/GradientDescentSolver.h"
#include "dart/optimizer/Problem.h"

namespace dart {
namespace optimizer {

//==============================================================================
const std::string GradientDescentSolver::Type = "GradientDescentSolver";

//==============================================================================
GradientDescentSolver::UniqueProperties::UniqueProperties(
    double _stepMultiplier,
    size_t _maxAttempts,
    size_t _perturbationStep,
    double _maxPerturbationFactor,
    double _maxRandomizationStep,
    double _defaultConstraintWeight,
    Eigen::VectorXd _eqConstraintWeights,
    Eigen::VectorXd _ineqConstraintWeights)
  : mStepSize(_stepMultiplier),
    mMaxAttempts(_maxAttempts),
    mPerturbationStep(_perturbationStep),
    mMaxPerturbationFactor(_maxPerturbationFactor),
    mMaxRandomizationStep(_maxRandomizationStep),
    mDefaultConstraintWeight(_defaultConstraintWeight),
    mEqConstraintWeights(_eqConstraintWeights),
    mIneqConstraintWeights(_ineqConstraintWeights)
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
    mGradientP(_properties),
    mRD(),
    mMT(mRD()),
    mDistribution(0.0, std::nextafter(1.0, 2.0)) // This allows mDistrubtion to produce numbers in the range [0,1] inclusive
{
  // Do nothing
}

//==============================================================================
GradientDescentSolver::GradientDescentSolver(std::shared_ptr<Problem> _problem)
  : Solver(_problem),
    mRD(),
    mMT(mRD()),
    mDistribution(0.0, std::nextafter(1.0, 2.0))
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
  bool minimized = false;
  bool satisfied = false;

  std::shared_ptr<Problem> problem = mProperties.mProblem;
  double tol = std::abs(mProperties.mTolerance);
  double gamma = mGradientP.mStepSize;
  size_t dim = problem->getDimension();

  Eigen::VectorXd x = problem->getInitialGuess();
  Eigen::VectorXd lastx = x;
  Eigen::VectorXd dx(x.size());
  Eigen::VectorXd grad(x.size());
  std::vector<bool> ineqViolated(problem->getNumIneqConstraints());

  size_t attemptCount = 0;
  do
  {
    size_t stepCount = 0;
    do
    {
      Eigen::Map<const Eigen::VectorXd> xMap(x.data(), dim);

      // Perturb the configuration if we have reached an iteration where we are
      // supposed to perturb it.
      if(mGradientP.mPerturbationStep > 0
         && (stepCount+1)%mGradientP.mPerturbationStep == 0)
      {
        dx = x; // Seed the configuration randomizer with the current configuration
        randomizeConfiguration(dx);

        // Step the current configuration towards the randomized configuration
        // proportionally to a randomized scaling factor
        double scale = mGradientP.mMaxPerturbationFactor*mDistribution(mMT);
        x += scale*(dx-x);
      }

      // Check if the constraints are satsified
      satisfied = true;
      for(size_t i=0; i<problem->getNumEqConstraints(); ++i)
      {
        if(std::abs(problem->getEqConstraint(i)->eval(xMap)) > tol)
        {
          satisfied = false;
          break; // If we already know that at least one constraint is violated,
                 // then don't bother checking the rest.
        }
      }

      for(size_t i=0; i<problem->getNumIneqConstraints(); ++i)
      {
        double ineqCost = problem->getIneqConstraint(i)->eval(xMap);
        if(ineqCost > std::abs(tol))
        {
          ineqViolated[i] = true;
          satisfied = false;
        }
        else
          ineqViolated[i] = false;
      }

      Eigen::Map<Eigen::VectorXd> dxMap(dx.data(), dim);
      Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), dim);
      // Compute the gradient of the objective, combined with the weighted
      // gradients of the softened constraints
      problem->getObjective()->evalGradient(xMap, dxMap);
      for(int i=0; i < static_cast<int>(problem->getNumEqConstraints()); ++i)
      {
        // TODO: Should we ignore the gradients of equality constraints that are
        // already satisfied, the way we do for inequality constraints? It might
        // save some operations.
        problem->getEqConstraint(i)->evalGradient(xMap, gradMap);
        double weight = mGradientP.mEqConstraintWeights.size() > i?
              mGradientP.mEqConstraintWeights[i] :
              mGradientP.mDefaultConstraintWeight;
        dx += weight * grad;
      }

      for(int i=0; i < static_cast<int>(problem->getNumIneqConstraints()); ++i)
      {
        if(ineqViolated[i])
        {
          problem->getIneqConstraint(i)->evalGradient(xMap, gradMap);
          double weight = mGradientP.mIneqConstraintWeights.size() > i?
                mGradientP.mIneqConstraintWeights[i] :
                mGradientP.mDefaultConstraintWeight;
          dx += weight * grad;
        }
      }

      x -= gamma*dx;
      clampToBoundary(x);

      if((x-lastx).norm() < tol)
        minimized = true;
      else
        minimized = false;

      lastx = x;
      ++stepCount;

      if(mProperties.mIterationsPerPrint > 0 &&
         stepCount%mProperties.mIterationsPerPrint == 0)
      {
        std::cout << "[GradientDescentSolver] Progress (attempt #"
                  << attemptCount << " | iteration #" << stepCount << ")\n"
                  << "cost: " << problem->getObjective()->eval(x) <<  " | x: "
                  << x.transpose() << std::endl;
      }

      if(stepCount > mProperties.mNumMaxIterations)
        break;

    } while(!minimized || !satisfied);

    if(!minimized || !satisfied)
    {
      ++attemptCount;

      if(mGradientP.mMaxAttempts > 0 && attemptCount >= mGradientP.mMaxAttempts)
        break;

      if(attemptCount-1 < problem->getAllSeeds().size())
      {
        x = problem->getSeed(attemptCount-1);
      }
      else
      {
        randomizeConfiguration(x);
      }
    }

  } while(!minimized || !satisfied);

  return minimized && satisfied;
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
  setStepSize(_properties.mStepSize);
  setMaxAttempts(_properties.mMaxAttempts);
  setPerturbationStep(_properties.mPerturbationStep);
  setMaxPerturbationFactor(_properties.mMaxPerturbationFactor);
  setDefaultConstraintWeight(_properties.mDefaultConstraintWeight);
  getConstraintWeights() = _properties.mEqConstraintWeights;
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
void GradientDescentSolver::setStepSize(double _newMultiplier)
{
  mGradientP.mStepSize = _newMultiplier;
}

//==============================================================================
double GradientDescentSolver::getStepSize() const
{
  return mGradientP.mStepSize;
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
void GradientDescentSolver::setPerturbationStep(size_t _step)
{
  mGradientP.mPerturbationStep = _step;
}

//==============================================================================
size_t GradientDescentSolver::getPerturbationStep() const
{
  return mGradientP.mPerturbationStep;
}

//==============================================================================
void GradientDescentSolver::setMaxPerturbationFactor(double _factor)
{
  mGradientP.mMaxPerturbationFactor = _factor;
}

//==============================================================================
double GradientDescentSolver::getMaxPerturbationFactor() const
{
  return mGradientP.mMaxPerturbationFactor;
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
  return mGradientP.mEqConstraintWeights;
}

//==============================================================================
const Eigen::VectorXd& GradientDescentSolver::getConstraintWeights() const
{
  return mGradientP.mEqConstraintWeights;
}

//==============================================================================
void GradientDescentSolver::randomizeConfiguration(Eigen::VectorXd& _x)
{
  if(nullptr == mProperties.mProblem)
    return;

  if(_x.size() < static_cast<int>(mProperties.mProblem->getDimension()))
    _x = Eigen::VectorXd::Zero(mProperties.mProblem->getDimension());

  for(int i=0; i<_x.size(); ++i)
  {
    double lower = mProperties.mProblem->getLowerBounds()[i];
    double upper = mProperties.mProblem->getUpperBounds()[i];
    double step = upper - lower;
    if(step > mGradientP.mMaxRandomizationStep)
    {
      step = 2*mGradientP.mMaxRandomizationStep;
      lower = _x[i] - step/2.0;
    }

    _x[i] = step*mDistribution(mMT) + lower;
  }
}

//==============================================================================
void GradientDescentSolver::clampToBoundary(Eigen::VectorXd& _x)
{
  if(nullptr == mProperties.mProblem)
    return;

  if( _x.size() != static_cast<int>(mProperties.mProblem->getDimension()) )
  {
    dtwarn << "[GradientDescentSolver::clampToBoundary] Mismatch between "
           << "configuration size [" << _x.size() << "] and the dimension of "
           << "the Problem [" << mProperties.mProblem->getDimension() << "]\n";
    return;
  }

  for(int i=0; i<_x.size(); ++i)
  {
    if( _x[i] < mProperties.mProblem->getLowerBounds()[i] )
      _x[i] = mProperties.mProblem->getLowerBounds()[i];
    else if( mProperties.mProblem->getUpperBounds()[i] < _x[i] )
      _x[i] = mProperties.mProblem->getUpperBounds()[i];
  }
}

} // namespace optimizer
} // namespace dart
