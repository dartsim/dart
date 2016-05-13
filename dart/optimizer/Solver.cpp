/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/optimizer/Solver.hpp"
#include "dart/optimizer/Problem.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
Solver::Properties::Properties(
    std::shared_ptr<Problem> _problem,
    double _tolerance,
    std::size_t _numMaxIterations,
    std::size_t _iterationsPerPrint,
    std::ostream* _ostream,
    bool _printFinalResult,
    const std::string &_resultFile)
  : mProblem(_problem),
    mTolerance(_tolerance),
    mNumMaxIterations(_numMaxIterations),
    mIterationsPerPrint(_iterationsPerPrint),
    mOutStream(_ostream),
    mPrintFinalResult(_printFinalResult),
    mResultFile(_resultFile)
{
  // Do nothing
}

//==============================================================================
Solver::Solver(const Properties& _properties)
  : mProperties(_properties)
{
  // Do nothing
}

//==============================================================================
Solver::Solver(std::shared_ptr<Problem> _problem)
  : mProperties(_problem)
{
  // Do nothing
}

//==============================================================================
void Solver::setProperties(const Properties& _properties)
{
  setProblem(_properties.mProblem);
  setNumMaxIterations(_properties.mNumMaxIterations);
  setIterationsPerPrint(_properties.mIterationsPerPrint);
  setOutStream(_properties.mOutStream);
  setPrintFinalResult(_properties.mPrintFinalResult);
  setResultFileName(_properties.mResultFile);
}

//==============================================================================
const Solver::Properties& Solver::getSolverProperties() const
{
  return mProperties;
}

//==============================================================================
void Solver::copy(const Solver& _otherSolver)
{
  if(this == &_otherSolver)
    return;

  setProperties(_otherSolver.getSolverProperties());
}

//==============================================================================
Solver& Solver::operator=(const Solver& _otherSolver)
{
  copy(_otherSolver);
  return *this;
}

//==============================================================================
void Solver::setProblem(std::shared_ptr<Problem> _newProblem)
{
  mProperties.mProblem = _newProblem;
}

//==============================================================================
std::shared_ptr<Problem> Solver::getProblem() const
{
  return mProperties.mProblem;
}

//==============================================================================
void Solver::setTolerance(double _newTolerance)
{
  mProperties.mTolerance = _newTolerance;
}

//==============================================================================
double Solver::getTolerance() const
{
  return mProperties.mTolerance;
}

//==============================================================================
void Solver::setNumMaxIterations(std::size_t _newMax)
{
  mProperties.mNumMaxIterations = _newMax;
}

//==============================================================================
std::size_t Solver::getNumMaxIterations() const
{
  return mProperties.mNumMaxIterations;
}

//==============================================================================
void Solver::setIterationsPerPrint(std::size_t _newRatio)
{
  mProperties.mIterationsPerPrint = _newRatio;
}

//==============================================================================
void Solver::setOutStream(std::ostream* _os)
{
  mProperties.mOutStream = _os;
}

//==============================================================================
std::ostream* Solver::getOutStream() const
{
  return mProperties.mOutStream;
}

//==============================================================================
std::size_t Solver::getIterationsPerPrint() const
{
  return mProperties.mIterationsPerPrint;
}

//==============================================================================
void Solver::setPrintFinalResult(bool _print)
{
  mProperties.mPrintFinalResult = _print;
}

//==============================================================================
bool Solver::getPrintFinalResult() const
{
  return mProperties.mPrintFinalResult;
}

//==============================================================================
void Solver::setResultFileName(const std::string& _resultFile)
{
  mProperties.mResultFile = _resultFile;
}

//==============================================================================
const std::string& Solver::getResultFileName() const
{
  return mProperties.mResultFile;
}

}  // namespace optimizer
}  // namespace dart
