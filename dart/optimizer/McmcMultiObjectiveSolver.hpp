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

#ifndef DART_OPTIMIZER_MCMCMULTIOBJECTIVESOLVER_HPP_
#define DART_OPTIMIZER_MCMCMULTIOBJECTIVESOLVER_HPP_

#include "dart/optimizer/MultiObjectiveSolver.hpp"

namespace dart {
namespace optimizer {

class MultiObjectiveProblem;

/// Implementation of Markov chain Mote Carlo method for multi-objective
/// optimization problems.
class McmcMultiObjectiveSolver : public MultiObjectiveSolver
{
public:
  struct UniqueProperties
  {
    /// Constructor
    UniqueProperties();
  };

  struct Properties : MultiObjectiveSolver::Properties, UniqueProperties
  {
    Properties(
        const MultiObjectiveSolver::Properties& solverProperties
        = MultiObjectiveSolver::Properties(),
        const UniqueProperties& descentProperties = UniqueProperties());
  };

  /// Default Constructor
  McmcMultiObjectiveSolver(const Properties& properties = Properties());

  /// Alternative Constructor
  McmcMultiObjectiveSolver(std::shared_ptr<MultiObjectiveProblem> problem);

  /// Destructor
  ~McmcMultiObjectiveSolver() override;

  // Documentation inherited
  bool solve(std::size_t numEvolutions = 1u) override;

  // Documentation inherited
  std::string getType() const override;

  // Documentation inherited
  std::shared_ptr<MultiObjectiveSolver> clone() const override;

  /// Sets the Properties of this McmcMultiObjectiveSolver
  void setProperties(const Properties& properties);

  /// Sets the Properties of this McmcMultiObjectiveSolver
  void setProperties(const UniqueProperties& properties);

  /// Returns the Properties of this GradientDescentSolver
  Properties getGradientDescentProperties() const;

  /// Copies the Properties of another McmcMultiObjectiveSolver
  void copy(const McmcMultiObjectiveSolver& other);

  /// Copies the Properties of another McmcMultiObjectiveSolver
  McmcMultiObjectiveSolver& operator=(const McmcMultiObjectiveSolver& other);

protected:
  /// McmcMultiObjectiveSolver properties
  UniqueProperties mMcmcMultiObjectiveSolverP;

  /// Distribution
  std::uniform_real_distribution<double> mDistribution;
};

} // namespace optimizer
} // namespace dart

#endif // DART_OPTIMIZER_MCMCMULTIOBJECTIVESOLVER_HPP_
