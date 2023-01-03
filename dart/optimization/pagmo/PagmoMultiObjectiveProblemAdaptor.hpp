/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_OPTIMIZER_PAGMO_PAGMOMULTIOBJECTIVEPROBLEMADAPTOR_HPP_
#define DART_OPTIMIZER_PAGMO_PAGMOMULTIOBJECTIVEPROBLEMADAPTOR_HPP_

#include <string>
#include <pagmo/pagmo.hpp>
#include "dart/optimization/MultiObjectiveProblem.hpp"

namespace dart {
namespace optimization {

/// Adaptor class for the user defined problem (UDP) that is used by Pagmo2.
///
/// Please see this for the details:
/// https://esa.github.io/pagmo2/docs/cpp/problem.html
class PagmoMultiObjectiveProblemAdaptor
{
public:
  /// Default constructor
  PagmoMultiObjectiveProblemAdaptor() = default;

  /// Constructor
  ///
  /// \param[in] problem Multi-objective problem to be adapted.
  explicit PagmoMultiObjectiveProblemAdaptor(
      std::shared_ptr<MultiObjectiveProblem> problem);

  /// Evaluates fitness, which is a vector of objectives, equality constraints,
  /// and inequality constraints.
  ///
  /// \param[in] x Optimization parameters.
  pagmo::vector_double fitness(const pagmo::vector_double& x) const;

  /// Returns the number of objectives of the optimization problem.
  pagmo::vector_double::size_type get_nobj() const;

  /// Returns the number of equality constraints.
  pagmo::vector_double::size_type get_nec() const;

  /// Returns the number of inequality constraints.
  pagmo::vector_double::size_type get_nic() const;

  /// Returns the dimension of the integer part of the problem.
  pagmo::vector_double::size_type get_nix() const;

  /// Returns the box-bounds. Infinities in the bounds are allowed.
  std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;

  /// Returns the problem name. Currently always returns "PagmoProblem".
  std::string get_name() const;

  /// Serializes to \c ar.
  template <typename Archive>
  void serialize(Archive& ar);

protected:
  std::shared_ptr<MultiObjectiveProblem> mProb;
};

//==============================================================================
template <typename Archive>
void PagmoMultiObjectiveProblemAdaptor::serialize(Archive& ar)
{
  ar(mProb->getSolutionDimension(), mProb->getFitnessDimension());
}

} // namespace optimization
} // namespace dart

#endif // DART_OPTIMIZER_PAGMO_PAGMOMULTIOBJECTIVEPROBLEMADAPTOR_HPP_
