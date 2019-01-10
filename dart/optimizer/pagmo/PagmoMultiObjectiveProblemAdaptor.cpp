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

#include "dart/optimizer/pagmo/PagmoMultiObjectiveProblemAdaptor.hpp"

#include "dart/optimizer/pagmo/PagmoUtils.hpp"

namespace dart {
namespace optimizer {

//==============================================================================
PagmoMultiObjectiveProblemAdaptor::PagmoMultiObjectiveProblemAdaptor(
    std::shared_ptr<MultiObjectiveProblem> problem)
  : mProb(std::move(problem))
{
  assert(mProb);
}

//==============================================================================
pagmo::vector_double PagmoMultiObjectiveProblemAdaptor::fitness(
    const pagmo::vector_double& x) const
{
  const Eigen::VectorXd val = PagmoTypes::convertVector(x);

  assert(mProb.get());
  return PagmoTypes::convertVector(mProb->evaluateFitness(val));
}

//==============================================================================
pagmo::vector_double::size_type PagmoMultiObjectiveProblemAdaptor::get_nobj()
    const
{
  return mProb->getObjectiveDimension();
}

//==============================================================================
pagmo::vector_double::size_type PagmoMultiObjectiveProblemAdaptor::get_nec()
    const
{
  return mProb->getEqConstraintDimension();
}

//==============================================================================
pagmo::vector_double::size_type PagmoMultiObjectiveProblemAdaptor::get_nic()
    const
{
  return mProb->getIneqConstraintDimension();
}

//==============================================================================
std::pair<pagmo::vector_double, pagmo::vector_double>
PagmoMultiObjectiveProblemAdaptor::get_bounds() const
{
  const pagmo::vector_double lb
      = PagmoTypes::convertVector(mProb->getLowerBounds());
  const pagmo::vector_double ub
      = PagmoTypes::convertVector(mProb->getUpperBounds());

  return std::make_pair(lb, ub);
}

//==============================================================================
pagmo::vector_double::size_type PagmoMultiObjectiveProblemAdaptor::get_nix()
    const
{
  pagmo::vector_double::size_type retval = 0u;
  return retval;
}

//==============================================================================
std::string PagmoMultiObjectiveProblemAdaptor::get_name() const
{
  // TODO(JS): Add name field to problem
  return "PagmoMultiObjectiveProblem";
}

} // namespace optimizer
} // namespace dart
