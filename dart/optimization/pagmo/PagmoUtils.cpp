/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/optimization/pagmo/PagmoUtils.hpp"

namespace dart {
namespace optimization {

//==============================================================================
std::vector<double> PagmoTypes::convertVector(const Eigen::VectorXd& v) {
  return std::vector<double>(v.data(), v.data() + v.size());
}

//==============================================================================
Eigen::Map<const Eigen::VectorXd> PagmoTypes::convertVector(
    const std::vector<double>& v) {
  return Eigen::Map<const Eigen::VectorXd>(
      v.data(), static_cast<int>(v.size()));
}

//==============================================================================
Population PagmoTypes::convertPopulation(
    const ::pagmo::population& pagmoPop,
    std::shared_ptr<MultiObjectiveProblem> problem) {
  Population pop(problem, pagmoPop.size());

  const auto& pagmoX = pagmoPop.get_x();
  const auto& pagmoF = pagmoPop.get_f();

  for (std::size_t i = 0u; i < pagmoPop.size(); ++i) {
    const Eigen::VectorXd x = convertVector(pagmoX[i]);
    const Eigen::VectorXd f = convertVector(pagmoF[i]);
    pop.set(i, x, f);
  }

  return pop;
}

//==============================================================================
pagmo::population PagmoTypes::convertPopulation(
    const Population& pop, const ::pagmo::problem& pagmoProb) {
  pagmo::population pagmoPop(pagmoProb, pop.getSize());

  for (std::size_t i = 0u; i < pagmoPop.size(); ++i) {
    const std::vector<double> pagmoX = convertVector(pop.getDecisionVector(i));
    const std::vector<double> pagmoF = convertVector(pop.getFitnessVector(i));
    pagmoPop.set_xf(i, pagmoX, pagmoF);
  }

  return pagmoPop;
}

} // namespace optimization
} // namespace dart
