/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/simulation/experimental/dynamics/articulated_body.hpp>

namespace dart::simulation::experimental::dynamics {

void ABAWorkspace::resize(
    std::size_t numLinks, const std::vector<std::size_t>& jointDOFs)
{
  m_linkData.resize(numLinks);
  m_jointData.resize(numLinks);

  for (std::size_t i = 0; i < numLinks; ++i) {
    m_jointData[i].resize(jointDOFs[i]);
  }
}

void ABAWorkspace::reset()
{
  for (auto& link : m_linkData) {
    link.articulatedInertia.setZero();
    link.biasForce.setZero();
    link.spatialVelocity.setZero();
    link.spatialAcceleration.setZero();
    link.partialAcceleration.setZero();
    link.transmittedForce.setZero();
  }

  for (auto& joint : m_jointData) {
    joint.projectedInertiaInverse.setZero();
    joint.totalForce.setZero();
  }
}

} // namespace dart::simulation::experimental::dynamics
