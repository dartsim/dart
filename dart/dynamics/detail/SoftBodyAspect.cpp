/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/detail/SoftBodyAspect.hpp"

namespace dart {
namespace dynamics {
namespace detail {

//==============================================================================
SoftBodyAspectProperties::SoftBodyAspectProperties(
    double Kv, double Ke, double DampCoeff,
    const std::vector<PointMass::Properties>& points,
    const std::vector<Eigen::Vector3i>& faces)
  : mKv(Kv),
    mKe(Ke),
    mDampCoeff(DampCoeff),
    mPointProps(points),
    mFaces(faces)
{
  // Do nothing
}

//==============================================================================
void SoftBodyAspectProperties::addPointMass(
    const PointMass::Properties& properties)
{
  mPointProps.push_back(properties);
}

//==============================================================================
bool SoftBodyAspectProperties::connectPointMasses(
    std::size_t i1, std::size_t i2)
{
  if (i1 >= mPointProps.size() || i2 >= mPointProps.size())
  {
    if (mPointProps.size() == 0)
      dtwarn << "[SoftBodyNode::Properties::addConnection] Attempting to "
             << "add a connection between indices " << i1 << " and " << i2
             << ", but there are currently no entries in mPointProps!\n";
    else
      dtwarn << "[SoftBodyNode::Properties::addConnection] Attempting to "
             << "add a connection between indices " << i1 << " and " << i2
             << ", but the entries in mPointProps only go up to "
             << mPointProps.size()-1 << "!\n";
    return false;
  }

  mPointProps[i1].mConnectedPointMassIndices.push_back(i2);
  mPointProps[i2].mConnectedPointMassIndices.push_back(i1);

  return true;
}

//==============================================================================
void SoftBodyAspectProperties::addFace(const Eigen::Vector3i& newFace)
{
  assert(newFace[0] != newFace[1]);
  assert(newFace[1] != newFace[2]);
  assert(newFace[2] != newFace[0]);
  assert(0 <= newFace[0]
      && static_cast<std::size_t>(newFace[0]) < mPointProps.size());
  assert(0 <= newFace[1]
      && static_cast<std::size_t>(newFace[1]) < mPointProps.size());
  assert(0 <= newFace[2]
      && static_cast<std::size_t>(newFace[2]) < mPointProps.size());

  mFaces.push_back(newFace);
}

} // namespace detail
} // namespace dynamics
} // namespace dart
