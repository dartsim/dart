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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/simulation/recording.hpp"

#include "dart/dynamics/skeleton.hpp"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>

namespace {

int totalDofs(std::span<const int> skeletonDofs)
{
  return std::accumulate(skeletonDofs.begin(), skeletonDofs.end(), 0);
}

int dofOffset(std::span<const int> skeletonDofs, int skeletonIndex)
{
  return std::accumulate(
      skeletonDofs.begin(), std::next(skeletonDofs.begin(), skeletonIndex), 0);
}

} // namespace

namespace dart {
namespace simulation {

//==============================================================================
Recording::Recording(std::span<const dynamics::SkeletonPtr> skeletons)
{
  mNumGenCoordsForSkeletons.reserve(skeletons.size());
  std::ranges::transform(
      skeletons,
      std::back_inserter(mNumGenCoordsForSkeletons),
      [](const dynamics::SkeletonPtr& skeleton) {
        return skeleton->getNumDofs();
      });
}

//==============================================================================
Recording::Recording(std::span<const int> skelDofs)
{
  mNumGenCoordsForSkeletons.reserve(skelDofs.size());
  std::ranges::copy(skelDofs, std::back_inserter(mNumGenCoordsForSkeletons));
}

//==============================================================================
Recording::~Recording() {}

//==============================================================================
int Recording::getNumFrames() const
{
  return mBakedStates.size();
}

//==============================================================================
int Recording::getNumSkeletons() const
{
  return mNumGenCoordsForSkeletons.size();
}

//==============================================================================
int Recording::getNumDofs(int _skelIdx) const
{
  return mNumGenCoordsForSkeletons[_skelIdx];
}

//==============================================================================
int Recording::getNumContacts(int _frameIdx) const
{
  return (mBakedStates[_frameIdx].size()
          - totalDofs(std::span<const int>{mNumGenCoordsForSkeletons}))
         / 6;
}

//==============================================================================
Eigen::VectorXd Recording::getConfig(int _frameIdx, int _skelIdx) const
{
  const int index
      = dofOffset(std::span<const int>{mNumGenCoordsForSkeletons}, _skelIdx);
  return mBakedStates[_frameIdx].segment(index, getNumDofs(_skelIdx));
}

//==============================================================================
double Recording::getGenCoord(int _frameIdx, int _skelIdx, int _dofIdx) const
{
  const int index
      = dofOffset(std::span<const int>{mNumGenCoordsForSkeletons}, _skelIdx);
  return mBakedStates[_frameIdx][index + _dofIdx];
}

//==============================================================================
Eigen::Vector3d Recording::getContactPoint(int _frameIdx, int _contactIdx) const
{
  const int contactOffset
      = totalDofs(std::span<const int>{mNumGenCoordsForSkeletons});
  return mBakedStates[_frameIdx].segment(contactOffset + _contactIdx * 6, 3);
}

//==============================================================================
Eigen::Vector3d Recording::getContactForce(int _frameIdx, int _contactIdx) const
{
  const int contactOffset
      = totalDofs(std::span<const int>{mNumGenCoordsForSkeletons});
  return mBakedStates[_frameIdx].segment(
      contactOffset + _contactIdx * 6 + 3, 3);
}

//==============================================================================
void Recording::clear()
{
  mBakedStates.clear();
}

//==============================================================================
void Recording::addState(const Eigen::VectorXd& _state)
{
  mBakedStates.push_back(_state);
}

//==============================================================================
void Recording::updateNumGenCoords(
    std::span<const dynamics::SkeletonPtr> skeletons)
{
  mNumGenCoordsForSkeletons.clear();
  mNumGenCoordsForSkeletons.reserve(skeletons.size());
  std::ranges::transform(
      skeletons,
      std::back_inserter(mNumGenCoordsForSkeletons),
      [](const dynamics::SkeletonPtr& skeleton) {
        return skeleton->getNumDofs();
      });
}

} // namespace simulation
} // namespace dart
