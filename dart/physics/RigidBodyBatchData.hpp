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

#pragma once

#include <dart/common/Containers.hpp>
#include <dart/common/SoA.hpp>

#include <dart/physics/MultiBodyJoint.hpp>
#include <dart/physics/MultiBodyLink.hpp>

namespace dart::physics {

template <typename S>
struct RigidBodyBatchData
{
  common::SoA<bool, math::SE3<S>> pack;

  /// @{ @name Size: number of multibodies

  std::vector<bool> removed;

  /// @}

  /// @{ @name Size: total number of links

  // Link properties
  std::vector<math::SE3<S>> transforms;

  /// @}

  [[nodiscard]] size_t getSize()
  {
    return removed.size();
  }

  void reset()
  {
    removed = {false};

    transforms.resize(1);
  }

  void addBack()
  {
    pack.pushBack(false, math::SE3<S>());
    removed.emplace_back(false);
    transforms.emplace_back();
  }

  void removeBack()
  {
    if (removed.empty()) {
      return;
    }

    removed.resize(removed.size() - 1);
    transforms.resize(transforms.size() - 1);
  }

  void transferBackTo(size_t index) {}
};

} // namespace dart::physics
