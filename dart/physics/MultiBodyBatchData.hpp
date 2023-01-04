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

#include <dart/physics/MultiBodyJoint.hpp>
#include <dart/physics/MultiBodyLink.hpp>

namespace dart::physics {

template <typename S>
struct MultiBodyBatchData
{
  /// @{ @name Size: number of multibodies

  std::vector<size_t> link_count;
  std::vector<size_t> link_offset;

  std::vector<size_t> dofs_count;
  std::vector<size_t> dofs_offset;

  std::vector<bool> removed;

  /// @}

  /// @{ @name Size: total number of links

  // Link properties
  std::vector<math::SE3<S>> transforms;

  /// @}

  /// @{ @name Size: total dimension of generalized coordinates

  math::VectorX<S> positions;

  /// @}

  void reset()
  {
    link_count = {0};
    link_offset = {0};

    dofs_count = {0};
    dofs_offset = {0};

    removed = {false};

    transforms.clear();

    positions.resize(0);
  }
};

} // namespace dart::physics
