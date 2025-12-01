/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#pragma once

#include <dart/gui/vsg/Export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vsg/all.h>

#include <algorithm>

namespace dart::gui::vsg {

inline ::vsg::dmat4 convertTransform(const Eigen::Isometry3d& tf)
{
  ::vsg::dmat4 mat;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      mat[c][r] = tf(r, c);
    }
  }
  return mat;
}

inline ::vsg::vec4 toVec4(const Eigen::Vector4d& v)
{
  return ::vsg::vec4(
      static_cast<float>(std::clamp(v[0], 0.0, 1.0)),
      static_cast<float>(std::clamp(v[1], 0.0, 1.0)),
      static_cast<float>(std::clamp(v[2], 0.0, 1.0)),
      static_cast<float>(std::clamp(v[3], 0.0, 1.0)));
}

} // namespace dart::gui::vsg
