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

#include "dart/gui/vsg/materials.hpp"

namespace dart::gui::vsg {

::vsg::ref_ptr<::vsg::vec4Value> createColorValue(const Eigen::Vector4d& color)
{
  return ::vsg::vec4Value::create(
      ::vsg::vec4(
          static_cast<float>(color.x()),
          static_cast<float>(color.y()),
          static_cast<float>(color.z()),
          static_cast<float>(color.w())));
}

::vsg::ref_ptr<::vsg::StateGroup> createStateGroup(
    const MaterialOptions& options)
{
  auto stateGroup = ::vsg::StateGroup::create();

  if (options.wireframe) {
    auto rasterState = ::vsg::RasterizationState::create();
    rasterState->polygonMode = VK_POLYGON_MODE_LINE;
    stateGroup->add(rasterState);
  }

  if (options.twoSided) {
    auto rasterState = ::vsg::RasterizationState::create();
    rasterState->cullMode = VK_CULL_MODE_NONE;
    stateGroup->add(rasterState);
  }

  return stateGroup;
}

} // namespace dart::gui::vsg
