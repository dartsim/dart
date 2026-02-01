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

#ifndef DART_GUI_SCENE_EXTRACTOR_HPP_
#define DART_GUI_SCENE_EXTRACTOR_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/scene.hpp>

#include <dart/simulation/fwd.hpp>

#include <dart/dynamics/fwd.hpp>

#include <optional>
#include <unordered_map>
#include <vector>

namespace dart {
namespace gui {

struct EntityInfo
{
  dart::dynamics::BodyNode* body_node = nullptr;
  dart::dynamics::SimpleFrame* simple_frame = nullptr;
  dart::dynamics::ShapeNode* shape_node = nullptr;
};

class DART_GUI_API SceneExtractor
{
public:
  Scene extract(const dart::simulation::World& world) const;

  Scene extract(
      const dart::simulation::World& world,
      const std::vector<dart::dynamics::SimpleFrame*>& frames) const;

  const std::unordered_map<uint64_t, EntityInfo>& entityMap() const;

private:
  std::optional<ShapeData> convertShape(
      const dart::dynamics::Shape& shape) const;

  Material extractMaterial(const dart::dynamics::ShapeNode& node) const;

  mutable std::unordered_map<uint64_t, EntityInfo> entity_map_;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_SCENE_EXTRACTOR_HPP_
