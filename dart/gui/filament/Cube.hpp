/*
 * Copyright (c) 2011-2018, The DART development contributors
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

/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DART_GUI_FILAMENT_CUBE_HPP_
#define DART_GUI_FILAMENT_CUBE_HPP_

#include <vector>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <utils/Entity.h>

namespace dart {
namespace gui {
namespace flmt {

class Cube
{
public:
  Cube(
      filament::Engine& engine,
      filament::Material const* material,
      ::math::float3 linearColor,
      bool culling = true);

  ::utils::Entity getSolidRenderable()
  {
    return mSolidRenderable;
  }

  ::utils::Entity getWireFrameRenderable()
  {
    return mWireFrameRenderable;
  }

  ~Cube();

  void mapFrustum(filament::Engine& engine, filament::Camera const* camera);
  void mapFrustum(filament::Engine& engine, ::math::mat4 const& transform);
  void mapAabb(filament::Engine& engine, filament::Box const& box);

private:
  static constexpr size_t WIREFRAME_OFFSET = 3 * 2 * 6;
  static const uint32_t mIndices[];
  static const ::math::float3 mVertices[];

  filament::Engine& mEngine;
  filament::VertexBuffer* mVertexBuffer = nullptr;
  filament::IndexBuffer* mIndexBuffer = nullptr;
  filament::Material const* mMaterial = nullptr;
  filament::MaterialInstance* mMaterialInstanceSolid = nullptr;
  filament::MaterialInstance* mMaterialInstanceWireFrame = nullptr;
  ::utils::Entity mSolidRenderable;
  ::utils::Entity mWireFrameRenderable;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_CUBE_HPP_
