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

#ifndef DART_GUI_FILAMENT_DRAWABLE_HPP_
#define DART_GUI_FILAMENT_DRAWABLE_HPP_

#include <map>

#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/Renderer.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/vec3.h>

#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/gui/filament/CameraManipulator.hpp"
#include "dart/gui/filament/MeshAssimp.hpp"
#include "dart/gui/filament/Path.hpp"

namespace dart {
namespace gui {
namespace flmt {

class WorldScene;

class Drawable
{
public:
  Drawable(filament::Engine& engine, filament::Scene& scene);
  virtual ~Drawable() = default;

  virtual void refresh();

  virtual ::utils::Entity getEntity() const;

protected:
  filament::VertexBuffer* mVB;
  filament::IndexBuffer* mIB;
  filament::Material* mMaterial;

  filament::Engine& mEngine;
  filament::Scene& mScene;
  ::utils::Entity mEntity;
};

class BoxDrawable : public Drawable
{
public:
  BoxDrawable(filament::Engine& engine, filament::Scene& scene, dynamics::BoxShape* boxShape);

protected:
  dynamics::BoxShape* mBoxShape;
};

class MeshDrawable : public Drawable
{
public:
  MeshDrawable(filament::Engine& engine, filament::Scene& scene, dynamics::MeshShape* meshShape);

  ~MeshDrawable();

  ::utils::Entity getEntity() const override
  {
    return mMeshConvertor.getRenderables()[0];
  }

protected:
  dynamics::MeshShape* mMeshShape;
  MeshAssimp mMeshConvertor;
  std::map<std::string, filament::MaterialInstance*> mMaterials;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_DRAWABLE_HPP_
