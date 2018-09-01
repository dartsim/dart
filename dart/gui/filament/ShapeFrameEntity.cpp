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

#include "dart/gui/filament/ShapeFrameEntity.hpp"

#include "stb_image.h"

#include <memory>
#include <fstream>
#include <string>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>
#include <filament/TransformManager.h>

#include "dart/gui/filament/Types.hpp"
#include "dart/gui/filament/WorldScene.hpp"

namespace dart {
namespace gui {
namespace flmt {

namespace {

std::shared_ptr<Drawable> createDrawable(
    filament::Engine& engine, filament::Scene& scene, dynamics::Shape* shape)
{
  std::shared_ptr<Drawable> drawable;

  if (shape->is<dynamics::MeshShape>())
  {
    drawable = std::make_shared<MeshDrawable>(
        engine, scene, static_cast<dynamics::MeshShape*>(shape));
  }

  if (drawable)
  {
    scene.addEntity(drawable->getEntity());
  }

//  assert(drawable); // TODO(JS): allow nullptr for now

  return drawable;
}

void destroyDrawable()
{

}

} // namespace

void ShapeFrameEntity::refresh(bool flag)
{
  mUtilized = true;

  auto shape = mShapeFrame->getShape();

  if (shape && mShapeFrame->hasVisualAspect())
  {
    if (mDrawable)
      mDrawable->refresh();
    else
      mDrawable = createDrawable(mEngine, mScene, shape.get());

//    assert(mDrawable);

    if (mDrawable)
    {
      auto& tcm = mEngine.getTransformManager();
      tcm.setTransform(
            tcm.getInstance(mDrawable->getEntity()),
            FilamentTypes::convertIsometry3d(mShapeFrame->getWorldTransform()));
    }
  }
  //  else if (mRenderShapeEntity)
  {
    // TODO(JS): remove
  }
}

} // namespace flmt
} // namespace gui
} // namespace dart
