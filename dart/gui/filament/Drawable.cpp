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

#include "dart/gui/filament/Drawable.hpp"

#include <filament/Engine.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include "dart/gui/filament/WorldScene.hpp"

namespace dart {
namespace gui {
namespace flmt {

Drawable::Drawable(WorldScene* worldScene)
  : mWorldScene(worldScene)
{

}

BoxDrawable::BoxDrawable(WorldScene* worldScene, dynamics::BoxShape* boxShape)
  : Drawable(worldScene), mBoxShape(boxShape)
{
  mVB = filament::VertexBuffer::Builder()
      .vertexCount(8)
      .bufferCount(1)
      .attribute(
        filament::VertexAttribute::POSITION,
        0,
        filament::VertexBuffer::AttributeType::FLOAT3,
        0,
        12)
      .attribute(
        filament::VertexAttribute::COLOR,
        0,
        filament::VertexBuffer::AttributeType::UBYTE4,
        8,
        12)
      .normalized(filament::VertexAttribute::COLOR)
      .build(*mEngine);
}

MeshDrawable::MeshDrawable(WorldScene* worldScene, dynamics::MeshShape* meshShape)
  : Drawable(worldScene),
    mMeshShape(meshShape),
    mMeshConvertor(MeshAssimp(*mWorldScene->getEngine()))
{
  if (!meshShape)
    return;

  auto uri = meshShape->getMeshUri2();
  auto retriever = meshShape->getResourceRetriever();

  auto path = retriever->getFilePath(uri);

//  ::utils::Path path2;

  mMeshConvertor.addFromFile(path, mMaterials);

  filament::Engine& engine = *mWorldScene->getEngine();

  auto& tcm = engine.getTransformManager();
  auto& rcm = engine.getRenderableManager();
  auto& em = ::utils::EntityManager::get();

  auto ti = tcm.getInstance(mMeshConvertor.getRenderables()[0]);

  // TODO(JS): local transform

  for (auto renderable : mMeshConvertor.getRenderables())
  {
    auto instance = rcm.getInstance(renderable);
    if (rcm.hasComponent(renderable))
    {
      rcm.setCastShadows(instance, true); // enable shadow
      rcm.setReceiveShadows(instance, false);
      mWorldScene->getScene()->addEntity(renderable);  // TODO(JS): fix;
    }
  }

  int a = 10;
}

} // namespace flmt
} // namespace gui
} // namespace dart
