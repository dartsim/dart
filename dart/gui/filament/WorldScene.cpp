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

#include "dart/gui/filament/WorldScene.hpp"

#include <deque>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <math/TVecHelpers.h>
#include <math/norm.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>

#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace gui {
namespace flmt {

namespace {

static constexpr bool ENABLE_SHADOWS = true;
static constexpr uint8_t GROUND_SHADOW_PACKAGE[] = {
#include "generated/material/groundShadow.inc"
};

static GroundPlane createGroundPlane(filament::Engine* engine)
{
  filament::Material* shadowMaterial
      = filament::Material::Builder()
            .package(
                (void*)GROUND_SHADOW_PACKAGE, sizeof(GROUND_SHADOW_PACKAGE))
            .build(*engine);

  const static uint32_t indices[]{0, 1, 2, 2, 3, 0};
  const static ::math::float3 vertices[]{
      {-10, 0, -10}, {-10, 0, 10}, {10, 0, 10}, {10, 0, -10},
  };
  ::math::short4 tbn = ::math::packSnorm16(
      normalize(
          positive(
              ::math::mat3f{::math::float3{1.0f, 0.0f, 0.0f},
                            ::math::float3{0.0f, 0.0f, 1.0f},
                            ::math::float3{0.0f, 1.0f, 0.0f}}
                  .toQuaternion()))
          .xyzw);
  const static ::math::short4 normals[]{tbn, tbn, tbn, tbn};
  filament::VertexBuffer* vertexBuffer
      = filament::VertexBuffer::Builder()
            .vertexCount(4)
            .bufferCount(2)
            .attribute(
                filament::VertexAttribute::POSITION,
                0,
                filament::VertexBuffer::AttributeType::FLOAT3)
            .attribute(
                filament::VertexAttribute::TANGENTS,
                1,
                filament::VertexBuffer::AttributeType::SHORT4)
            .normalized(filament::VertexAttribute::TANGENTS)
            .build(*engine);
  vertexBuffer->setBufferAt(
      *engine,
      0,
      filament::VertexBuffer::BufferDescriptor(
          vertices, vertexBuffer->getVertexCount() * sizeof(vertices[0])));
  vertexBuffer->setBufferAt(
      *engine,
      1,
      filament::VertexBuffer::BufferDescriptor(
          normals, vertexBuffer->getVertexCount() * sizeof(normals[0])));
  filament::IndexBuffer* indexBuffer
      = filament::IndexBuffer::Builder().indexCount(6).build(*engine);
  indexBuffer->setBuffer(
      *engine,
      filament::IndexBuffer::BufferDescriptor(
          indices, indexBuffer->getIndexCount() * sizeof(uint32_t)));

  auto& em = ::utils::EntityManager::get();
  ::utils::Entity renderable = em.create();
  filament::RenderableManager::Builder(1)
      .boundingBox({{0, 0, 0}, {10, 1e-4f, 10}})
      .material(0, shadowMaterial->getDefaultInstance())
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::TRIANGLES,
          vertexBuffer,
          indexBuffer,
          0,
          6)
      .culling(false)
      .receiveShadows(ENABLE_SHADOWS)
      .castShadows(false)
      .build(*engine, renderable);

  auto& tcm = engine->getTransformManager();
  tcm.setTransform(
      tcm.getInstance(renderable),
      ::math::mat4f::translate(::math::float3{0, -1, -4}));
  return {
      .vb = vertexBuffer,
      .ib = indexBuffer,
      .mat = shadowMaterial,
      .renderable = renderable,
  };
}

} // (anonymous) namespace

WorldScene::WorldScene(simulation::WorldPtr world) : mWorld(std::move(world))
{
  // Do nothing
}

WorldScene::~WorldScene()
{
  // Do nothing
}

void WorldScene::setScene(filament::Engine* engine, filament::Scene* scene)
{
  if (scene == mScene)
    return;

  if (!engine)
    destroyScene();

  mEngine = engine;
  mScene = scene;
  if (mScene)
  {
    setupScene();
  }
  else
  {
    destroyScene();
    // TODO(JS): clear?
  }
}

void WorldScene::refresh()
{
  for (std::size_t i = 0u; i < mStepsPerFrame; ++i)
    mWorld->step();

  refreshSkeletons();
  clearUnusedNodes();
}

void WorldScene::setupScene()
{
  if (!mEngine)
    return;

  assert(mScene);

  auto& em = ::utils::EntityManager::get();

  // Add light sources into the scene.
  mLight = em.create();
  filament::LightManager::Builder(filament::LightManager::Type::SUN)
      .color(
          filament::Color::toLinear<filament::ACCURATE>(
              filament::sRGBColor(0.98f, 0.92f, 0.89f)))
      .intensity(110000)
      .direction({0.7, -1, -0.8})
      .sunAngularRadius(1.9f)
      .castShadows(true)
      .build(*mEngine, mLight);
  mScene->addEntity(mLight);

  mPlane = createGroundPlane(mEngine);
  mScene->addEntity(mPlane.renderable);

  refresh();
}

void WorldScene::destroyScene()
{
  if (!mEngine)
    return;

  mScene->remove(mPlane.renderable);
  mScene->remove(mLight);

  mFrameToNode.clear();

  mEngine = nullptr;
  mScene = nullptr;
}

void WorldScene::refreshSkeletons()
{
  if (!mWorld)
    return;

  for (auto i = 0u; i < mWorld->getNumSkeletons(); ++i)
  {
    const dynamics::SkeletonPtr& skel = mWorld->getSkeleton(i);
    for (auto j = 0u; j < skel->getNumTrees(); ++j)
      refreshBaseFrameNode(skel->getRootBodyNode(j));
  }
}

void WorldScene::refreshBaseFrameNode(dynamics::Frame* frame)
{
  std::deque<dynamics::Frame*> frames;
  frames.push_back(frame);
  while (!frames.empty())
  {
    dynamics::Frame* nextFrame = frames.front();
    frames.pop_front();

    if (nextFrame->isShapeFrame())
      refreshShapeFrameNode(nextFrame);

    const std::set<dynamics::Frame*>& childFrames = nextFrame->getChildFrames();
    for (auto* child : childFrames)
      frames.push_back(child);
  }
}

void WorldScene::refreshShapeFrameNode(dynamics::Frame* frame)
{
  std::pair<NodeMap::iterator, bool> insertion
      = mFrameToNode.insert(std::make_pair(frame, nullptr));
  NodeMap::iterator& itr = insertion.first;
  bool exist = !insertion.second;

  if (exist)
  {
    ShapeFrameEntity* sfEntity = itr->second.get();
    if (!sfEntity)
      return;

    sfEntity->refresh(true);

    return;
  }

  assert(frame->isShapeFrame());

  assert(mEngine);
  assert(mScene);

  //auto entity = ::utils::EntityManager::get().create();
  std::unique_ptr<ShapeFrameEntity> sfEntity
      = common::make_unique<ShapeFrameEntity>(
          *mEngine, *mScene, frame->asShapeFrame());

  itr->second = std::move(sfEntity);
  //mScene->addEntity(entity);
}

void WorldScene::clearUnusedNodes()
{
}

} // namespace flmt
} // namespace gui
} // namespace dart
