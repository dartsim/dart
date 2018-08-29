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

#include "dart/gui/filament/Cube.hpp"

#include <filament/IndexBuffer.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>
#include <utils/EntityManager.h>

namespace dart {
namespace gui {
namespace flmt {

// clang-format off

const uint32_t Cube::mIndices[] = {
    // solid
    2,0,1, 2,1,3,  // far
    6,4,5, 6,5,7,  // near
    2,0,4, 2,4,6,  // left
    3,1,5, 3,5,7,  // right
    0,4,5, 0,5,1,  // bottom
    2,6,7, 2,7,3,  // top

    // wire-frame
    0,1, 1,3, 3,2, 2,0,     // far
    4,5, 5,7, 7,6, 6,4,     // near
    0,4, 1,5, 3,7, 2,6,
};

const math::float3 Cube::mVertices[] = {
    { -1, -1,  1},  // 0. left bottom far
    {  1, -1,  1},  // 1. right bottom far
    { -1,  1,  1},  // 2. left top far
    {  1,  1,  1},  // 3. right top far
    { -1, -1, -1},  // 4. left bottom near
    {  1, -1, -1},  // 5. right bottom near
    { -1,  1, -1},  // 6. left top near
    {  1,  1, -1}}; // 7. right top near

// clang-format on

Cube::Cube(
    filament::Engine& engine,
    filament::Material const* material,
    math::float3 linearColor,
    bool culling)
  : mEngine(engine), mMaterial(material)
{

  mVertexBuffer = filament::VertexBuffer::Builder()
                      .vertexCount(8)
                      .bufferCount(1)
                      .attribute(
                          filament::VertexAttribute::POSITION,
                          0,
                          filament::VertexBuffer::AttributeType::FLOAT3)
                      .build(engine);

  mIndexBuffer = filament::IndexBuffer::Builder()
                     .indexCount(12 * 2 + 3 * 2 * 6)
                     .build(engine);

  if (mMaterial)
  {
    mMaterialInstanceSolid = mMaterial->createInstance();
    mMaterialInstanceWireFrame = mMaterial->createInstance();
    mMaterialInstanceSolid->setParameter(
        "color",
        filament::RgbaType::LINEAR,
        filament::LinearColorA{
            linearColor.r, linearColor.g, linearColor.b, 0.05f});
    mMaterialInstanceWireFrame->setParameter(
        "color",
        filament::RgbaType::LINEAR,
        filament::LinearColorA{
            linearColor.r, linearColor.g, linearColor.b, 0.25f});
  }

  mVertexBuffer->setBufferAt(
      engine,
      0,
      filament::VertexBuffer::BufferDescriptor(
          mVertices, mVertexBuffer->getVertexCount() * sizeof(mVertices[0])));

  mIndexBuffer->setBuffer(
      engine,
      filament::IndexBuffer::BufferDescriptor(
          mIndices, mIndexBuffer->getIndexCount() * sizeof(uint32_t)));

  utils::EntityManager& em = utils::EntityManager::get();
  mSolidRenderable = em.create();
  filament::RenderableManager::Builder(1)
      .boundingBox({{0, 0, 0}, {1, 1, 1}})
      .material(0, mMaterialInstanceSolid)
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::TRIANGLES,
          mVertexBuffer,
          mIndexBuffer,
          0,
          3 * 2 * 6)
      .priority(7)
      .culling(culling)
      .build(engine, mSolidRenderable);

  mWireFrameRenderable = em.create();
  filament::RenderableManager::Builder(1)
      .boundingBox({{0, 0, 0}, {1, 1, 1}})
      .material(0, mMaterialInstanceWireFrame)
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::LINES,
          mVertexBuffer,
          mIndexBuffer,
          WIREFRAME_OFFSET,
          24)
      .priority(6)
      .culling(culling)
      .build(engine, mWireFrameRenderable);
}

void Cube::mapFrustum(filament::Engine& engine, filament::Camera const* camera)
{
  // the Camera far plane is at infinity, but we want it closer for display
  const math::mat4 vm(camera->getModelMatrix());
  math::mat4 p(vm * inverse(camera->getCullingProjectionMatrix()));
  return mapFrustum(engine, p);
}

void Cube::mapFrustum(filament::Engine& engine, math::mat4 const& transform)
{
  // the Camera far plane is at infinity, but we want it closer for display
  math::mat4f p(transform);
  auto& tcm = engine.getTransformManager();
  tcm.setTransform(tcm.getInstance(mSolidRenderable), p);
  tcm.setTransform(tcm.getInstance(mWireFrameRenderable), p);
}

void Cube::mapAabb(filament::Engine& engine, filament::Box const& box)
{
  math::mat4 p
      = math::mat4::translate(box.center) * math::mat4::scale(box.halfExtent);
  return mapFrustum(engine, p);
}

Cube::~Cube()
{
  mEngine.destroy(mVertexBuffer);
  mEngine.destroy(mIndexBuffer);
  mEngine.destroy(mMaterialInstanceSolid);
  mEngine.destroy(mMaterialInstanceWireFrame);
  // We don't own the material, only instances
  mEngine.destroy(mSolidRenderable);
  mEngine.destroy(mWireFrameRenderable);

  utils::EntityManager& em = utils::EntityManager::get();
  em.destroy(mSolidRenderable);
  em.destroy(mWireFrameRenderable);
}

} // namespace flmt
} // namespace gui
} // namespace dart
