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

#include <dart/collision/Fwd.hpp>

#include <dart/common/EntityManager.hpp>
#include <dart/common/allocator/StdAllocator.hpp>

namespace dart::collision {

template <typename S>
class Scene
{
public:
  using Scalar = S;

  explicit Scene(Engine<S>* engine);

  virtual ~Scene();

  template <typename T, typename... Args>
  ObjectGeometryEmbedded<S, T>* createObject(Args&&... args);

  void update();

  const common::EntityManager* getEntityManager() const;

  common::EntityManager* getEntityManager();

  const Engine<S>* getEngine() const;

  Engine<S>* getEngine();

private:
  Engine<S>* m_engine;
  common::Entity m_entity;
  std::vector<Object<S>*, common::StdAllocator<Object<S>*>> m_objects;
};

} // namespace dart::collision

//==============================================================================
// Implementation
//==============================================================================

#include <dart/collision/detail/ObjectComponents.hpp>

#include <dart/common/Macros.hpp>

namespace dart::collision {

//==============================================================================
template <typename S>
Scene<S>::Scene(Engine<S>* engine) : m_engine(engine)
{
  DART_ASSERT(engine);
  auto em = getEntityManager();
  m_entity = em->create();
}

//==============================================================================
template <typename S>
Scene<S>::~Scene()
{
  auto& allocator = m_engine->getAllocator();

  for (auto object : m_objects) {
    allocator.destroy(object);
  }
  m_objects.clear();

  auto em = getEntityManager();
  em->destroy(m_entity);
}

//==============================================================================
template <typename S>
template <typename T, typename... Args>
ObjectGeometryEmbedded<S, T>* Scene<S>::createObject(Args&&... args)
{
  auto& allocator = m_engine->getAllocator();

  auto object = allocator.template construct<ObjectGeometryEmbedded<Scalar, T>>(
      this, std::forward<Args>(args)...);
  if (!object) {
    return nullptr;
  }

  if (!object->init()) {
    return nullptr;
  }

  m_objects.push_back(object);
  return object;
}

//==============================================================================
template <typename S>
void Scene<S>::update()
{
  auto em = getEntityManager();

  auto local_aabb_view = em->template view<detail::CompLocalAabb<S>>();
  auto global_aabb_view = em->template view<detail::CompGlobalAabb<S>>();
  auto sphere_view = em->template view<detail::CompSphere<S>>();

  // Update local AABBs
  local_aabb_view.each([](const auto entity, auto& c_aabb) {
    (void)entity;
    //    std::cout << "[DEBUG] entity id : " << static_cast<size_t>(entity) <<
    //    std::endl;
    std::cout << "[DEBUG] local aabb dirty: " << c_aabb.dirty << std::endl;
  });

  // Update global AABBs
  global_aabb_view.each([](const auto entity, auto& c_aabb) {
    (void)entity;
    //    std::cout << "[DEBUG] entity id : " << static_cast<size_t>(entity) <<
    //    std::endl;
    std::cout << "[DEBUG] global aabb dirty: " << c_aabb.dirty << std::endl;
  });

  sphere_view.each([](const auto entity, auto& c_sphere) {
    (void)entity;
    //    std::cout << "[DEBUG] entity id : " << static_cast<size_t>(entity) <<
    //    std::endl;
    std::cout << "[DEBUG] sphere radius: " << c_sphere.sphere.getRadius()
              << std::endl;
  });
}

//==============================================================================
template <typename S>
const common::EntityManager* Scene<S>::getEntityManager() const
{
  return m_engine->getEntityManager();
}

//==============================================================================
template <typename S>
common::EntityManager* Scene<S>::getEntityManager()
{
  return m_engine->getEntityManager();
}

//==============================================================================
template <typename S>
const Engine<S>* Scene<S>::getEngine() const
{
  return m_engine;
}

//==============================================================================
template <typename S>
Engine<S>* Scene<S>::getEngine()
{
  return m_engine;
}

} // namespace dart::collision
