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

namespace dart::collision {

template <typename S>
class Object
{
public:
  using Scalar = S;

  explicit Object(Scene<S>* scene);

  virtual ~Object();

protected:
  bool init();

  friend class Scene<S>;
  Scene<S>* m_scene;
  common::Entity m_entity;
  math::Geometry3<S>* m_geom;
};

template <typename S, typename T>
class ObjectGeometryEmbedded : public Object<S>
{
public:
  using Scalar = S;

  template <typename... Args>
  explicit ObjectGeometryEmbedded(Scene<S>* scene, Args&&... args);

  const T* getGeometry() const;

  T* getGeometry();

private:
  T* m_geom_derived{nullptr};
};

} // namespace dart::collision

//==============================================================================
// Implementation
//==============================================================================

#include <dart/collision/detail/ObjectComponents.hpp>

#include <dart/math/geometry/Sphere.hpp>

#include <dart/common/Macros.hpp>

namespace dart::collision {

namespace detail {

template <typename ShapeType, typename S = typename ShapeType::Scalar>
struct ShapeComponentDispatcer
{
  static ShapeType* Get(common::EntityManager& em, common::Entity entity)
  {
    if constexpr (std::is_same_v<ShapeType, math::Sphere<S>>) {
      return &em.get<CompSphere<S>>(entity);
    } else {
      return nullptr;
    }
  }
};

} // namespace detail

//==============================================================================
template <typename S>
Object<S>::Object(Scene<S>* scene) : m_scene(scene)
{
  DART_ASSERT(scene);
}

//==============================================================================
template <typename S>
Object<S>::~Object()
{
  auto em = m_scene->getEntityManager();
  em->destroy(m_entity);
}

//==============================================================================
template <typename S>
bool Object<S>::init()
{
  auto em = m_scene->getEntityManager();

  // Create entity
  m_entity = em->create();

  // Create local AABB component
  auto& local_aabb = em->template emplace<detail::CompLocalAabb<S>>(m_entity);
  (void)local_aabb;

  // Create global AABB component
  auto& global_aabb = em->template emplace<detail::CompGlobalAabb<S>>(m_entity);
  (void)global_aabb;

  return true;
}

//==============================================================================
template <typename S, typename T>
template <typename... Args>
ObjectGeometryEmbedded<S, T>::ObjectGeometryEmbedded(
    Scene<S>* scene, Args&&... /* args */)
  : Object<S>(scene)
{
  // auto em = this->m_scene->getEntityManager();
  //  em->template emplace<detail::CompSphere<S>>();
}

//==============================================================================
template <typename S, typename T>
const T* ObjectGeometryEmbedded<S, T>::getGeometry() const
{
  auto em = this->m_scene->getEntityManager();
  return detail::ShapeComponentDispatcer<T>::Get(em, this->m_entity);
}

//==============================================================================
template <typename S, typename T>
T* ObjectGeometryEmbedded<S, T>::getGeometry()
{
  auto em = this->m_scene->getEntityManager();
  return detail::ShapeComponentDispatcer<T>::Get(em, this->m_entity);
}

} // namespace dart::collision
