/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/collision/fcl/fcl_engine.hpp"

#include <assimp/scene.h>

#include "dart/collision/collision_result.hpp"
#include "dart/collision/fcl/fcl_primitive_shape_utils.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/geometry/Sphere.hpp"
#include "dart/collision/fcl/fcl_group.hpp"
#include "dart/collision/fcl/fcl_object.hpp"

namespace dart {
namespace collision2 {

//==============================================================================
template <typename S>
std::shared_ptr<FclEngine<S>> FclEngine<S>::Create()
{
  return std::shared_ptr<FclEngine>(new FclEngine());
}

//==============================================================================
template <typename S>
FclEngine<S>::~FclEngine()
{
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& FclEngine<S>::get_type() const
{
  return GetStaticType();
}

//==============================================================================
template <typename S>
const std::string& FclEngine<S>::GetStaticType()
{
  static const std::string type = "fcl";
  return type;
}

//==============================================================================
template <typename S>
GroupPtr<S> FclEngine<S>::create_group()
{
  return std::make_shared<FclGroup<S>>(this);
}

//==============================================================================
template <typename S>
bool FclEngine<S>::collide(ObjectPtr<S> object1, ObjectPtr<S> object2)
{
  auto derived1 = std::dynamic_pointer_cast<FclObject<S>>(object1);
  if (!derived1)
  {
    dterr << "Invalid object\n";
    return false;
  }

  auto derived2 = std::dynamic_pointer_cast<FclObject<S>>(object2);
  if (!derived2)
  {
    dterr << "Invalid object\n";
    return false;
  }

  auto fcl_collision_object1 = derived1->get_fcl_collision_object();
  if (!fcl_collision_object1)
  {
    dterr << "Invalid object\n";
    return false;
  }

  auto fcl_collision_object2 = derived2->get_fcl_collision_object();
  if (!fcl_collision_object2)
  {
    dterr << "Invalid object\n";
    return false;
  }

  FclCollisionRequest<S> request;
  FclCollisionResult<S> result;

  return ::fcl::collide(
      fcl_collision_object1, fcl_collision_object2, request, result);
}

//==============================================================================
template <typename S>
std::shared_ptr<FclCollisionGeometry<S>>
FclEngine<S>::create_fcl_collision_geometry(math::ConstGeometryPtr shape)
{
  return create_fcl_collision_geometry_impl(shape, m_primitive_shape_type);
}

//==============================================================================
template <typename S>
std::shared_ptr<FclCollisionGeometry<S>>
FclEngine<S>::create_fcl_collision_geometry_impl(
    const math::ConstGeometryPtr& shape, FclEngine<S>::PrimitiveShape type)
{
  FclCollisionGeometry<S>* geom = nullptr;
  const auto& shapeType = shape->getType();

  if (auto sphere = shape->as<math::Sphered>())
  {
    const auto radius = sphere->getRadius();

    if (FclEngine<S>::PRIMITIVE == type)
    {
      geom = new FclSphere<S>(radius);
    }
    else
    {
      auto fcl_mesh = new ::fcl::BVHModel<FclOBBRSS<S>>();
      auto fcl_sphere = FclSphere<S>(radius);
      ::fcl::generateBVHModel(*fcl_mesh, fcl_sphere, FclTransform3<S>(), 16, 16);
      geom = fcl_mesh;
    }
  }
  else
  {
    dterr << "[FclEngine<S>::createFCLCollisionGeometry] "
          << "Attempting to create an unsupported shape type [" << shapeType
          << "]. Creating a sphere with 0.1 radius "
          << "instead.\n";

    geom = new FclSphere<S>(0.1);
  }

  assert(geom);

  return std::shared_ptr<FclCollisionGeometry<S>>(geom);
}

} // namespace collision2
} // namespace dart
