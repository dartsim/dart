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

#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/fcl/backward_compatibility.hpp"
#include "dart/collision/fcl/fcl_engine.hpp"
#include "dart/collision/fcl/fcl_object.hpp"
#include "dart/collision/fcl/fcl_primitive_shape_utils.hpp"
#include "dart/collision/fcl/fcl_scene.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
Contact<Scalar> convert_contact(
    const FclContact<Scalar>& fcl_contact,
    FclObject<Scalar>* object1,
    FclObject<Scalar>* object2,
    const CollisionOption<Scalar>& option)
{
  Contact<Scalar> contact;

  contact.collision_object1 = object1;
  contact.collision_object2 = object2;

  if (option.enable_contact) {
    contact.point = to_vector3<Scalar>(fcl_contact.pos);
    contact.normal = -to_vector3<Scalar>(fcl_contact.normal);
    contact.depth = fcl_contact.penetration_depth;
  }

  return contact;
}

//==============================================================================
template <typename Scalar>
void report_contacts(
    int num_contacts,
    const FclCollisionResult<Scalar>& fcl_result,
    FclObject<Scalar>* b1,
    FclObject<Scalar>* b2,
    const CollisionOption<Scalar>& option,
    CollisionResult<Scalar>& result)
{
  for (auto i = 0; i < num_contacts; ++i) {
    result.add_contact(
        convert_contact(fcl_result.getContact(i), b1, b2, option));

    if (result.get_num_contacts() >= option.max_num_contacts) {
      return;
    }
  }
}

//==============================================================================
template <typename Scalar>
std::shared_ptr<FclEngine<Scalar>> FclEngine<Scalar>::Create()
{
  return std::shared_ptr<FclEngine>(new FclEngine());
}

//==============================================================================
template <typename Scalar>
FclEngine<Scalar>::FclEngine(common::MemoryManager& memory_manager)
  : Engine<Scalar>(memory_manager)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
FclEngine<Scalar>::~FclEngine()
{
  auto& mm = this->m_memory_manager;
  auto& allocator = mm.get_mutable_free_list_allocator();

  for (auto i = 0; i < m_scenes.size(); ++i) {
    allocator.destroy(m_scenes.get_derived(i));
  }
  m_scenes.clear();
}

//==============================================================================
template <typename Scalar>
const std::string& FclEngine<Scalar>::get_type() const
{
  return GetType();
}

//==============================================================================
template <typename Scalar>
const std::string& FclEngine<Scalar>::GetType()
{
  static const std::string type = "fcl";
  return type;
}

//==============================================================================
template <typename Scalar>
bool FclEngine<Scalar>::collide(
    Object<Scalar>* object1,
    Object<Scalar>* object2,
    const CollisionOption<Scalar>& option,
    CollisionResult<Scalar>* result)
{
  auto derived1 = object1->template as<FclObject<Scalar>>();
  if (!derived1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto derived2 = object2->template as<FclObject<Scalar>>();
  if (!derived2) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto fcl_collision_object1 = derived1->get_fcl_collision_object();
  if (!fcl_collision_object1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto fcl_collision_object2 = derived2->get_fcl_collision_object();
  if (!fcl_collision_object2) {
    DART_ERROR("Invalid object");
    return false;
  }

  FclCollisionRequest<Scalar> fcl_request;
  fcl_request.enable_contact = option.enable_contact;
  fcl_request.num_max_contacts = option.max_num_contacts;

  FclCollisionResult<Scalar> fcl_result;

  const auto num_contacts = ::fcl::collide(
      fcl_collision_object1, fcl_collision_object2, fcl_request, fcl_result);

  if (num_contacts > 0 && result) {
    report_contacts(
        num_contacts, fcl_result, derived1, derived2, option, *result);
  }

  return (num_contacts > 0);
}

//==============================================================================
template <typename Scalar>
Scene<Scalar>* FclEngine<Scalar>::create_scene_impl()
{
  auto scene
      = this->m_memory_manager.template construct_using_free<FclScene<Scalar>>(
          this);
  if (scene) {
    m_scenes.push_back_derived(scene);
  }
  return scene;
}

//==============================================================================
template <typename Scalar>
void FclEngine<Scalar>::destroy_scene_impl(Scene<Scalar>* scene)
{
  if (auto casted = dynamic_cast<FclScene<Scalar>*>(scene)) {
    auto& mm = this->m_memory_manager;
    auto& allocator = mm.get_mutable_free_list_allocator();

    m_scenes.erase_derived(casted);
    allocator.destroy(casted);
  }
}

//==============================================================================
template <typename Scalar>
const common::ArrayForBasePtr<Scene<Scalar>>& FclEngine<Scalar>::get_scenes()
    const
{
  return m_scenes;
}

//==============================================================================
template <typename Scalar>
common::ArrayForBasePtr<Scene<Scalar>>& FclEngine<Scalar>::get_mutable_scenes()
{
  return m_scenes;
}

//==============================================================================
template <typename Scalar>
std::shared_ptr<FclCollisionGeometry<Scalar>>
FclEngine<Scalar>::create_fcl_collision_geometry(
    const math::ConstGeometryPtr& shape)
{
  FclCollisionGeometry<Scalar>* geom = nullptr;
  const auto& shapeType = shape->get_type();

  if (auto sphere = shape->as<math::Sphere<Scalar>>()) {
    const auto radius = sphere->get_radius();

    if (FclEngine<Scalar>::PRIMITIVE == m_primitive_shape_type) {
      geom = new FclSphere<Scalar>(radius);
    } else {
      auto fcl_mesh = new ::fcl::BVHModel<FclOBBRSS<Scalar>>();
      auto fcl_sphere = FclSphere<Scalar>(radius);
      ::fcl::generateBVHModel(
          *fcl_mesh, fcl_sphere, get_identity_transform<Scalar>(), 16, 16);
      // TODO(JS): Consider using icosphere
      geom = fcl_mesh;
    }
  } else {
    DART_ERROR(
        "Attempting to create an unsupported shape type [{}]. Creating a "
        "sphere with 0.1 radius instead.",
        shapeType);
    geom = new FclSphere<Scalar>(0.1);
  }

  assert(geom);

  return std::shared_ptr<FclCollisionGeometry<Scalar>>(geom);
}

} // namespace collision
} // namespace dart
