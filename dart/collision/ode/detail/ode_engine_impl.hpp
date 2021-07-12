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

#include "dart/collision/collision_result.hpp"
#include "dart/collision/ode/ode_conversion.hpp"
#include "dart/collision/ode/ode_engine.hpp"
#include "dart/collision/ode/ode_group.hpp"
#include "dart/collision/ode/ode_object.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
Contact<S> convert_contact(
    const dContactGeom& odeContact,
    OdeObject<S>* b1,
    OdeObject<S>* b2,
    const CollisionOption<S>& option);

//==============================================================================
template <typename S>
void report_contacts(
    int num_contacts,
    dContactGeom* contact_geoms,
    OdeObject<S>* b1,
    OdeObject<S>* b2,
    const CollisionOption<S>& option,
    CollisionResult<S>& result);

//==============================================================================
template <typename S>
std::shared_ptr<OdeEngine<S>> OdeEngine<S>::Create() {
  return std::shared_ptr<OdeEngine>(new OdeEngine());
}

//==============================================================================
template <typename S>
OdeEngine<S>::OdeEngine() {
  // Initialize ODE. dInitODE is deprecated.
  const auto initialized = dInitODE2(0);
  DART_ASSERT(initialized, "Failed to initialize the ODE engine.");
  DART_UNUSED(initialized);

  dAllocateODEDataForThread(dAllocateMaskAll);

  m_ode_world_id = dWorldCreate();
  DART_ASSERT(m_ode_world_id, "Failed to create ODE world.");
}

//==============================================================================
template <typename S>
OdeEngine<S>::~OdeEngine() {
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& OdeEngine<S>::get_type() const {
  return GetStaticType();
}

//==============================================================================
template <typename S>
const std::string& OdeEngine<S>::GetStaticType() {
  static const std::string type = "ode";
  return type;
}

//==============================================================================
template <typename S>
GroupPtr<S> OdeEngine<S>::create_group() {
  return std::make_shared<OdeGroup<S>>(this);
}

//==============================================================================
template <typename S>
bool OdeEngine<S>::collide(
    ObjectPtr<S> object1,
    ObjectPtr<S> object2,
    const CollisionOption<S>& option,
    CollisionResult<S>* result) {
  auto derived1 = std::dynamic_pointer_cast<OdeObject<S>>(object1);
  if (!derived1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto derived2 = std::dynamic_pointer_cast<OdeObject<S>>(object2);
  if (!derived2) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto ode_geom_id1 = derived1->get_ode_geom_id();
  if (!ode_geom_id1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto ode_geom_id2 = derived2->get_ode_geom_id();
  if (!ode_geom_id2) {
    DART_ERROR("Invalid object");
    return false;
  }

  dContactGeom* ode_contact_geom = new dContactGeom[option.max_num_contacts];
  const int num_contacts = dCollide(
      ode_geom_id1,
      ode_geom_id2,
      option.max_num_contacts,
      ode_contact_geom,
      sizeof(dContactGeom));

  if (result) {
    // TODO(JS):
  }

  delete[] ode_contact_geom;
  return (num_contacts > 0);
}

//==============================================================================
template <typename S>
dWorldID OdeEngine<S>::get_ode_world_id() const {
  return m_ode_world_id;
}

//==============================================================================
template <typename S>
void report_contacts(
    int num_contacts,
    dContactGeom* contact_geoms,
    OdeObject<S>* b1,
    OdeObject<S>* b2,
    const CollisionOption<S>& option,
    CollisionResult<S>& result) {
  if (0u == num_contacts) {
    return;
  }

  // For binary check, return after adding the first contact point to the result
  // without the checkings of repeatidity and co-linearity.
  if (1u == option.max_num_contacts) {
    result.addContact(convert_contact(contact_geoms[0], b1, b2, option));
    return;
  }

  for (auto i = 0; i < num_contacts; ++i) {
    result.addContact(convert_contact(contact_geoms[i], b1, b2, option));

    if (result.getNumContacts() >= option.maxNumContacts) {
      return;
    }
  }
}

//==============================================================================
template <typename S>
Contact<S> convert_contact(
    const dContactGeom& odeContact,
    OdeObject<S>* object1,
    OdeObject<S>* object2,
    const CollisionOption<S>& option) {
  Contact<S> contact;

  contact.collision_object1 = object1;
  contact.collision_object2 = object2;

  if (option.enable_contact) {
    contact.point = to_vector3<S>(odeContact.pos);
    contact.normal = to_vector3<S>(odeContact.normal);
    contact.depth = odeContact.depth;
  }

  return contact;
}

} // namespace collision
} // namespace dart
