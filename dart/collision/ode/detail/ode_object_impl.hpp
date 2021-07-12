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

#include "dart/collision/ode/detail/ode_box.hpp"
#include "dart/collision/ode/detail/ode_capsule.hpp"
#include "dart/collision/ode/detail/ode_cylinder.hpp"
#include "dart/collision/ode/detail/ode_heightmap.hpp"
#include "dart/collision/ode/detail/ode_mesh.hpp"
#include "dart/collision/ode/detail/ode_plane.hpp"
#include "dart/collision/ode/detail/ode_sphere.hpp"
#include "dart/collision/ode/ode_conversion.hpp"
#include "dart/collision/ode/ode_group.hpp"
#include "dart/collision/ode/ode_object.hpp"
#include "dart/math/geometry/capsule.hpp"
#include "dart/math/geometry/cuboid.hpp"
#include "dart/math/geometry/cylinder.hpp"
#include "dart/math/geometry/heightmap.hpp"
#include "dart/math/geometry/plane3.hpp"
#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
static detail::OdeGeom* create_ode_geom(
    OdeObject<S>* collObj, const math::Geometry* shape);

//==============================================================================
template <typename S>
math::Isometry3<S> OdeObject<S>::get_pose() const {
  math::Isometry3<S> out = math::Isometry3<S>::Identity();

  const auto* ode_quat = dBodyGetQuaternion(m_ode_body_id);
  out.linear()
      = math::Quaternion<S>(ode_quat[0], ode_quat[1], ode_quat[2], ode_quat[3])
            .toRotationMatrix();

  const auto* ode_pos = dBodyGetPosition(m_ode_body_id);
  out.translation() << ode_pos[0], ode_pos[1], ode_pos[2];

  return out;
}

//==============================================================================
template <typename S>
void OdeObject<S>::set_pose(const math::Isometry3<S>& tf) {
  // If body id is nullptr, this object is immobile. Immobile geom doesn't need
  // to update its pose.
  if (!m_ode_body_id) {
    return;
  }

  const math::Quaternion<S> quat(tf.linear());
  dQuaternion ode_quat;
  ode_quat[0] = quat.w();
  ode_quat[1] = quat.x();
  ode_quat[2] = quat.y();
  ode_quat[3] = quat.z();
  dBodySetQuaternion(m_ode_body_id, ode_quat);

  set_position(tf.translation());
}

//==============================================================================
template <typename S>
math::Vector3<S> OdeObject<S>::get_position() const {
  const auto* ode_pos = dBodyGetPosition(m_ode_body_id);
  return math::Vector3<S>(ode_pos[0], ode_pos[1], ode_pos[2]);
}

//==============================================================================
template <typename S>
void OdeObject<S>::set_position(const math::Vector3<S>& pos) {
  // If body id is nullptr, this object is immobile. Immobile geom doesn't need
  // to update its pose.
  if (!m_ode_body_id) {
    return;
  }

  dBodySetPosition(m_ode_body_id, pos[0], pos[1], pos[1]);
}

//==============================================================================
template <typename S>
OdeObject<S>::OdeObject(OdeGroup<S>* group, math::GeometryPtr shape)
  : Object<S>(group, shape), m_ode_geom{nullptr}, m_ode_body_id(nullptr) {
  // Create detail::OdeGeom according to the shape type.
  // The geometry may have a transform assigned to it which is to
  // be treated as relative transform to the main body.
  m_ode_geom.reset(create_ode_geom(this, shape.get()));

  const auto geomId = m_ode_geom->get_ode_geom_id();
  assert(geomId);

  if (m_ode_geom->is_placeable()) {
    // if the geometry already has a pose, it is to be considered
    // a constant relative pose to the body.
    // Get the geometry pose to ensure this offset is set correctly.
    // Assigning a body to the geometry will overwrite the geometry
    // pose, so back it up first.
    dQuaternion geomRelRot;
    dGeomGetQuaternion(geomId, geomRelRot);
    const dReal* geomRelPos = dGeomGetPosition(geomId);
    assert(geomRelPos);

    // create the body
    m_ode_body_id = dBodyCreate(group->get_ode_engine()->get_ode_world_id());
    // attach geometry to body. This will set the geometry pose to identity.
    dGeomSetBody(geomId, m_ode_body_id);

    // set the offset
    dGeomSetOffsetPosition(geomId, geomRelPos[0], geomRelPos[1], geomRelPos[2]);
    dGeomSetOffsetQuaternion(geomId, geomRelRot);
  }
}

//==============================================================================
template <typename S>
dBodyID OdeObject<S>::get_ode_body_id() const {
  return m_ode_body_id;
}

//==============================================================================
template <typename S>
dGeomID OdeObject<S>::get_ode_geom_id() const {
  return m_ode_geom->get_ode_geom_id();
}

//==============================================================================
template <typename S>
void OdeObject<S>::update_engine_data() {
  m_ode_geom->update_engine_data();
}

//==============================================================================
template <typename S>
detail::OdeGeom* create_ode_geom(
    OdeObject<S>* object, const math::Geometry* shape) {
  detail::OdeGeom* geom = nullptr;

  if (const auto sphere = shape->as<math::Sphere<S>>()) {
    const auto& radius = sphere->get_radius();
    geom = new detail::OdeSphere(object, radius);

  } else if (const auto box = shape->as<math::Cuboid<S>>()) {
    const auto& size = box->get_size();
    geom = new detail::OdeBox(object, size);

  } else if (const auto capsule = shape->as<math::Capsule<S>>()) {
    const auto& radius = capsule->get_radius();
    const auto& height = capsule->get_height();
    geom = new detail::OdeCapsule(object, radius, height);

  } else if (const auto cylinder = shape->as<math::Cylinder<S>>()) {
    const auto& radius = cylinder->get_radius();
    const auto& height = cylinder->get_height();
    geom = new detail::OdeCylinder(object, radius, height);

  } else if (const auto plane = shape->as<math::Plane3<S>>()) {
    const auto& normal = plane->get_normal();
    const auto& offset = plane->get_offset();
    geom = new detail::OdePlane(object, normal, offset);

  } else if (const auto mesh = shape->as<math::TriMesh<S>>()) {
    geom = new detail::OdeMesh(object, mesh, math::Vector3<S>::Ones());

  } else if (const auto heightmap = shape->as<math::Heightmap<S>>()) {
    geom = new detail::OdeHeightmap<S>(object, heightmap);

  } else {
    DART_ERROR(
        "Attempting to create an unsupported shape type [{}]. Creating a "
        "sphere with 0.001 radius instead.",
        shape->get_type());
    geom = new detail::OdeSphere(object, 0.01);
  }
  // TODO(JS): not implemented for EllipsoidShape, ConeShape, MultiSphereShape,
  // and SoftMeshShape.

  assert(geom);
  const auto geomId = geom->get_ode_geom_id();
  dGeomSetData(geomId, object);

  return geom;
}

} // namespace collision
} // namespace dart
