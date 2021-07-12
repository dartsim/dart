/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/collision/ode/detail/ode_mesh.hpp"

#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
OdeMesh::OdeMesh(
    const OdeObject<double>* parent,
    const math::TriMesh<double>* scene,
    const Eigen::Vector3d& scale)
  : OdeGeom(parent), m_ode_tri_mesh_data_id(nullptr) {
  // Fill vertices, normals, and indices in the ODE friendly data structures.
  fill_arrays(scene, scale);

  /// This will hold the vertex data of the triangle mesh
  if (!m_ode_tri_mesh_data_id) {
    m_ode_tri_mesh_data_id = dGeomTriMeshDataCreate();
  }

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildDouble1(
      m_ode_tri_mesh_data_id,
      m_vertices.data(),
      3 * sizeof(double),
      static_cast<int>(m_vertices.size() / 3),
      m_indices.data(),
      static_cast<int>(m_indices.size()),
      3 * sizeof(int),
      m_normals.data());

  m_geom_id = dCreateTriMesh(
      nullptr, m_ode_tri_mesh_data_id, nullptr, nullptr, nullptr);
}

//==============================================================================
OdeMesh::~OdeMesh() {
  dGeomDestroy(m_geom_id);

  if (m_ode_tri_mesh_data_id) {
    dGeomTriMeshDataDestroy(m_ode_tri_mesh_data_id);
  }
}

//==============================================================================
void OdeMesh::update_engine_data() {
  // Do nothing
}

//==============================================================================
void OdeMesh::fill_arrays(
    const math::TriMesh<double>* scene, const Eigen::Vector3d& scale) {
  m_vertices.resize(scene->getVertices().size() * 3);
  m_normals.resize(scene->getVertexNormals().size() * 3);
  m_indices.resize(scene->get_triangles().size() * 3);

  auto index = 0u;
  for (const auto& vertex : scene->getVertices()) {
    m_vertices[index++] = vertex[0] * scale[0];
    m_vertices[index++] = vertex[1] * scale[1];
    m_vertices[index++] = vertex[2] * scale[2];
  }

  index = 0u;
  for (const auto& normal : scene->getVertexNormals()) {
    m_normals[index++] = normal[0];
    m_normals[index++] = normal[1];
    m_normals[index++] = normal[2];
  }

  index = 0u;
  for (const auto& triangle : scene->get_triangles()) {
    m_indices[index++] = static_cast<int>(triangle[0]);
    m_indices[index++] = static_cast<int>(triangle[1]);
    m_indices[index++] = static_cast<int>(triangle[2]);
  }
}

} // namespace detail
} // namespace collision
} // namespace dart
