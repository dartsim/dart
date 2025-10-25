/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/ode/detail/OdeMesh.hpp"

#include "dart/dynamics/MeshShape.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
OdeMesh::OdeMesh(
    const OdeCollisionObject* parent,
    const std::shared_ptr<math::TriMesh<double>>& mesh,
    const Eigen::Vector3d& scale)
  : OdeGeom(parent), mOdeTriMeshDataId(nullptr)
{
  // Fill vertices, normals, and indices in the ODE friendly data structures.
  fillArraysFromTriMesh(mesh, scale);

  /// This will hold the vertex data of the triangle mesh
  if (!mOdeTriMeshDataId)
    mOdeTriMeshDataId = dGeomTriMeshDataCreate();

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildDouble1(
      mOdeTriMeshDataId,
      mVertices.data(),
      3 * sizeof(double),
      static_cast<int>(mVertices.size() / 3),
      mIndices.data(),
      static_cast<int>(mIndices.size()),
      3 * sizeof(int),
      mNormals.data());

  mGeomId = dCreateTriMesh(0, mOdeTriMeshDataId, nullptr, nullptr, nullptr);
}

//==============================================================================
OdeMesh::~OdeMesh()
{
  dGeomDestroy(mGeomId);

  if (mOdeTriMeshDataId)
    dGeomTriMeshDataDestroy(mOdeTriMeshDataId);
}

//==============================================================================
void OdeMesh::updateEngineData()
{
  // Do nothing
}

//==============================================================================
void OdeMesh::fillArraysFromTriMesh(
    const std::shared_ptr<math::TriMesh<double>>& mesh,
    const Eigen::Vector3d& scale)
{
  mVertices.clear();
  mNormals.clear();
  mIndices.clear();

  if (!mesh) {
    return;
  }

  const auto& vertices = mesh->getVertices();
  const auto& triangles = mesh->getTriangles();
  const auto& normals = mesh->getVertexNormals();

  // Fill vertices
  mVertices.reserve(vertices.size() * 3);
  for (const auto& vertex : vertices) {
    mVertices.push_back(vertex.x() * scale.x());
    mVertices.push_back(vertex.y() * scale.y());
    mVertices.push_back(vertex.z() * scale.z());
  }

  // Fill normals
  mNormals.reserve(normals.size() * 3);
  for (const auto& normal : normals) {
    mNormals.push_back(normal.x());
    mNormals.push_back(normal.y());
    mNormals.push_back(normal.z());
  }

  // Fill indices
  mIndices.reserve(triangles.size() * 3);
  for (const auto& triangle : triangles) {
    mIndices.push_back(static_cast<int>(triangle.x()));
    mIndices.push_back(static_cast<int>(triangle.y()));
    mIndices.push_back(static_cast<int>(triangle.z()));
  }
}

} // namespace detail
} // namespace collision
} // namespace dart
