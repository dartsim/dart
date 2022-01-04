/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/collision/ode/detail/OdeMesh.hpp"

#include "dart/dynamics/MeshShape.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
OdeMesh::OdeMesh(
    const OdeCollisionObject* parent,
    const aiScene* scene,
    const Eigen::Vector3d& scale)
  : OdeGeom(parent), mOdeTriMeshDataId(nullptr)
{
  // Fill vertices, normals, and indices in the ODE friendly data structures.
  fillArrays(scene, scale);

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
void OdeMesh::fillArrays(const aiScene* scene, const Eigen::Vector3d& scale)
{
  mVertices.clear();
  mNormals.clear();
  mIndices.clear();

  // Cound the total numbers of vertices and indices.
  auto mNumVertices = 0u;
  auto mNumIndices = 0u;
  for (auto i = 0u; i < scene->mNumMeshes; ++i)
  {
    const auto mesh = scene->mMeshes[i];

    mNumVertices += mesh->mNumVertices;
    mNumIndices += mesh->mNumFaces;
  }
  mNumVertices *= 3u;
  mNumIndices *= 3u;
  // The number of indices of each face is always 3 because we use the assimp
  // option `aiProcess_Triangulate` when loading meshes.

  mVertices.resize(mNumVertices);
  mNormals.resize(mNumVertices);
  mIndices.resize(mNumIndices);

  auto vertexIndex = 0u;
  auto indexIndex = 0u;
  auto offset = 0u;
  for (auto i = 0u; i < scene->mNumMeshes; ++i)
  {
    const auto mesh = scene->mMeshes[i];

    for (auto j = 0u; j < mesh->mNumVertices; ++j)
    {
      mVertices[vertexIndex] = mesh->mVertices[j].x * scale.x();
      mNormals[vertexIndex++] = mesh->mNormals[j].x;

      mVertices[vertexIndex] = mesh->mVertices[j].y * scale.y();
      mNormals[vertexIndex++] = mesh->mNormals[j].y;

      mVertices[vertexIndex] = mesh->mVertices[j].z * scale.z();
      mNormals[vertexIndex++] = mesh->mNormals[j].z;
    }

    for (auto j = 0u; j < mesh->mNumFaces; ++j)
    {
      mIndices[indexIndex++] = mesh->mFaces[j].mIndices[0] + offset;
      mIndices[indexIndex++] = mesh->mFaces[j].mIndices[1] + offset;
      mIndices[indexIndex++] = mesh->mFaces[j].mIndices[2] + offset;
    }

    offset += mesh->mNumVertices;
  }
}

} // namespace detail
} // namespace collision
} // namespace dart
