/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/collision/ode/detail/OdeCylinderMesh.hpp"

#include "dart/common/Macros.hpp"
#include "dart/math/Helpers.hpp"

#include <Eigen/Core>

#include <algorithm>

#include <cmath>

namespace dart {
namespace collision {
namespace detail {
namespace {

void appendTriangle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    std::vector<double>& vertices,
    std::vector<double>& normals,
    std::vector<int>& indices)
{
  Eigen::Vector3d normal = (p2 - p1).cross(p3 - p1);
  const double norm = normal.norm();
  if (norm > 0.0)
    normal /= norm;

  const auto baseIndex = static_cast<int>(vertices.size() / 3);
  const Eigen::Vector3d points[] = {p1, p2, p3};
  for (const auto& point : points) {
    vertices.push_back(point.x());
    vertices.push_back(point.y());
    vertices.push_back(point.z());
    normals.push_back(normal.x());
    normals.push_back(normal.y());
    normals.push_back(normal.z());
  }

  indices.push_back(baseIndex);
  indices.push_back(baseIndex + 1);
  indices.push_back(baseIndex + 2);
}

} // namespace

//==============================================================================
OdeCylinderMesh::OdeCylinderMesh(
    const OdeCollisionObject* parent,
    double radius,
    double height,
    int slices,
    int stacks)
  : OdeGeom(parent), mOdeTriMeshDataId(nullptr)
{
  buildMesh(radius, height, slices, stacks);

  if (!mOdeTriMeshDataId)
    mOdeTriMeshDataId = dGeomTriMeshDataCreate();

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
OdeCylinderMesh::~OdeCylinderMesh()
{
  dGeomDestroy(mGeomId);

  if (mOdeTriMeshDataId)
    dGeomTriMeshDataDestroy(mOdeTriMeshDataId);
}

//==============================================================================
void OdeCylinderMesh::updateEngineData()
{
  // Do nothing
}

//==============================================================================
void OdeCylinderMesh::buildMesh(
    double radius, double height, int slices, int stacks)
{
  DART_ASSERT(radius > 0.0);
  DART_ASSERT(height > 0.0);

  slices = std::max(slices, 3);
  stacks = std::max(stacks, 1);

  mVertices.clear();
  mNormals.clear();
  mIndices.clear();

  const double zBase = -0.5 * height;
  const double zTop = 0.5 * height;
  const double twoPi = 2.0 * math::constantsd::pi();

  std::vector<double> sinCache(slices + 1);
  std::vector<double> cosCache(slices + 1);
  for (int i = 0; i <= slices; ++i) {
    const double angle = twoPi * static_cast<double>(i) / slices;
    sinCache[i] = std::sin(angle);
    cosCache[i] = std::cos(angle);
  }

  const Eigen::Vector3d centerBottom(0.0, 0.0, zBase);
  const Eigen::Vector3d centerTop(0.0, 0.0, zTop);

  for (int i = 0; i < slices; ++i) {
    const Eigen::Vector3d p1(radius * sinCache[i], radius * cosCache[i], zBase);
    const Eigen::Vector3d p2(
        radius * sinCache[i + 1], radius * cosCache[i + 1], zBase);
    const Eigen::Vector3d p3(radius * sinCache[i], radius * cosCache[i], zTop);
    const Eigen::Vector3d p4(
        radius * sinCache[i + 1], radius * cosCache[i + 1], zTop);

    appendTriangle(centerBottom, p1, p2, mVertices, mNormals, mIndices);
    appendTriangle(centerTop, p4, p3, mVertices, mNormals, mIndices);

    for (int j = 0; j < stacks; ++j) {
      const double zLow = zBase + height * static_cast<double>(j) / stacks;
      const double zHigh = zBase + height * static_cast<double>(j + 1) / stacks;

      const Eigen::Vector3d s1(
          radius * sinCache[i], radius * cosCache[i], zLow);
      const Eigen::Vector3d s2(
          radius * sinCache[i + 1], radius * cosCache[i + 1], zLow);
      const Eigen::Vector3d s3(
          radius * sinCache[i], radius * cosCache[i], zHigh);
      const Eigen::Vector3d s4(
          radius * sinCache[i + 1], radius * cosCache[i + 1], zHigh);

      appendTriangle(s1, s3, s2, mVertices, mNormals, mIndices);
      appendTriangle(s2, s3, s4, mVertices, mNormals, mIndices);
    }
  }
}

} // namespace detail
} // namespace collision
} // namespace dart
