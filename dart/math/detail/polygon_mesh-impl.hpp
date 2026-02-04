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

#ifndef DART_MATH_DETAIL_POLYGONMESH_IMPL_HPP_
#define DART_MATH_DETAIL_POLYGONMESH_IMPL_HPP_

#include <dart/math/polygon_mesh.hpp>

#include <algorithm>
#include <limits>
#include <span>
#include <utility>

#include <cmath>
#include <cstddef>

namespace dart {
namespace math {

namespace detail {

struct ProjectedPoint
{
  double x{0.0};
  double y{0.0};
};

inline double cross(
    const ProjectedPoint& a, const ProjectedPoint& b, const ProjectedPoint& c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline double polygonArea2(std::span<const ProjectedPoint> points)
{
  const std::size_t count = points.size();
  double area2 = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& current = points[i];
    const auto& next = points[(i + 1) % count];
    area2 += current.x * next.y - next.x * current.y;
  }
  return area2;
}

template <
    typename Index,
    typename Vector3,
    typename Triangle,
    typename TriangleAllocator>
bool triangulateFaceEarClipping(
    std::span<const Index> face,
    std::span<const Vector3> vertices,
    std::vector<Triangle, TriangleAllocator>& triangles)
{
  triangles.clear();
  const std::size_t count = face.size();
  if (count < 3) {
    return false;
  }

  if (count == 3) {
    triangles.emplace_back(face[0], face[1], face[2]);
    return true;
  }

  double normalX = 0.0;
  double normalY = 0.0;
  double normalZ = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& current = vertices[face[i]];
    const auto& next = vertices[face[(i + 1) % count]];
    const double cx = static_cast<double>(current.x());
    const double cy = static_cast<double>(current.y());
    const double cz = static_cast<double>(current.z());
    const double nx = static_cast<double>(next.x());
    const double ny = static_cast<double>(next.y());
    const double nz = static_cast<double>(next.z());
    normalX += (cy - ny) * (cz + nz);
    normalY += (cz - nz) * (cx + nx);
    normalZ += (cx - nx) * (cy + ny);
  }

  const double absX = std::abs(normalX);
  const double absY = std::abs(normalY);
  const double absZ = std::abs(normalZ);
  if (absX <= std::numeric_limits<double>::epsilon()
      && absY <= std::numeric_limits<double>::epsilon()
      && absZ <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  enum class DropAxis
  {
    X,
    Y,
    Z
  };

  DropAxis dropAxis = DropAxis::Z;
  if (absX >= absY && absX >= absZ) {
    dropAxis = DropAxis::X;
  } else if (absY >= absZ) {
    dropAxis = DropAxis::Y;
  }

  std::vector<ProjectedPoint> projected;
  projected.reserve(count);
  double scale = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& vertex = vertices[face[i]];
    ProjectedPoint point;
    if (dropAxis == DropAxis::X) {
      point.x = static_cast<double>(vertex.y());
      point.y = static_cast<double>(vertex.z());
    } else if (dropAxis == DropAxis::Y) {
      point.x = static_cast<double>(vertex.x());
      point.y = static_cast<double>(vertex.z());
    } else {
      point.x = static_cast<double>(vertex.x());
      point.y = static_cast<double>(vertex.y());
    }
    scale = std::max(scale, std::max(std::abs(point.x), std::abs(point.y)));
    projected.emplace_back(point);
  }

  if (scale < 1.0) {
    scale = 1.0;
  }
  const double eps = std::max(
      1e-12, std::numeric_limits<double>::epsilon() * scale * scale * 100.0);

  const double area2 = polygonArea2(std::span<const ProjectedPoint>{projected});
  if (std::abs(area2) <= eps) {
    return false;
  }
  const bool isCCW = area2 > 0.0;

  std::vector<std::size_t> polygon(count);
  for (std::size_t i = 0; i < count; ++i) {
    polygon[i] = i;
  }

  bool removedColinear = true;
  while (removedColinear && polygon.size() > 3) {
    removedColinear = false;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const std::size_t prev
          = polygon[(i + polygon.size() - 1) % polygon.size()];
      const std::size_t curr = polygon[i];
      const std::size_t next = polygon[(i + 1) % polygon.size()];
      if (std::abs(cross(projected[prev], projected[curr], projected[next]))
          <= eps) {
        polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
        removedColinear = true;
        break;
      }
    }
  }

  if (polygon.size() < 3) {
    return false;
  }

  triangles.reserve(polygon.size() - 2);
  const std::size_t maxIterations = polygon.size() * polygon.size();
  std::size_t iterations = 0;

  auto isConvex = [&](std::size_t prev, std::size_t curr, std::size_t next) {
    const double turn
        = cross(projected[prev], projected[curr], projected[next]);
    return isCCW ? (turn > eps) : (turn < -eps);
  };

  auto containsPoint = [&](std::size_t prev,
                           std::size_t curr,
                           std::size_t next,
                           std::size_t idx) -> bool {
    const auto& point = projected[idx];
    const double c1 = cross(projected[prev], projected[curr], point);
    const double c2 = cross(projected[curr], projected[next], point);
    const double c3 = cross(projected[next], projected[prev], point);
    if (isCCW) {
      return c1 >= -eps && c2 >= -eps && c3 >= -eps;
    }
    return c1 <= eps && c2 <= eps && c3 <= eps;
  };

  while (polygon.size() > 3 && iterations < maxIterations) {
    bool earFound = false;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const std::size_t prev
          = polygon[(i + polygon.size() - 1) % polygon.size()];
      const std::size_t curr = polygon[i];
      const std::size_t next = polygon[(i + 1) % polygon.size()];

      if (!isConvex(prev, curr, next)) {
        continue;
      }

      bool hasInteriorPoint = false;
      for (std::size_t j = 0; j < polygon.size(); ++j) {
        const std::size_t idx = polygon[j];
        if (idx == prev || idx == curr || idx == next) {
          continue;
        }
        if (containsPoint(prev, curr, next, idx)) {
          hasInteriorPoint = true;
          break;
        }
      }
      if (hasInteriorPoint) {
        continue;
      }

      triangles.emplace_back(face[prev], face[curr], face[next]);
      polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
      earFound = true;
      break;
    }

    if (!earFound) {
      return false;
    }
    ++iterations;
  }

  if (polygon.size() == 3) {
    triangles.emplace_back(
        face[polygon[0]], face[polygon[1]], face[polygon[2]]);
    return true;
  }

  return false;
}

} // namespace detail

//==============================================================================
template <typename S>
PolygonMesh<S>::PolygonMesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::reserveFaces(std::size_t n)
{
  mFaces.reserve(n);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::reserveVertices(std::size_t n)
{
  this->mVertices.reserve(n);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertex(S x, S y, S z)
{
  this->mVertices.emplace_back(x, y, z);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertex(const Vector3& vertex)
{
  this->mVertices.push_back(vertex);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::reserveVertexNormals(std::size_t n)
{
  this->mVertexNormals.reserve(n);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertexNormal(S x, S y, S z)
{
  this->mVertexNormals.emplace_back(x, y, z);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertexNormal(const Vector3& normal)
{
  this->mVertexNormals.push_back(normal);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addFace(const Face& face)
{
  mFaces.push_back(face);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addFace(Face&& face)
{
  mFaces.push_back(std::move(face));
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addFace(std::initializer_list<Index> indices)
{
  mFaces.emplace_back(indices);
}

//==============================================================================
template <typename S>
bool PolygonMesh<S>::hasFaces() const
{
  return !mFaces.empty();
}

//==============================================================================
template <typename S>
std::size_t PolygonMesh<S>::getNumFaces() const
{
  return mFaces.size();
}

//==============================================================================
template <typename S>
const typename PolygonMesh<S>::Faces& PolygonMesh<S>::getFaces() const
{
  return mFaces;
}

//==============================================================================
template <typename S>
typename PolygonMesh<S>::Faces& PolygonMesh<S>::getFaces()
{
  return mFaces;
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::clear()
{
  mFaces.clear();
  Base::clear();
}

//==============================================================================
template <typename S>
typename PolygonMesh<S>::TriMeshType PolygonMesh<S>::triangulate() const
{
  TriMeshType triMesh;

  triMesh.getVertices() = this->mVertices;
  if (this->hasVertexNormals()) {
    triMesh.getVertexNormals() = this->mVertexNormals;
  }

  std::size_t triangleCount = 0;
  for (const auto& face : mFaces) {
    if (face.size() >= 3) {
      triangleCount += face.size() - 2;
    }
  }
  triMesh.reserveTriangles(triangleCount);

  typename TriMeshType::Triangles triangles;
  for (const auto& face : mFaces) {
    if (face.size() < 3) {
      continue;
    }
    triangles.clear();
    if (!detail::triangulateFaceEarClipping(
            std::span<const Index>{face},
            std::span<const Vector3>{this->mVertices},
            triangles)) {
      const Index v0 = face[0];
      for (std::size_t i = 1; i + 1 < face.size(); ++i) {
        triMesh.addTriangle(v0, face[i], face[i + 1]);
      }
      continue;
    }
    for (const auto& triangle : triangles) {
      triMesh.addTriangle(triangle[0], triangle[1], triangle[2]);
    }
  }

  if (triMesh.hasTriangles()
      && triMesh.getVertexNormals().size() != triMesh.getVertices().size()) {
    triMesh.computeVertexNormals();
  }

  return triMesh;
}

} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_POLYGONMESH_IMPL_HPP_
