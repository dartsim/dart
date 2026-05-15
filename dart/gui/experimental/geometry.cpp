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

#include <dart/gui/experimental/geometry.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <utility>

#include <cmath>

namespace dart::gui::experimental {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3f toVector3f(const Eigen::Vector3d& vector)
{
  return vector.cast<float>();
}

Eigen::Vector3f normalizeOr(
    const Eigen::Vector3f& vector, const Eigen::Vector3f& fallback)
{
  const float lengthSquared = vector.squaredNorm();
  if (lengthSquared <= 1e-12f) {
    return fallback;
  }

  return vector / std::sqrt(lengthSquared);
}

Eigen::Vector3f crossProduct(
    const Eigen::Vector3f& lhs, const Eigen::Vector3f& rhs)
{
  return lhs.cross(rhs);
}

void updateBounds(MeshGeometry& mesh, const Eigen::Vector3f& point)
{
  if (!mesh.hasBounds) {
    mesh.boundsMin = point;
    mesh.boundsMax = point;
    mesh.hasBounds = true;
    return;
  }

  mesh.boundsMin = mesh.boundsMin.cwiseMin(point);
  mesh.boundsMax = mesh.boundsMax.cwiseMax(point);
}

void appendVertex(
    MeshGeometry& mesh,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& normal,
    const Eigen::Vector2f& uv = Eigen::Vector2f::Zero())
{
  MeshVertex vertex;
  vertex.position = position;
  vertex.normal = normal;
  vertex.uv = uv;
  mesh.vertices.push_back(vertex);
  updateBounds(mesh, position);
}

void appendFace(
    MeshGeometry& mesh,
    const std::array<Eigen::Vector3f, 8>& points,
    std::uint32_t a,
    std::uint32_t b,
    std::uint32_t c,
    std::uint32_t d,
    const Eigen::Vector3f& normal)
{
  const auto start = static_cast<std::uint32_t>(mesh.vertices.size());
  appendVertex(mesh, points[a], normal, {0.0f, 0.0f});
  appendVertex(mesh, points[b], normal, {1.0f, 0.0f});
  appendVertex(mesh, points[c], normal, {1.0f, 1.0f});
  appendVertex(mesh, points[d], normal, {0.0f, 1.0f});
  appendTriangle(mesh, start, start + 1u, start + 2u);
  appendTriangle(mesh, start, start + 2u, start + 3u);
}

} // namespace

void appendTriangle(
    MeshGeometry& mesh, std::uint32_t a, std::uint32_t b, std::uint32_t c)
{
  mesh.indices.push_back(a);
  mesh.indices.push_back(b);
  mesh.indices.push_back(c);
  mesh.triangles.push_back({a, b, c});
}

MeshIndexRange appendBoxMeshGeometry(
    MeshGeometry& mesh,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& size)
{
  const MeshIndexRange rangeStart{mesh.indices.size(), 0u};
  const Eigen::Vector3f centerPoint = toVector3f(center);
  const Eigen::Vector3f halfSize = toVector3f(size) * 0.5f;
  if (!centerPoint.allFinite() || !halfSize.allFinite()
      || (halfSize.array() <= 0.0f).any()) {
    return rangeStart;
  }

  const std::array<Eigen::Vector3f, 8> points
      = {Eigen::Vector3f{
             centerPoint.x() - halfSize.x(),
             centerPoint.y() - halfSize.y(),
             centerPoint.z() - halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() + halfSize.x(),
             centerPoint.y() - halfSize.y(),
             centerPoint.z() - halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() + halfSize.x(),
             centerPoint.y() + halfSize.y(),
             centerPoint.z() - halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() - halfSize.x(),
             centerPoint.y() + halfSize.y(),
             centerPoint.z() - halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() - halfSize.x(),
             centerPoint.y() - halfSize.y(),
             centerPoint.z() + halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() + halfSize.x(),
             centerPoint.y() - halfSize.y(),
             centerPoint.z() + halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() + halfSize.x(),
             centerPoint.y() + halfSize.y(),
             centerPoint.z() + halfSize.z()},
         Eigen::Vector3f{
             centerPoint.x() - halfSize.x(),
             centerPoint.y() + halfSize.y(),
             centerPoint.z() + halfSize.z()}};

  appendFace(mesh, points, 4, 5, 6, 7, {0.0f, 0.0f, 1.0f});
  appendFace(mesh, points, 0, 3, 2, 1, {0.0f, 0.0f, -1.0f});
  appendFace(mesh, points, 3, 7, 6, 2, {0.0f, 1.0f, 0.0f});
  appendFace(mesh, points, 0, 1, 5, 4, {0.0f, -1.0f, 0.0f});
  appendFace(mesh, points, 1, 2, 6, 5, {1.0f, 0.0f, 0.0f});
  appendFace(mesh, points, 0, 4, 7, 3, {-1.0f, 0.0f, 0.0f});

  return {rangeStart.indexOffset, mesh.indices.size() - rangeStart.indexOffset};
}

void appendEllipsoidMeshGeometry(
    MeshGeometry& mesh,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& radii)
{
  static constexpr std::uint32_t segments = 32u;
  static constexpr std::uint32_t rings = 16u;
  const std::uint32_t start = static_cast<std::uint32_t>(mesh.vertices.size());

  for (std::uint32_t ring = 0; ring <= rings; ++ring) {
    const double theta = -kPi / 2.0 + kPi * ring / rings;
    const double z = std::sin(theta);
    const double radial = std::cos(theta);
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double phi = 2.0 * kPi * segment / segments;
      const Eigen::Vector3d point{
          center.x() + radii.x() * radial * std::cos(phi),
          center.y() + radii.y() * radial * std::sin(phi),
          center.z() + radii.z() * z};
      const Eigen::Vector3d normal{
          radii.x() > 1e-12 ? radial * std::cos(phi) / radii.x() : 0.0,
          radii.y() > 1e-12 ? radial * std::sin(phi) / radii.y() : 0.0,
          radii.z() > 1e-12 ? z / radii.z() : 0.0};
      appendVertex(
          mesh,
          toVector3f(point),
          toVector3f(
              normal.squaredNorm() > 1e-12
                  ? normal.normalized()
                  : Eigen::Vector3d(0.0, 0.0, z >= 0.0 ? 1.0 : -1.0)));
    }
  }

  for (std::uint32_t ring = 0; ring < rings; ++ring) {
    for (std::uint32_t segment = 0; segment < segments; ++segment) {
      const std::uint32_t a = ring * (segments + 1u) + segment;
      const std::uint32_t b = a + 1u;
      const std::uint32_t c = (ring + 1u) * (segments + 1u) + segment;
      const std::uint32_t d = c + 1u;
      appendTriangle(mesh, start + a, start + b, start + c);
      appendTriangle(mesh, start + b, start + d, start + c);
    }
  }
}

MeshGeometry makeBoxMeshGeometry(const Eigen::Vector3d& size)
{
  MeshGeometry mesh;
  mesh.vertices.reserve(24u);
  mesh.indices.reserve(36u);
  mesh.triangles.reserve(12u);
  appendBoxMeshGeometry(mesh, Eigen::Vector3d::Zero(), size);
  return mesh;
}

MeshGeometry makeEllipsoidMeshGeometry(const Eigen::Vector3d& radii)
{
  static constexpr std::uint32_t segments = 32u;
  static constexpr std::uint32_t rings = 16u;

  MeshGeometry mesh;
  mesh.vertices.reserve((rings + 1u) * (segments + 1u));
  mesh.indices.reserve(rings * segments * 6u);
  mesh.triangles.reserve(rings * segments * 2u);
  appendEllipsoidMeshGeometry(mesh, Eigen::Vector3d::Zero(), radii);
  return mesh;
}

MeshGeometry makeMultiSphereMeshGeometry(
    const std::vector<Eigen::Vector3d>& centers,
    const std::vector<double>& radii)
{
  static constexpr std::uint32_t segments = 32u;
  static constexpr std::uint32_t rings = 16u;
  const std::size_t sphereCount = std::min(centers.size(), radii.size());
  const std::size_t verticesPerSphere = (rings + 1u) * (segments + 1u);
  const std::size_t trianglesPerSphere = rings * segments * 2u;

  MeshGeometry mesh;
  mesh.vertices.reserve(sphereCount * verticesPerSphere);
  mesh.indices.reserve(sphereCount * trianglesPerSphere * 3u);
  mesh.triangles.reserve(sphereCount * trianglesPerSphere);

  for (std::size_t i = 0; i < sphereCount; ++i) {
    const double radius = radii[i];
    if (radius <= 0.0 || !std::isfinite(radius) || !centers[i].allFinite()) {
      continue;
    }
    appendEllipsoidMeshGeometry(
        mesh, centers[i], Eigen::Vector3d::Constant(radius));
  }

  return mesh;
}

MeshGeometry makeCylinderMeshGeometry(double radius, double height)
{
  static constexpr std::uint32_t segments = 40u;
  const float halfHeight = static_cast<float>(height * 0.5);
  MeshGeometry mesh;
  mesh.vertices.reserve((segments + 1u) * 4u + 2u);
  mesh.indices.reserve(segments * 12u);
  mesh.triangles.reserve(segments * 4u);

  for (const auto& [z, v] :
       {std::pair{-halfHeight, 0.0f}, std::pair{halfHeight, 1.0f}}) {
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double phi = 2.0 * kPi * segment / segments;
      const float cosPhi = static_cast<float>(std::cos(phi));
      const float sinPhi = static_cast<float>(std::sin(phi));
      appendVertex(
          mesh,
          {static_cast<float>(radius * cosPhi),
           static_cast<float>(radius * sinPhi),
           z},
          normalizeOr({cosPhi, sinPhi, 0.0f}, {1.0f, 0.0f, 0.0f}),
          {static_cast<float>(segment) / static_cast<float>(segments), v});
    }
  }

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    const std::uint32_t a = segment;
    const std::uint32_t b = segment + 1u;
    const std::uint32_t c = (segments + 1u) + segment;
    const std::uint32_t d = c + 1u;
    appendTriangle(mesh, a, b, c);
    appendTriangle(mesh, b, d, c);
  }

  const std::uint32_t bottomCapStart
      = static_cast<std::uint32_t>(mesh.vertices.size());
  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * kPi * segment / segments;
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    appendVertex(
        mesh,
        {static_cast<float>(radius * cosPhi),
         static_cast<float>(radius * sinPhi),
         -halfHeight},
        {0.0f, 0.0f, -1.0f},
        {0.5f + 0.5f * cosPhi, 0.5f + 0.5f * sinPhi});
  }
  const std::uint32_t topCapStart
      = static_cast<std::uint32_t>(mesh.vertices.size());
  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * kPi * segment / segments;
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    appendVertex(
        mesh,
        {static_cast<float>(radius * cosPhi),
         static_cast<float>(radius * sinPhi),
         halfHeight},
        {0.0f, 0.0f, 1.0f},
        {0.5f + 0.5f * cosPhi, 0.5f + 0.5f * sinPhi});
  }
  const std::uint32_t bottomCenter
      = static_cast<std::uint32_t>(mesh.vertices.size());
  appendVertex(
      mesh, {0.0f, 0.0f, -halfHeight}, {0.0f, 0.0f, -1.0f}, {0.5f, 0.5f});
  const std::uint32_t topCenter
      = static_cast<std::uint32_t>(mesh.vertices.size());
  appendVertex(
      mesh, {0.0f, 0.0f, halfHeight}, {0.0f, 0.0f, 1.0f}, {0.5f, 0.5f});
  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    appendTriangle(
        mesh,
        bottomCenter,
        bottomCapStart + segment + 1u,
        bottomCapStart + segment);
    appendTriangle(
        mesh, topCenter, topCapStart + segment, topCapStart + segment + 1u);
  }

  return mesh;
}

MeshGeometry makeConeMeshGeometry(double radius, double height)
{
  static constexpr std::uint32_t segments = 40u;
  const float halfHeight = static_cast<float>(height * 0.5);
  MeshGeometry mesh;
  mesh.vertices.reserve(segments * 3u + segments + 2u);
  mesh.indices.reserve(segments * 6u);
  mesh.triangles.reserve(segments * 2u);

  const auto sideNormal = [&](double phi) {
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    const float z = height > 1e-12 ? static_cast<float>(radius / height) : 0.0f;
    return normalizeOr({cosPhi, sinPhi, z}, {cosPhi, sinPhi, 0.0f});
  };

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    const double phi0 = 2.0 * kPi * segment / segments;
    const double phi1 = 2.0 * kPi * (segment + 1u) / segments;
    const double phiMid = 0.5 * (phi0 + phi1);
    const float u0 = static_cast<float>(segment) / static_cast<float>(segments);
    const float u1
        = static_cast<float>(segment + 1u) / static_cast<float>(segments);
    const float cos0 = static_cast<float>(std::cos(phi0));
    const float sin0 = static_cast<float>(std::sin(phi0));
    const float cos1 = static_cast<float>(std::cos(phi1));
    const float sin1 = static_cast<float>(std::sin(phi1));
    const std::uint32_t base0
        = static_cast<std::uint32_t>(mesh.vertices.size());
    appendVertex(
        mesh,
        {static_cast<float>(radius * cos0),
         static_cast<float>(radius * sin0),
         -halfHeight},
        sideNormal(phi0),
        {u0, 0.0f});
    const std::uint32_t base1
        = static_cast<std::uint32_t>(mesh.vertices.size());
    appendVertex(
        mesh,
        {static_cast<float>(radius * cos1),
         static_cast<float>(radius * sin1),
         -halfHeight},
        sideNormal(phi1),
        {u1, 0.0f});
    const std::uint32_t tip = static_cast<std::uint32_t>(mesh.vertices.size());
    appendVertex(
        mesh,
        {0.0f, 0.0f, halfHeight},
        sideNormal(phiMid),
        {0.5f * (u0 + u1), 1.0f});
    appendTriangle(mesh, base0, base1, tip);
  }

  const std::uint32_t baseStart
      = static_cast<std::uint32_t>(mesh.vertices.size());
  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * kPi * segment / segments;
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    appendVertex(
        mesh,
        {static_cast<float>(radius * cosPhi),
         static_cast<float>(radius * sinPhi),
         -halfHeight},
        {0.0f, 0.0f, -1.0f},
        {0.5f + 0.5f * cosPhi, 0.5f + 0.5f * sinPhi});
  }
  const std::uint32_t center = static_cast<std::uint32_t>(mesh.vertices.size());
  appendVertex(
      mesh, {0.0f, 0.0f, -halfHeight}, {0.0f, 0.0f, -1.0f}, {0.5f, 0.5f});

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    appendTriangle(mesh, center, baseStart + segment + 1u, baseStart + segment);
  }

  return mesh;
}

MeshGeometry makePyramidMeshGeometry(const Eigen::Vector3d& size)
{
  const float halfWidth = static_cast<float>(size.x() * 0.5);
  const float halfDepth = static_cast<float>(size.y() * 0.5);
  const float halfHeight = static_cast<float>(size.z() * 0.5);
  const std::array<Eigen::Vector3f, 5> points
      = {Eigen::Vector3f{0.0f, 0.0f, halfHeight},
         Eigen::Vector3f{-halfWidth, -halfDepth, -halfHeight},
         Eigen::Vector3f{halfWidth, -halfDepth, -halfHeight},
         Eigen::Vector3f{halfWidth, halfDepth, -halfHeight},
         Eigen::Vector3f{-halfWidth, halfDepth, -halfHeight}};

  MeshGeometry mesh;
  mesh.vertices.reserve(18u);
  mesh.indices.reserve(18u);
  mesh.triangles.reserve(6u);

  const auto appendPyramidFace = [&](std::uint32_t a,
                                     std::uint32_t b,
                                     std::uint32_t c,
                                     Eigen::Vector2f uvA,
                                     Eigen::Vector2f uvB,
                                     Eigen::Vector2f uvC) {
    const Eigen::Vector3f normal = normalizeOr(
        crossProduct(points[b] - points[a], points[c] - points[a]),
        {0.0f, 0.0f, 1.0f});
    const auto start = static_cast<std::uint32_t>(mesh.vertices.size());
    appendVertex(mesh, points[a], normal, uvA);
    appendVertex(mesh, points[b], normal, uvB);
    appendVertex(mesh, points[c], normal, uvC);
    appendTriangle(mesh, start, start + 1u, start + 2u);
  };

  appendPyramidFace(0, 1, 2, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendPyramidFace(0, 2, 3, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendPyramidFace(0, 3, 4, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendPyramidFace(0, 4, 1, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendPyramidFace(1, 4, 3, {0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f});
  appendPyramidFace(1, 3, 2, {0.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f});

  return mesh;
}

MeshGeometry makeCapsuleMeshGeometry(double radius, double height)
{
  static constexpr std::uint32_t segments = 32u;
  static constexpr std::uint32_t hemisphereRings = 8u;

  struct Ring
  {
    double z;
    double radius;
  };

  std::vector<Ring> rings;
  rings.reserve(hemisphereRings * 2u + 2u);
  for (std::uint32_t i = 0; i <= hemisphereRings; ++i) {
    const double phi = kPi / 2.0 - (kPi / 2.0) * i / hemisphereRings;
    rings.push_back(
        {height * 0.5 + radius * std::sin(phi), radius * std::cos(phi)});
  }
  if (height > 0.0) {
    rings.push_back({-height * 0.5, radius});
  }
  for (std::uint32_t i = 1; i <= hemisphereRings; ++i) {
    const double phi = -(kPi / 2.0) * i / hemisphereRings;
    rings.push_back(
        {-height * 0.5 + radius * std::sin(phi), radius * std::cos(phi)});
  }

  MeshGeometry mesh;
  mesh.vertices.reserve(rings.size() * (segments + 1u));
  mesh.indices.reserve((rings.size() - 1u) * segments * 6u);
  mesh.triangles.reserve((rings.size() - 1u) * segments * 2u);
  for (const Ring& ring : rings) {
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double theta = 2.0 * kPi * segment / segments;
      const float x = static_cast<float>(ring.radius * std::cos(theta));
      const float y = static_cast<float>(ring.radius * std::sin(theta));
      const float z = static_cast<float>(ring.z);
      const double capCenterZ = std::clamp(ring.z, -height * 0.5, height * 0.5);
      appendVertex(
          mesh,
          {x, y, z},
          normalizeOr(
              {x, y, static_cast<float>(ring.z - capCenterZ)},
              {0.0f, 0.0f, ring.z >= 0.0 ? 1.0f : -1.0f}),
          {static_cast<float>(segment) / static_cast<float>(segments),
           static_cast<float>(mesh.vertices.size() / (segments + 1u))
               / static_cast<float>(rings.size() - 1u)});
    }
  }

  for (std::uint32_t ring = 0; ring + 1u < rings.size(); ++ring) {
    for (std::uint32_t segment = 0; segment < segments; ++segment) {
      const std::uint32_t a = ring * (segments + 1u) + segment;
      const std::uint32_t b = a + 1u;
      const std::uint32_t c = (ring + 1u) * (segments + 1u) + segment;
      const std::uint32_t d = c + 1u;
      appendTriangle(mesh, a, b, c);
      appendTriangle(mesh, b, d, c);
    }
  }

  return mesh;
}

} // namespace dart::gui::experimental
