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

#include "renderable_factory.hpp"

#include "colors.hpp"

#include <dart/gui/detail/geometry.hpp>

#include <Eigen/Core>
#include <backend/BufferDescriptor.h>
#include <backend/DriverEnums.h>
#include <filament/Box.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/VertexBuffer.h>
#include <geometry/SurfaceOrientation.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace dart::gui::detail {

using dart::gui::appendBoxMeshGeometry;
using dart::gui::DebugLineDescriptor;
using dart::gui::DebugTriangleDescriptor;
using dart::gui::GeometryDescriptor;
using dart::gui::makeCapsuleMeshGeometry;
using dart::gui::makeConeMeshGeometry;
using dart::gui::makeCylinderMeshGeometry;
using dart::gui::makeEllipsoidMeshGeometry;
using dart::gui::makeMultiSphereMeshGeometry;
using dart::gui::makePyramidMeshGeometry;
using dart::gui::MeshAlphaMode;
using dart::gui::MeshGeometry;
using dart::gui::MeshIndexRange;
using dart::gui::MeshMaterialDescriptor;
using dart::gui::MeshPartDescriptor;
using dart::gui::RenderableDescriptor;
using dart::gui::ShapeKind;
using ::filament::math::float2;
using ::filament::math::float3;
using ::filament::math::float4;
using utils::EntityManager;

namespace {

std::size_t recordGeometryRange(
    Renderable& renderable,
    std::size_t primitive,
    std::size_t offset,
    std::size_t count,
    bool triangles,
    bool wireframeEligible = true)
{
  if (renderable.geometryRanges.size() <= primitive)
    renderable.geometryRanges.resize(primitive + 1);
  renderable.geometryRanges[primitive]
      = {offset, count, triangles, wireframeEligible};
  return offset;
}

constexpr int kDebugLineTubeSegments = 8;
constexpr double kTwoPi = 6.2831853071795864769;
constexpr double kMinDebugLineLength = 1e-9;
constexpr double kMinDebugTriangleArea = 0.0;
constexpr float kDefaultDielectricMetallic = 0.0f;
constexpr float kDefaultMatteRoughness = 0.82f;

bool isOpaqueShapeAlpha(double alpha)
{
  // OSG classifies primitive alpha before narrowing its colors to floats.
  return std::abs(alpha) > 1.0 - 1e-9;
}

struct Vertex
{
  float3 position;
  ::filament::math::short4 tangent;
  ::filament::math::float2 uv = {0.0f, 0.0f};
  float4 color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct DebugVertex
{
  ::filament::math::float3 position;
  // Linear dark colors need more precision than normalized bytes: authored
  // sRGB .025 becomes .001935, which UBYTE4 would round to black.
  ::filament::math::half4 color = {0.0f, 0.0f, 0.0f, 0.0f};
  ::filament::math::float2 uv = {0.0f, 0.0f};
};

::filament::Box makeFilamentBounds(const float3& min, const float3& max)
{
  ::filament::Box box;
  box.set(min, max);
  return box;
}

template <typename T, std::size_t Size>
::filament::backend::BufferDescriptor makeBufferDescriptor(
    const std::array<T, Size>& data)
{
  auto* owned = new std::array<T, Size>(data);
  return ::filament::backend::BufferDescriptor(
      owned->data(),
      owned->size() * sizeof(T),
      [](void*, std::size_t, void* user) {
        delete static_cast<std::array<T, Size>*>(user);
      },
      owned);
}

template <typename T>
::filament::backend::BufferDescriptor makeBufferDescriptor(
    std::vector<T>&& data)
{
  auto* owned = new std::vector<T>(std::move(data));
  return ::filament::backend::BufferDescriptor(
      owned->data(),
      owned->size() * sizeof(T),
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<T>*>(user);
      },
      owned);
}

float3 toFloat3(const Eigen::Vector3d& vector)
{
  return {
      static_cast<float>(vector.x()),
      static_cast<float>(vector.y()),
      static_cast<float>(vector.z())};
}

float3 toFloat3f(const Eigen::Vector3f& vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

float2 toFloat2f(const Eigen::Vector2f& vector)
{
  return {vector.x(), vector.y()};
}

float3 normalizeOr(const float3& vector, const float3& fallback)
{
  const float lengthSquared
      = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  if (lengthSquared <= 1e-12f) {
    return fallback;
  }

  const float inverseLength = 1.0f / std::sqrt(lengthSquared);
  return {
      vector.x * inverseLength,
      vector.y * inverseLength,
      vector.z * inverseLength};
}

float3 crossProduct(const float3& lhs, const float3& rhs)
{
  return {
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x};
}

bool hasUsableTextureCoordinates(const std::vector<Vertex>& vertices)
{
  return std::any_of(
      vertices.begin(), vertices.end(), [](const Vertex& vertex) {
        return std::abs(vertex.uv.x) > 1e-7f || std::abs(vertex.uv.y) > 1e-7f;
      });
}

void generateTangentFrames(
    std::vector<Vertex>& vertices,
    const std::vector<::filament::math::uint3>& triangles,
    const std::vector<float3>& normals = {})
{
  std::vector<::filament::math::float2> uvs;
  std::vector<float3> positions;
  // SDK 1.77.3's flat-normal generator reads tightly packed positions even
  // when a stride is supplied. Its UV tangent generator also requires them.
  positions.reserve(vertices.size());
  std::transform(
      vertices.begin(),
      vertices.end(),
      std::back_inserter(positions),
      [](const Vertex& vertex) { return vertex.position; });
  if (hasUsableTextureCoordinates(vertices)) {
    uvs.reserve(vertices.size());
    std::transform(
        vertices.begin(),
        vertices.end(),
        std::back_inserter(uvs),
        [](const Vertex& vertex) { return vertex.uv; });
  }

  ::filament::geometry::SurfaceOrientation::Builder builder;
  builder.vertexCount(vertices.size())
      .positions(positions.data())
      .triangleCount(triangles.size())
      .triangles(triangles.data());
  if (normals.size() == vertices.size()) {
    builder.normals(normals.data());
  }
  if (!uvs.empty()) {
    builder.uvs(uvs.data());
  }

  std::unique_ptr<::filament::geometry::SurfaceOrientation> orientation(
      builder.build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    throw std::runtime_error("Failed to generate Filament tangent frames");
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));
}

float resolveMeshMaterialAlpha(
    float visualAlpha, float materialAlpha, MeshAlphaMode alphaMode)
{
  switch (alphaMode) {
    case MeshAlphaMode::Blend:
      return visualAlpha * materialAlpha;
    case MeshAlphaMode::Auto:
      return materialAlpha;
    case MeshAlphaMode::ShapeAlpha:
      return visualAlpha;
  }

  return visualAlpha * materialAlpha;
}

Renderable createBoxRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const float3& halfExtents,
    const float4& color)
{
  static constexpr std::array<std::uint16_t, 36> indices = {
      0,  1,  2,  0,  2,  3,  4,  6,  5,  4,  7,  6,  8,  10, 9,  8,  11, 10,
      12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 22, 21, 20, 23, 22};
  static constexpr std::array<::filament::math::ushort3, 12> triangles = {{
      {0, 1, 2},
      {0, 2, 3},
      {4, 6, 5},
      {4, 7, 6},
      {8, 10, 9},
      {8, 11, 10},
      {12, 13, 14},
      {12, 14, 15},
      {16, 17, 18},
      {16, 18, 19},
      {20, 22, 21},
      {20, 23, 22},
  }};

  const auto hx = halfExtents.x;
  const auto hy = halfExtents.y;
  const auto hz = halfExtents.z;
  const ::filament::math::short4 tangent = {0, 0, 0, 32767};
  std::array<Vertex, 24> vertices = {{
      {{-hx, -hy, hz}, tangent},  {{hx, -hy, hz}, tangent},
      {{hx, hy, hz}, tangent},    {{-hx, hy, hz}, tangent},
      {{-hx, -hy, -hz}, tangent}, {{hx, -hy, -hz}, tangent},
      {{hx, hy, -hz}, tangent},   {{-hx, hy, -hz}, tangent},
      {{-hx, hy, -hz}, tangent},  {{hx, hy, -hz}, tangent},
      {{hx, hy, hz}, tangent},    {{-hx, hy, hz}, tangent},
      {{-hx, -hy, -hz}, tangent}, {{hx, -hy, -hz}, tangent},
      {{hx, -hy, hz}, tangent},   {{-hx, -hy, hz}, tangent},
      {{hx, -hy, -hz}, tangent},  {{hx, hy, -hz}, tangent},
      {{hx, hy, hz}, tangent},    {{hx, -hy, hz}, tangent},
      {{-hx, -hy, -hz}, tangent}, {{-hx, hy, -hz}, tangent},
      {{-hx, hy, hz}, tangent},   {{-hx, -hy, hz}, tangent},
  }};

  std::array<float3, 24> positions;
  std::transform(
      vertices.begin(),
      vertices.end(),
      positions.begin(),
      [](const Vertex& vertex) { return vertex.position; });

  std::unique_ptr<::filament::geometry::SurfaceOrientation> orientation(
      ::filament::geometry::SurfaceOrientation::Builder()
          .vertexCount(vertices.size())
          .positions(positions.data())
          .triangleCount(triangles.size())
          .triangles(triangles.data())
          .build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    throw std::runtime_error("Failed to generate Filament tangent frames");
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));

  Renderable renderable;
  renderable.vertexBuffer
      = ::filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                ::filament::VertexAttribute::POSITION,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(Vertex, position),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::TANGENTS,
                0,
                ::filament::VertexBuffer::AttributeType::SHORT4,
                offsetof(Vertex, tangent),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::UV0,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(Vertex, uv),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::COLOR,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT4,
                offsetof(Vertex, color),
                sizeof(Vertex))
            .normalized(::filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(vertices));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::USHORT)
            .build(engine);
  renderable.cpuIndices.assign(indices.begin(), indices.end());
  renderable.indexBuffer->setBuffer(engine, makeBufferDescriptor(indices));

  auto* materialInstance
      = addRenderableMaterial(renderable, material, color, true);
  configureLitMaterialInstance(
      *materialInstance, color, 0.0f, 0.55f, float3{0.0f, 0.0f, 0.0f});

  renderable.entity = EntityManager::get().create();
  ::filament::RenderableManager::Builder(1)
      .boundingBox(makeFilamentBounds(
          {-halfExtents.x, -halfExtents.y, -halfExtents.z},
          {halfExtents.x, halfExtents.y, halfExtents.z}))
      .material(0, materialInstance)
      .geometry(
          0,
          ::filament::RenderableManager::PrimitiveType::TRIANGLES,
          renderable.vertexBuffer,
          renderable.indexBuffer,
          recordGeometryRange(renderable, 0, 0, indices.size(), true),
          indices.size())
      .castShadows(true)
      .receiveShadows(true)
      .build(engine, renderable.entity);
  auto& renderables = engine.getRenderableManager();
  renderables.setScreenSpaceContactShadows(
      renderables.getInstance(renderable.entity), true);

  return renderable;
}

void appendTriangle(
    std::vector<std::uint32_t>& indices,
    std::vector<::filament::math::uint3>& triangles,
    std::uint32_t a,
    std::uint32_t b,
    std::uint32_t c)
{
  indices.push_back(a);
  indices.push_back(b);
  indices.push_back(c);
  triangles.push_back({a, b, c});
}

struct Bounds
{
  float3 min;
  float3 max;
};

Bounds computeBounds(const std::vector<Vertex>& vertices)
{
  Bounds bounds{vertices.front().position, vertices.front().position};
  for (const Vertex& vertex : vertices) {
    bounds.min.x = std::min(bounds.min.x, vertex.position.x);
    bounds.min.y = std::min(bounds.min.y, vertex.position.y);
    bounds.min.z = std::min(bounds.min.z, vertex.position.z);
    bounds.max.x = std::max(bounds.max.x, vertex.position.x);
    bounds.max.y = std::max(bounds.max.y, vertex.position.y);
    bounds.max.z = std::max(bounds.max.z, vertex.position.z);
  }
  return bounds;
}

Bounds resolveDescriptorBounds(
    const GeometryDescriptor& geometry, const std::vector<Vertex>& vertices)
{
  if (geometry.hasLocalBounds && geometry.localBoundsMin.allFinite()
      && geometry.localBoundsMax.allFinite()
      && (geometry.localBoundsMin.array() <= geometry.localBoundsMax.array())
             .all()) {
    return Bounds{
        toFloat3(geometry.localBoundsMin), toFloat3(geometry.localBoundsMax)};
  }

  return computeBounds(vertices);
}

Bounds computeDebugBounds(const std::vector<DebugVertex>& vertices)
{
  Bounds bounds{vertices.front().position, vertices.front().position};
  for (const DebugVertex& vertex : vertices) {
    bounds.min.x = std::min(bounds.min.x, vertex.position.x);
    bounds.min.y = std::min(bounds.min.y, vertex.position.y);
    bounds.min.z = std::min(bounds.min.z, vertex.position.z);
    bounds.max.x = std::max(bounds.max.x, vertex.position.x);
    bounds.max.y = std::max(bounds.max.y, vertex.position.y);
    bounds.max.z = std::max(bounds.max.z, vertex.position.z);
  }
  return bounds;
}

struct TriangleMeshBuffers
{
  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<::filament::math::uint3> triangles;
  std::vector<float3> normals;
  Bounds bounds{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
};

TriangleMeshBuffers makeTriangleMeshBuffers(MeshGeometry&& geometry)
{
  const ::filament::math::short4 tangent = {0, 0, 0, 32767};
  TriangleMeshBuffers buffers;
  buffers.vertices.reserve(geometry.vertices.size());
  buffers.normals.reserve(geometry.vertices.size());
  for (const auto& vertex : geometry.vertices) {
    buffers.vertices.push_back(
        Vertex{toFloat3f(vertex.position), tangent, toFloat2f(vertex.uv)});
    buffers.normals.push_back(toFloat3f(vertex.normal));
  }

  buffers.indices = std::move(geometry.indices);
  buffers.triangles.reserve(geometry.triangles.size());
  for (const auto& triangle : geometry.triangles) {
    buffers.triangles.push_back({triangle.a, triangle.b, triangle.c});
  }

  buffers.bounds
      = {toFloat3f(geometry.boundsMin), toFloat3f(geometry.boundsMax)};
  return buffers;
}

::filament::math::half4 toLinearHalfColor(const Eigen::Vector4d& rgba)
{
  // Preserve the normalized color range of the former byte-packed stream.
  return toLinearColor(rgba.cwiseMax(0.0).cwiseMin(1.0).eval());
}

bool isDrawableDebugLine(const DebugLineDescriptor& line)
{
  if (!line.from.allFinite() || !line.to.allFinite()) {
    return false;
  }

  const double length = (line.to - line.from).norm();
  return std::isfinite(length) && length >= kMinDebugLineLength;
}

bool appendThinDebugLine(
    std::vector<DebugVertex>& vertices,
    std::vector<std::uint32_t>& indices,
    const DebugLineDescriptor& line)
{
  if (!isDrawableDebugLine(line)) {
    return false;
  }

  const auto start = static_cast<std::uint32_t>(vertices.size());
  const auto color = toLinearHalfColor(line.rgba);
  vertices.push_back({toFloat3(line.from), color});
  vertices.push_back({toFloat3(line.to), color});
  indices.push_back(start);
  indices.push_back(start + 1u);
  return true;
}

bool appendThickDebugLine(
    std::vector<DebugVertex>& vertices,
    std::vector<std::uint32_t>& triangleIndices,
    const DebugLineDescriptor& line)
{
  if (line.thickness <= 0.0 || !std::isfinite(line.thickness)
      || !isDrawableDebugLine(line)) {
    return false;
  }

  const Eigen::Vector3d segment = line.to - line.from;
  const double length = segment.norm();

  const Eigen::Vector3d direction = segment / length;
  const Eigen::Vector3d seed = std::abs(direction.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitY();
  Eigen::Vector3d first = direction.cross(seed);
  const double firstNorm = first.norm();
  if (!std::isfinite(firstNorm) || firstNorm < 1e-12) {
    return false;
  }
  first /= firstNorm;
  const Eigen::Vector3d second = direction.cross(first).normalized();
  if (!second.allFinite()) {
    return false;
  }

  const double radius = std::min(line.thickness * 0.5, length * 0.2);
  if (radius <= 0.0 || !std::isfinite(radius)) {
    return false;
  }

  const auto start = static_cast<std::uint32_t>(vertices.size());
  const auto color = toLinearHalfColor(line.rgba);
  for (int end = 0; end < 2; ++end) {
    const Eigen::Vector3d center = end == 0 ? line.from : line.to;
    for (int segmentIndex = 0; segmentIndex < kDebugLineTubeSegments;
         ++segmentIndex) {
      const double angle = kTwoPi * static_cast<double>(segmentIndex)
                           / static_cast<double>(kDebugLineTubeSegments);
      const Eigen::Vector3d offset
          = (first * std::cos(angle) + second * std::sin(angle)) * radius;
      vertices.push_back({toFloat3(center + offset), color});
    }
  }

  for (int segmentIndex = 0; segmentIndex < kDebugLineTubeSegments;
       ++segmentIndex) {
    const auto a = static_cast<std::uint32_t>(start + segmentIndex);
    const auto b = static_cast<std::uint32_t>(
        start + ((segmentIndex + 1) % kDebugLineTubeSegments));
    const auto c = static_cast<std::uint32_t>(
        start + kDebugLineTubeSegments + segmentIndex);
    const auto d = static_cast<std::uint32_t>(
        start + kDebugLineTubeSegments
        + ((segmentIndex + 1) % kDebugLineTubeSegments));
    triangleIndices.push_back(a);
    triangleIndices.push_back(c);
    triangleIndices.push_back(b);
    triangleIndices.push_back(b);
    triangleIndices.push_back(c);
    triangleIndices.push_back(d);
  }

  const auto fromCenter = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back({toFloat3(line.from), color});
  const auto toCenter = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back({toFloat3(line.to), color});
  for (int segmentIndex = 0; segmentIndex < kDebugLineTubeSegments;
       ++segmentIndex) {
    const auto a = static_cast<std::uint32_t>(start + segmentIndex);
    const auto b = static_cast<std::uint32_t>(
        start + ((segmentIndex + 1) % kDebugLineTubeSegments));
    const auto c = static_cast<std::uint32_t>(
        start + kDebugLineTubeSegments + segmentIndex);
    const auto d = static_cast<std::uint32_t>(
        start + kDebugLineTubeSegments
        + ((segmentIndex + 1) % kDebugLineTubeSegments));
    triangleIndices.push_back(fromCenter);
    triangleIndices.push_back(b);
    triangleIndices.push_back(a);
    triangleIndices.push_back(toCenter);
    triangleIndices.push_back(c);
    triangleIndices.push_back(d);
  }

  return true;
}

bool appendDebugTriangle(
    std::vector<DebugVertex>& vertices,
    std::vector<std::uint32_t>& triangleIndices,
    const DebugTriangleDescriptor& triangle)
{
  if (!triangle.a.allFinite() || !triangle.b.allFinite()
      || !triangle.c.allFinite()) {
    return false;
  }
  const Eigen::Vector3d normal
      = (triangle.b - triangle.a).cross(triangle.c - triangle.a);
  const double area = normal.norm() * 0.5;
  if (!std::isfinite(area) || area <= kMinDebugTriangleArea) {
    return false;
  }

  const auto start = static_cast<std::uint32_t>(vertices.size());
  const auto color = toLinearHalfColor(triangle.rgba);
  vertices.push_back({toFloat3(triangle.a), color});
  vertices.push_back({toFloat3(triangle.b), color});
  vertices.push_back({toFloat3(triangle.c), color});
  triangleIndices.push_back(start);
  triangleIndices.push_back(start + 1u);
  triangleIndices.push_back(start + 2u);
  return true;
}

} // namespace

std::optional<Renderable> createDebugLineRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const std::vector<DebugLineDescriptor>& lines)
{
  return createDebugLineRenderable(engine, material, lines, {});
}

std::optional<Renderable> createDebugLineRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const std::vector<DebugLineDescriptor>& lines,
    const std::vector<DebugTriangleDescriptor>& triangles)
{
  if (lines.empty() && triangles.empty()) {
    return std::nullopt;
  }

  std::vector<DebugVertex> vertices;
  std::vector<std::uint32_t> lineIndices;
  std::vector<std::uint32_t> triangleIndices;
  vertices.reserve(lines.size() * 2u + triangles.size() * 3u);
  lineIndices.reserve(lines.size() * 2u);
  triangleIndices.reserve(triangles.size() * 3u);

  for (const DebugLineDescriptor& line : lines) {
    if (line.thickness > 0.0
        && appendThickDebugLine(vertices, triangleIndices, line)) {
      continue;
    }
    appendThinDebugLine(vertices, lineIndices, line);
  }
  for (const DebugTriangleDescriptor& triangle : triangles) {
    appendDebugTriangle(vertices, triangleIndices, triangle);
  }
  if (vertices.empty()) {
    return std::nullopt;
  }

  const Bounds bounds = computeDebugBounds(vertices);
  std::vector<std::uint32_t> indices;
  indices.reserve(lineIndices.size() + triangleIndices.size());
  indices.insert(indices.end(), lineIndices.begin(), lineIndices.end());
  const std::size_t triangleIndexOffset = indices.size();
  indices.insert(indices.end(), triangleIndices.begin(), triangleIndices.end());
  Renderable renderable;
  renderable.vertexBuffer
      = ::filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                ::filament::VertexAttribute::POSITION,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(DebugVertex, position),
                sizeof(DebugVertex))
            .attribute(
                ::filament::VertexAttribute::COLOR,
                0,
                ::filament::VertexBuffer::AttributeType::HALF4,
                offsetof(DebugVertex, color),
                sizeof(DebugVertex))
            .attribute(
                ::filament::VertexAttribute::UV0,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(DebugVertex, uv),
                sizeof(DebugVertex))
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.cpuIndices = indices;
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  auto* materialInstance = addRenderableMaterial(renderable, material);
  renderable.entity = EntityManager::get().create();
  const bool hasLines = !lineIndices.empty();
  const bool hasTriangles = !triangleIndices.empty();
  const std::uint8_t primitiveCount
      = static_cast<std::uint8_t>((hasLines ? 1 : 0) + (hasTriangles ? 1 : 0));
  ::filament::RenderableManager::Builder builder(primitiveCount);
  builder.boundingBox(makeFilamentBounds(bounds.min, bounds.max))
      .castShadows(false)
      .receiveShadows(false);

  std::uint8_t primitiveIndex = 0;
  if (hasLines) {
    builder.material(primitiveIndex, materialInstance)
        .geometry(
            primitiveIndex,
            ::filament::RenderableManager::PrimitiveType::LINES,
            renderable.vertexBuffer,
            renderable.indexBuffer,
            recordGeometryRange(
                renderable, primitiveIndex, 0, lineIndices.size(), false),
            lineIndices.size());
    ++primitiveIndex;
  }
  if (hasTriangles) {
    builder.material(primitiveIndex, materialInstance)
        .geometry(
            primitiveIndex,
            ::filament::RenderableManager::PrimitiveType::TRIANGLES,
            renderable.vertexBuffer,
            renderable.indexBuffer,
            recordGeometryRange(
                renderable,
                primitiveIndex,
                triangleIndexOffset,
                triangleIndices.size(),
                true),
            triangleIndices.size());
  }
  builder.build(engine, renderable.entity);

  return renderable;
}

namespace {

std::optional<Renderable> createLineSegmentRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const RenderableDescriptor& descriptor)
{
  return createDebugLineRenderable(
      engine,
      material,
      {},
      dart::gui::makeLineSegmentVisualTriangles(
          descriptor, {3, 3, 2}, {0, 0, 0}, 45, 720));
}

Renderable createTriangleMeshRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    std::vector<Vertex> vertices,
    std::vector<std::uint32_t> indices,
    std::vector<::filament::math::uint3> triangles,
    std::vector<float3> normals,
    const float4& color,
    const float3& minBounds,
    const float3& maxBounds,
    const PbrTextureBindings& textures = {},
    const TextureBinding* fallbackTexture = nullptr,
    float metallic = kDefaultDielectricMetallic,
    float roughness = kDefaultMatteRoughness,
    float3 emissiveColor = {0.0f, 0.0f, 0.0f},
    bool followsDescriptorColor = true,
    bool doubleSided = true)
{
  const std::size_t indexCount = indices.size();
  generateTangentFrames(vertices, triangles, normals);

  Renderable renderable;
  renderable.vertexBuffer
      = ::filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                ::filament::VertexAttribute::POSITION,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(Vertex, position),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::TANGENTS,
                0,
                ::filament::VertexBuffer::AttributeType::SHORT4,
                offsetof(Vertex, tangent),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::UV0,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(Vertex, uv),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::COLOR,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT4,
                offsetof(Vertex, color),
                sizeof(Vertex))
            .normalized(::filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.cpuIndices = indices;
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  auto* materialInstance = addRenderableMaterial(
      renderable, material, color, followsDescriptorColor);
  configureLitMaterialInstance(
      *materialInstance,
      color,
      metallic,
      roughness,
      emissiveColor,
      textures,
      fallbackTexture);
  materialInstance->setDoubleSided(doubleSided);
  if (!doubleSided) {
    materialInstance->setCullingMode(::filament::backend::CullingMode::BACK);
  }

  renderable.entity = EntityManager::get().create();
  ::filament::RenderableManager::Builder(1)
      .boundingBox(makeFilamentBounds(minBounds, maxBounds))
      .material(0, materialInstance)
      .geometry(
          0,
          ::filament::RenderableManager::PrimitiveType::TRIANGLES,
          renderable.vertexBuffer,
          renderable.indexBuffer,
          recordGeometryRange(renderable, 0, 0, indexCount, true),
          indexCount)
      .castShadows(true)
      .receiveShadows(true)
      .build(engine, renderable.entity);
  auto& renderables = engine.getRenderableManager();
  renderables.setScreenSpaceContactShadows(
      renderables.getInstance(renderable.entity), true);

  return renderable;
}

Renderable createGeometryMeshRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    MeshGeometry geometry,
    const float4& color)
{
  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(buffers.vertices),
      std::move(buffers.indices),
      std::move(buffers.triangles),
      std::move(buffers.normals),
      color,
      buffers.bounds.min,
      buffers.bounds.max);
}

Renderable createEllipsoidRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const Eigen::Vector3d& radii,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeEllipsoidMeshGeometry(radii), color);
}

Renderable createMultiSphereRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const std::vector<Eigen::Vector3d>& centers,
    const std::vector<double>& radii,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeMultiSphereMeshGeometry(centers, radii), color);
}

Renderable createCylinderRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeCylinderMeshGeometry(radius, height), color);
}

Renderable createConeRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeConeMeshGeometry(radius, height), color);
}

Renderable createPyramidRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const Eigen::Vector3d& size,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makePyramidMeshGeometry(size), color);
}

Renderable createCapsuleRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeCapsuleMeshGeometry(radius, height), color);
}

std::optional<Renderable> createDescriptorTriangleMeshRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const GeometryDescriptor& geometry,
    const float4& color)
{
  if (geometry.triangleVertices.empty() || geometry.triangleIndices.empty()) {
    return std::nullopt;
  }
  if (geometry.triangleVertices.size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const ::filament::math::short4 tangent = {0, 0, 0, 32767};
  std::vector<Vertex> vertices;
  vertices.reserve(geometry.triangleVertices.size());
  for (const Eigen::Vector3d& vertex : geometry.triangleVertices) {
    if (!vertex.allFinite()) {
      return std::nullopt;
    }
    vertices.push_back(Vertex{toFloat3(vertex), tangent});
  }

  std::vector<std::uint32_t> indices;
  std::vector<::filament::math::uint3> triangles;
  indices.reserve(geometry.triangleIndices.size() * 3u);
  triangles.reserve(geometry.triangleIndices.size());
  for (const Eigen::Vector3i& triangle : geometry.triangleIndices) {
    if ((triangle.array() < 0).any()) {
      return std::nullopt;
    }
    const auto first = static_cast<std::size_t>(triangle.x());
    const auto second = static_cast<std::size_t>(triangle.y());
    const auto third = static_cast<std::size_t>(triangle.z());
    if (first >= vertices.size() || second >= vertices.size()
        || third >= vertices.size()) {
      return std::nullopt;
    }
    appendTriangle(
        indices,
        triangles,
        static_cast<std::uint32_t>(first),
        static_cast<std::uint32_t>(second),
        static_cast<std::uint32_t>(third));
  }
  if (indices.empty()) {
    return std::nullopt;
  }

  std::vector<float3> normals;
  if (geometry.triangleNormals.size() == vertices.size()) {
    normals.reserve(vertices.size());
    for (const Eigen::Vector3d& sourceNormal : geometry.triangleNormals) {
      if (!sourceNormal.allFinite() || sourceNormal.squaredNorm() <= 1e-12) {
        normals.clear();
        break;
      }
      normals.push_back(toFloat3(sourceNormal.normalized()));
    }
  }

  if (normals.size() != vertices.size()) {
    std::vector<float3> normalSums(vertices.size(), {0.0f, 0.0f, 0.0f});
    for (const auto& triangle : triangles) {
      const float3 normal = normalizeOr(
          crossProduct(
              vertices[triangle.y].position - vertices[triangle.x].position,
              vertices[triangle.z].position - vertices[triangle.x].position),
          {0.0f, 0.0f, 1.0f});
      for (std::uint32_t index : {triangle.x, triangle.y, triangle.z}) {
        normalSums[index].x += normal.x;
        normalSums[index].y += normal.y;
        normalSums[index].z += normal.z;
      }
    }

    normals.clear();
    normals.reserve(normalSums.size());
    for (const float3& normal : normalSums) {
      normals.push_back(normalizeOr(normal, {0.0f, 0.0f, 1.0f}));
    }
  }

  const Bounds bounds = resolveDescriptorBounds(geometry, vertices);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      bounds.min,
      bounds.max);
}

std::optional<Renderable> createPointCloudRenderable(
    ::filament::Engine& engine,
    const MaterialSet& materials,
    const RenderableDescriptor& descriptor)
{
  if (descriptor.geometry.pointCloudPoints.empty()) {
    return std::nullopt;
  }

  if (descriptor.geometry.pointShapeType != dart::gui::PointShapeType::Box)
    return createDebugLineRenderable(
        engine,
        materials.pointColor,
        {},
        dart::gui::makePointCloudVisualTriangles(
            descriptor, {3, 3, 2}, {0, 0, 0}, {0, 0, 1}, 45, 720));

  float4 color = toLinearColor(descriptor.material.rgba);
  if (descriptor.geometry.pointCloudColors.size() == 1u) {
    color = toLinearColor(descriptor.geometry.pointCloudColors.front());
  }

  const double pointSize = std::max(descriptor.geometry.pointSize, 1e-4);
  MeshGeometry geometry;
  std::vector<MeshIndexRange> pointRanges;
  geometry.vertices.reserve(descriptor.geometry.pointCloudPoints.size() * 24u);
  geometry.indices.reserve(descriptor.geometry.pointCloudPoints.size() * 36u);
  geometry.triangles.reserve(descriptor.geometry.pointCloudPoints.size() * 12u);
  pointRanges.reserve(descriptor.geometry.pointCloudPoints.size());

  for (const Eigen::Vector3d& point : descriptor.geometry.pointCloudPoints) {
    pointRanges.push_back(appendBoxMeshGeometry(
        geometry, point, Eigen::Vector3d::Constant(pointSize)));
  }

  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  const Bounds bounds = buffers.bounds;
  const bool usePerPointColors = descriptor.geometry.pointCloudColors.size()
                                 == descriptor.geometry.pointCloudPoints.size();
  if (usePerPointColors) {
    generateTangentFrames(buffers.vertices, buffers.triangles, buffers.normals);

    Renderable renderable;
    renderable.vertexBuffer
        = ::filament::VertexBuffer::Builder()
              .vertexCount(buffers.vertices.size())
              .bufferCount(1)
              .attribute(
                  ::filament::VertexAttribute::POSITION,
                  0,
                  ::filament::VertexBuffer::AttributeType::FLOAT3,
                  offsetof(Vertex, position),
                  sizeof(Vertex))
              .attribute(
                  ::filament::VertexAttribute::TANGENTS,
                  0,
                  ::filament::VertexBuffer::AttributeType::SHORT4,
                  offsetof(Vertex, tangent),
                  sizeof(Vertex))
              .attribute(
                  ::filament::VertexAttribute::UV0,
                  0,
                  ::filament::VertexBuffer::AttributeType::FLOAT2,
                  offsetof(Vertex, uv),
                  sizeof(Vertex))
              .attribute(
                  ::filament::VertexAttribute::COLOR,
                  0,
                  ::filament::VertexBuffer::AttributeType::FLOAT4,
                  offsetof(Vertex, color),
                  sizeof(Vertex))
              .normalized(::filament::VertexAttribute::TANGENTS)
              .build(engine);
    renderable.vertexBuffer->setBufferAt(
        engine, 0, makeBufferDescriptor(std::move(buffers.vertices)));

    renderable.indexBuffer
        = ::filament::IndexBuffer::Builder()
              .indexCount(buffers.indices.size())
              .bufferType(::filament::IndexBuffer::IndexType::UINT)
              .build(engine);
    renderable.cpuIndices = buffers.indices;
    renderable.indexBuffer->setBuffer(
        engine, makeBufferDescriptor(std::move(buffers.indices)));

    renderable.entity = EntityManager::get().create();
    auto builder = ::filament::RenderableManager::Builder(pointRanges.size());
    builder.boundingBox(makeFilamentBounds(bounds.min, bounds.max))
        .castShadows(true)
        .receiveShadows(true);
    for (std::size_t i = 0; i < pointRanges.size(); ++i) {
      const float4 pointColor
          = toLinearColor(descriptor.geometry.pointCloudColors[i]);
      auto* materialInstance = addRenderableMaterial(
          renderable, materials.transparentLit, pointColor, false);
      configureLitMaterialInstance(
          *materialInstance, pointColor, 0.0f, 0.58f, {0.0f, 0.0f, 0.0f});
      builder.material(i, materialInstance)
          .geometry(
              i,
              ::filament::RenderableManager::PrimitiveType::TRIANGLES,
              renderable.vertexBuffer,
              renderable.indexBuffer,
              recordGeometryRange(
                  renderable,
                  i,
                  pointRanges[i].indexOffset,
                  pointRanges[i].indexCount,
                  true),
              pointRanges[i].indexCount);
    }
    builder.build(engine, renderable.entity);

    auto& renderables = engine.getRenderableManager();
    renderables.setScreenSpaceContactShadows(
        renderables.getInstance(renderable.entity), true);
    return renderable;
  }

  return createTriangleMeshRenderable(
      engine,
      materials.transparentLit,
      std::move(buffers.vertices),
      std::move(buffers.indices),
      std::move(buffers.triangles),
      std::move(buffers.normals),
      color,
      bounds.min,
      bounds.max);
}

std::optional<Renderable> createVoxelGridRenderable(
    ::filament::Engine& engine,
    const MaterialSet& materials,
    const RenderableDescriptor& descriptor)
{
  if (descriptor.geometry.voxelCenters.empty()) {
    return std::nullopt;
  }

  const float4 color = toLinearColor(descriptor.material.rgba);
  const double voxelSize = std::max(descriptor.geometry.voxelSize, 1e-4);
  MeshGeometry geometry;
  geometry.vertices.reserve(descriptor.geometry.voxelCenters.size() * 24u);
  geometry.indices.reserve(descriptor.geometry.voxelCenters.size() * 36u);
  geometry.triangles.reserve(descriptor.geometry.voxelCenters.size() * 12u);

  for (std::size_t i = 0; i < descriptor.geometry.voxelCenters.size(); ++i) {
    const double size = descriptor.geometry.voxelSizes.size()
                                == descriptor.geometry.voxelCenters.size()
                            ? descriptor.geometry.voxelSizes[i]
                            : voxelSize;
    appendBoxMeshGeometry(
        geometry,
        descriptor.geometry.voxelCenters[i],
        Eigen::Vector3d::Constant(size));
  }

  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  const Bounds bounds = buffers.bounds;
  return createTriangleMeshRenderable(
      engine,
      selectLitMaterial(
          materials, false, !isOpaqueShapeAlpha(descriptor.material.rgba.w())),
      std::move(buffers.vertices),
      std::move(buffers.indices),
      std::move(buffers.triangles),
      std::move(buffers.normals),
      color,
      bounds.min,
      bounds.max,
      {},
      nullptr,
      0.0f,
      0.58f,
      {0.0f, 0.0f, 0.0f},
      true,
      false);
}

std::optional<Vertex> makeMeshVertex(
    const GeometryDescriptor& geometry,
    std::size_t source,
    const float4& color,
    bool vertexColors)
{
  if (source >= geometry.triangleVertices.size()
      || !geometry.triangleVertices[source].allFinite())
    return std::nullopt;
  float2 uv{0, 0};
  if (geometry.meshTextureCoordComponents >= 1
      && geometry.meshTextureCoordinates.size()
             == geometry.triangleVertices.size()) {
    const auto& coordinate = geometry.meshTextureCoordinates[source];
    if (!coordinate.allFinite())
      return std::nullopt;
    uv
        = {static_cast<float>(coordinate.x()),
           geometry.meshTextureCoordComponents >= 2
               ? static_cast<float>(coordinate.y())
               : 0.0f};
  }
  float4 rgba{1, 1, 1, 1};
  if (vertexColors
      && geometry.meshVertexColors.size() == geometry.triangleVertices.size()) {
    rgba = toLinearColor(geometry.meshVertexColors[source]);
    rgba.w = resolveMeshMaterialAlpha(color.w, rgba.w, geometry.meshAlphaMode);
  }
  return Vertex{
      toFloat3(geometry.triangleVertices[source]), {0, 0, 0, 32767}, uv, rgba};
}

std::vector<Vertex> makeMeshPixelVertices(
    const RenderableDescriptor& descriptor,
    const MeshPartDescriptor& part,
    const MeshGeometry& mesh)
{
  const auto& geometry = descriptor.geometry;
  const auto color = toLinearColor(descriptor.material.rgba);
  float4 baseColor = color;
  if (part.usesVertexColors)
    baseColor = {1, 1, 1, 1};
  else if (
      geometry.meshUsesMaterialColors
      && part.materialIndex < geometry.meshMaterials.size()) {
    baseColor
        = toLinearColor(geometry.meshMaterials[part.materialIndex].diffuse);
    baseColor.w = resolveMeshMaterialAlpha(
        color.w, baseColor.w, geometry.meshAlphaMode);
  }
  std::vector<Vertex> vertices;
  vertices.reserve(mesh.vertices.size());
  for (const auto& corner : mesh.vertices) {
    const auto source = makeMeshVertex(
        geometry, corner.sourceVertex, color, part.usesVertexColors);
    if (!source)
      return {};
    auto vertex = *source;
    vertex.position = toFloat3f(corner.position);
    vertex.color *= baseColor;
    vertices.push_back(vertex);
  }
  return vertices;
}

std::optional<Renderable> createMeshRenderable(
    ::filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const RenderableDescriptor& descriptor,
    const float4& color)
{
  const auto& geometry = descriptor.geometry;
  const auto& meshVertices = geometry.triangleVertices;
  const auto& meshTriangles = geometry.triangleIndices;
  const bool meshPixels
      = !geometry.meshPointIndices.empty() || !geometry.meshLineIndices.empty();
  if (meshVertices.empty() || (meshTriangles.empty() && !meshPixels)) {
    return std::nullopt;
  }
  if (meshVertices.size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  std::vector<bool> coloredVertices(
      meshVertices.size(), geometry.meshUsesVertexColors);
  for (const auto& part : geometry.meshParts) {
    const auto first = std::min(part.vertexOffset, meshVertices.size());
    const auto count = std::min(part.vertexCount, meshVertices.size() - first);
    std::fill(
        coloredVertices.begin() + first,
        coloredVertices.begin() + first + count,
        part.usesVertexColors);
  }
  std::vector<Vertex> vertices;
  vertices.reserve(meshVertices.size());
  for (std::size_t i = 0; i < meshVertices.size(); ++i) {
    const auto vertex = makeMeshVertex(geometry, i, color, coloredVertices[i]);
    if (!vertex)
      return std::nullopt;
    vertices.push_back(*vertex);
  }

  std::vector<float3> normals;
  if (geometry.triangleNormals.size() == meshVertices.size()) {
    normals.reserve(meshVertices.size());
    for (const Eigen::Vector3d& sourceNormal : geometry.triangleNormals) {
      if (!sourceNormal.allFinite() || sourceNormal.squaredNorm() <= 1e-12) {
        normals.clear();
        break;
      }
      normals.push_back(toFloat3(sourceNormal.normalized()));
    }
  }

  std::vector<std::uint32_t> indices;
  std::vector<::filament::math::uint3> triangles;
  indices.reserve(meshTriangles.size() * 3);
  triangles.reserve(meshTriangles.size());
  for (const Eigen::Vector3i& triangle : meshTriangles) {
    if ((triangle.array() < 0).any()) {
      return std::nullopt;
    }

    const auto first = static_cast<std::size_t>(triangle.x());
    const auto second = static_cast<std::size_t>(triangle.y());
    const auto third = static_cast<std::size_t>(triangle.z());
    if (first >= vertices.size() || second >= vertices.size()
        || third >= vertices.size()) {
      return std::nullopt;
    }
    appendTriangle(
        indices,
        triangles,
        static_cast<std::uint32_t>(first),
        static_cast<std::uint32_t>(second),
        static_cast<std::uint32_t>(third));
  }

  struct MeshMaterialState
  {
    float4 baseColor;
    PbrTextureBindings textures;
    float metallic = kDefaultDielectricMetallic;
    float roughness = kDefaultMatteRoughness;
    float3 emissiveColor{0.0f, 0.0f, 0.0f};
    bool followsDescriptorColor = true;
    bool transparent = false;
    const MeshMaterialDescriptor* legacy = nullptr;
  };

  const auto makeMaterialState = [&](unsigned int materialIndex,
                                     bool vertexColors) {
    MeshMaterialState state;
    state.baseColor = color;
    const bool shapeOpaque
        = std::abs(static_cast<float>(descriptor.material.rgba.w()))
          > 1.0f - 1e-6f;
    state.transparent = vertexColors || !shapeOpaque;
    if (vertexColors) {
      state.baseColor = {1, 1, 1, 1};
      state.followsDescriptorColor = false;
    }

    if (materialIndex < geometry.meshMaterials.size()) {
      const MeshMaterialDescriptor& meshMaterial
          = geometry.meshMaterials[materialIndex];
      if (geometry.meshUsesMaterialColors) {
        // OSG uses all four original Phong alphas to select its render bin.
        const auto coefficients
            = {&meshMaterial.ambient,
               &meshMaterial.diffuse,
               &meshMaterial.specular,
               &meshMaterial.emissive};
        const bool materialTransparent = std::any_of(
            coefficients.begin(),
            coefficients.end(),
            [](const auto* coefficient) {
              return std::abs(static_cast<float>(coefficient->w()))
                     < 1.0f - 1e-6f;
            });
        switch (geometry.meshAlphaMode) {
          case MeshAlphaMode::ShapeAlpha:
            state.transparent = vertexColors || !shapeOpaque;
            break;
          case MeshAlphaMode::Blend:
            state.transparent
                = vertexColors || !shapeOpaque || materialTransparent;
            break;
          case MeshAlphaMode::Auto:
            state.transparent = vertexColors || materialTransparent;
            break;
        }
        state.followsDescriptorColor = false;
        state.baseColor = toLinearColor(meshMaterial.diffuse);
        state.baseColor.w = resolveMeshMaterialAlpha(
            color.w, state.baseColor.w, geometry.meshAlphaMode);
        state.metallic = static_cast<float>(meshMaterial.metallicFactor);
        state.roughness = static_cast<float>(meshMaterial.roughnessFactor);
        if (!meshMaterial.hasPbrMetallic && !meshMaterial.hasPbrRoughness
            && meshMaterial.metallicTexturePath.empty()
            && meshMaterial.roughnessTexturePath.empty()
            && meshMaterial.metallicRoughnessTexturePath.empty())
          state.legacy = &meshMaterial;
        // Emission is a linear intensity, not an authored display color.
        state.emissiveColor = toFloat3(meshMaterial.emissive.head<3>().eval());
      }

      // Legacy OSG binds authored textures even without a UV array; the
      // vertex layout supplies constant (0, 0) coordinates in that case.
      {
        const auto loadBinding
            = [&](const std::string& source,
                  TextureColorSpace colorSpace) -> const TextureBinding* {
          if (source.empty()) {
            return nullptr;
          }
          return getOrLoadTextureBinding(
              engine, textureCache, source, colorSpace);
        };

        const std::string& baseColorTexturePath
            = !meshMaterial.baseColorTexturePath.empty()
                  ? meshMaterial.baseColorTexturePath
                  : (!meshMaterial.textureImagePaths.empty()
                         ? meshMaterial.textureImagePaths[0]
                         : meshMaterial.baseColorTexturePath);
        state.textures.baseColor
            = loadBinding(baseColorTexturePath, TextureColorSpace::Srgb);
        state.textures.metallic = loadBinding(
            meshMaterial.metallicTexturePath, TextureColorSpace::Linear);
        state.textures.roughness = loadBinding(
            meshMaterial.roughnessTexturePath, TextureColorSpace::Linear);
        state.textures.metallicRoughness = loadBinding(
            meshMaterial.metallicRoughnessTexturePath,
            TextureColorSpace::Linear);
        state.textures.normal = loadBinding(
            meshMaterial.normalTexturePath, TextureColorSpace::Linear);
        state.textures.occlusion = loadBinding(
            meshMaterial.occlusionTexturePath, TextureColorSpace::Linear);
        state.textures.emissive = loadBinding(
            meshMaterial.emissiveTexturePath, TextureColorSpace::Srgb);
      }
    }

    return state;
  };

  const auto makeMaterialInstance
      = [&](Renderable& renderable,
            const MeshMaterialState& state) -> ::filament::MaterialInstance* {
    const bool usesTextures = hasTextureBindings(state.textures);
    const bool transparent = state.transparent;
    auto& selectedMaterial
        = state.legacy
              ? (usesTextures
                     ? (transparent ? materials.legacyTransparentTexturedLit
                                    : materials.legacyTexturedLit)
                     : (transparent ? materials.legacyTransparentLit
                                    : materials.legacyLit))
              : selectLitMaterial(materials, usesTextures, transparent);
    auto* materialInstance = addRenderableMaterial(
        renderable,
        selectedMaterial,
        state.baseColor,
        state.followsDescriptorColor);
    configureLitMaterialInstance(
        *materialInstance,
        state.baseColor,
        state.metallic,
        state.roughness,
        state.emissiveColor,
        state.textures,
        usesTextures ? &materials.fallbackTexture : nullptr);
    for (const auto* binding :
         {state.textures.baseColor,
          state.textures.metallic,
          state.textures.roughness,
          state.textures.metallicRoughness,
          state.textures.normal,
          state.textures.occlusion,
          state.textures.emissive}) {
      if (binding && binding->texture
          && std::find(
                 renderable.sampledTextures.begin(),
                 renderable.sampledTextures.end(),
                 binding->texture)
                 == renderable.sampledTextures.end())
        renderable.sampledTextures.push_back(binding->texture);
    }
    if (state.legacy) {
      const auto& legacy = *state.legacy;
      const double exponent
          = std::max(0.0, legacy.shininess * legacy.shininessStrength);
      // Approximate the Phong lobe width, accounting for Filament's perceptual
      // roughness squared mapping. Preserve the authored RGB specular tint.
      const float glossiness
          = static_cast<float>(1.0 - std::pow(2.0 / (exponent + 2.0), .25));
      materialInstance->setParameter("glossiness", glossiness);
      // F0 reflectance and ambient coefficients already use linear values.
      materialInstance->setParameter(
          "specularColor",
          toFloat3(
              legacy.specular.head<3>().cwiseMax(0.0).cwiseMin(1.0).eval()));
      materialInstance->setParameter(
          "ambientColor",
          toFloat3(legacy.ambient.head<3>().cwiseMax(0.0).eval()));
    }
    return materialInstance;
  };

  const Bounds bounds = resolveDescriptorBounds(geometry, vertices);
  std::vector<MeshPartDescriptor> parts;
  parts.reserve(geometry.meshParts.size());
  std::size_t coveredTriangleCount = 0;
  bool useSubMeshes = !geometry.meshParts.empty();
  for (const MeshPartDescriptor& range : geometry.meshParts) {
    if (range.triangleCount == 0 && range.pointCount == 0
        && range.lineCount == 0) {
      continue;
    }
    if (range.triangleOffset > meshTriangles.size()
        || range.triangleCount > meshTriangles.size() - range.triangleOffset) {
      useSubMeshes = false;
      break;
    }
    coveredTriangleCount += range.triangleCount;
    parts.push_back(range);
  }
  useSubMeshes = useSubMeshes && !parts.empty()
                 && coveredTriangleCount == meshTriangles.size();
  if (!useSubMeshes) {
    MeshPartDescriptor part;
    part.vertexCount = meshVertices.size();
    part.triangleCount = meshTriangles.size();
    part.pointCount = geometry.meshPointIndices.size();
    part.lineCount = geometry.meshLineIndices.size();
    part.usesVertexColors = geometry.meshUsesVertexColors;
    parts = {part};
  }
  if (!triangles.empty())
    generateTangentFrames(vertices, triangles, normals);

  struct MeshPrimitive
  {
    MeshPartDescriptor part;
    MeshIndexRange range;
    bool pixels;
  };
  std::vector<MeshPrimitive> primitives;
  for (const auto& part : parts) {
    if (part.triangleCount)
      primitives.push_back(
          {part, {part.triangleOffset * 3, part.triangleCount * 3}, false});
    const auto mesh = makeMeshPointLineGeometry(
        descriptor, part, {3, 3, 2}, {0, 0, 0}, {0, 0, 1}, 45, 720);
    const auto pixelVertices = makeMeshPixelVertices(descriptor, part, mesh);
    if (pixelVertices.empty())
      continue;
    if (vertices.size() + pixelVertices.size()
        > std::numeric_limits<std::uint32_t>::max())
      return std::nullopt;
    const auto vertexOffset = static_cast<std::uint32_t>(vertices.size());
    const auto indexOffset = indices.size();
    vertices.insert(vertices.end(), pixelVertices.begin(), pixelVertices.end());
    for (std::uint32_t index : mesh.indices)
      indices.push_back(vertexOffset + index);
    primitives.push_back({part, {indexOffset, mesh.indices.size()}, true});
  }
  if (primitives.empty())
    return std::nullopt;
  const Bounds renderBounds = meshPixels ? computeBounds(vertices) : bounds;

  Renderable renderable;
  if (meshPixels) {
    renderable.meshScreenVertexOffset = meshVertices.size();
    renderable.meshScreenParts = parts;
  }
  renderable.vertexBuffer
      = ::filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                ::filament::VertexAttribute::POSITION,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(Vertex, position),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::TANGENTS,
                0,
                ::filament::VertexBuffer::AttributeType::SHORT4,
                offsetof(Vertex, tangent),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::UV0,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(Vertex, uv),
                sizeof(Vertex))
            .attribute(
                ::filament::VertexAttribute::COLOR,
                0,
                ::filament::VertexBuffer::AttributeType::FLOAT4,
                offsetof(Vertex, color),
                sizeof(Vertex))
            .normalized(::filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.cpuIndices = indices;
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  renderable.entity = EntityManager::get().create();
  // A material or texture failure must not leak the partial renderable, whose
  // instances Filament would otherwise keep committing every frame.
  try {
    auto builder = ::filament::RenderableManager::Builder(primitives.size());
    builder.boundingBox(makeFilamentBounds(renderBounds.min, renderBounds.max))
        .castShadows(true)
        .receiveShadows(true);
    for (std::size_t partIndex = 0; partIndex < primitives.size();
         ++partIndex) {
      const auto& primitive = primitives[partIndex];
      const auto& part = primitive.part;
      const auto state
          = makeMaterialState(part.materialIndex, part.usesVertexColors);
      auto* materialInstance
          = primitive.pixels ? addRenderableMaterial(
                renderable,
                state.transparent ? materials.pointColor
                                  : materials.opaquePointColor)
                             : makeMaterialInstance(renderable, state);
      if (primitive.pixels) {
        materialInstance->setDepthWrite(!state.transparent);
        setBaseColorTextureParameters(
            *materialInstance,
            materials.fallbackTexture,
            state.textures.baseColor);
        if (state.textures.baseColor && state.textures.baseColor->texture
            && std::find(
                   renderable.sampledTextures.begin(),
                   renderable.sampledTextures.end(),
                   state.textures.baseColor->texture)
                   == renderable.sampledTextures.end())
          renderable.sampledTextures.push_back(
              state.textures.baseColor->texture);
      }
      builder.material(partIndex, materialInstance)
          .geometry(
              partIndex,
              ::filament::RenderableManager::PrimitiveType::TRIANGLES,
              renderable.vertexBuffer,
              renderable.indexBuffer,
              recordGeometryRange(
                  renderable,
                  partIndex,
                  primitive.range.indexOffset,
                  primitive.range.indexCount,
                  true,
                  !primitive.pixels),
              primitive.range.indexCount);
    }
    builder.build(engine, renderable.entity);
  } catch (...) {
    destroyRenderable(engine, renderable);
    throw;
  }

  auto& renderables = engine.getRenderableManager();
  renderables.setScreenSpaceContactShadows(
      renderables.getInstance(renderable.entity), true);

  return renderable;
}

Renderable createPlaneRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const Eigen::Vector3d& normal,
    double offset,
    const float4& color,
    const TextureBinding* textureBinding = nullptr,
    const TextureBinding* fallbackTexture = nullptr)
{
  static constexpr double halfExtent = 1.0;
  const ::filament::math::short4 tangent = {0, 0, 0, 32767};
  Eigen::Vector3d unitNormal = normal;
  if (unitNormal.squaredNorm() < 1e-12) {
    unitNormal = Eigen::Vector3d::UnitZ();
  } else {
    unitNormal.normalize();
  }
  const Eigen::Vector3d seed = std::abs(unitNormal.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d axisU = seed.cross(unitNormal).normalized();
  const Eigen::Vector3d axisV = unitNormal.cross(axisU).normalized();
  const Eigen::Vector3d center = unitNormal * offset;

  std::vector<Vertex> vertices{
      {toFloat3(center - axisU * halfExtent - axisV * halfExtent),
       tangent,
       {0.0f, 0.0f}},
      {toFloat3(center + axisU * halfExtent - axisV * halfExtent),
       tangent,
       {4.0f, 0.0f}},
      {toFloat3(center + axisU * halfExtent + axisV * halfExtent),
       tangent,
       {4.0f, 4.0f}},
      {toFloat3(center - axisU * halfExtent + axisV * halfExtent),
       tangent,
       {0.0f, 4.0f}},
  };
  std::vector<std::uint32_t> indices;
  std::vector<::filament::math::uint3> triangles;
  const std::vector<float3> normals(4, toFloat3(unitNormal));
  indices.reserve(6);
  triangles.reserve(2);
  appendTriangle(indices, triangles, 0, 1, 2);
  appendTriangle(indices, triangles, 0, 2, 3);

  const Bounds bounds = computeBounds(vertices);
  PbrTextureBindings textures;
  textures.baseColor = textureBinding;
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      normals,
      color,
      bounds.min,
      bounds.max,
      textures,
      textureBinding != nullptr ? fallbackTexture : nullptr);
}

} // namespace

// True for shapes rendered with a lit material that exposes metallic/roughness/
// reflectance parameters and does not supply its own per-part asset materials.
bool shapeUsesLitMaterialOverride(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Box:
    case ShapeKind::Sphere:
    case ShapeKind::Ellipsoid:
    case ShapeKind::Cylinder:
    case ShapeKind::Capsule:
    case ShapeKind::Cone:
    case ShapeKind::Pyramid:
    case ShapeKind::MultiSphere:
    case ShapeKind::ConvexMesh:
    case ShapeKind::Heightmap:
    case ShapeKind::SoftMesh:
    case ShapeKind::Plane:
      return true;
    case ShapeKind::LineSegments:
    case ShapeKind::PointCloud:
    case ShapeKind::VoxelGrid:
    case ShapeKind::Mesh:
    case ShapeKind::Unsupported:
      return false;
  }
  return false;
}

std::optional<Renderable> createRenderableFromDescriptor(
    ::filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const RenderableDescriptor& descriptor)
{
  const bool transparent = !isOpaqueShapeAlpha(descriptor.material.rgba.w());
  const float4 color = toLinearColor(descriptor.material.rgba);
  auto& solidMaterial = selectLitMaterial(materials, false, transparent);
  auto& texturedMaterial = selectLitMaterial(materials, true, transparent);
  std::optional<Renderable> renderable;
  switch (descriptor.geometry.kind) {
    case ShapeKind::Box:
      renderable = createBoxRenderable(
          engine,
          solidMaterial,
          toFloat3(descriptor.geometry.size * 0.5),
          color);
      break;
    case ShapeKind::Sphere:
      renderable = createEllipsoidRenderable(
          engine,
          solidMaterial,
          Eigen::Vector3d::Constant(descriptor.geometry.radius),
          color);
      break;
    case ShapeKind::Ellipsoid:
      renderable = createEllipsoidRenderable(
          engine, solidMaterial, descriptor.geometry.size * 0.5, color);
      break;
    case ShapeKind::Cylinder:
      renderable = createCylinderRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::Cone:
      renderable = createConeRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::Pyramid:
      renderable = createPyramidRenderable(
          engine, solidMaterial, descriptor.geometry.size, color);
      break;
    case ShapeKind::MultiSphere:
      if (descriptor.geometry.sphereCenters.size()
              == descriptor.geometry.sphereRadii.size()
          && std::any_of(
              descriptor.geometry.sphereRadii.begin(),
              descriptor.geometry.sphereRadii.end(),
              [](double radius) { return radius > 0.0; })) {
        renderable = createMultiSphereRenderable(
            engine,
            solidMaterial,
            descriptor.geometry.sphereCenters,
            descriptor.geometry.sphereRadii,
            color);
      }
      break;
    case ShapeKind::LineSegments:
      renderable = createLineSegmentRenderable(
          engine,
          transparent ? materials.pointColor : materials.opaquePointColor,
          descriptor);
      break;
    case ShapeKind::Capsule:
      renderable = createCapsuleRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::ConvexMesh:
    case ShapeKind::Heightmap:
    case ShapeKind::SoftMesh:
      renderable = createDescriptorTriangleMeshRenderable(
          engine, solidMaterial, descriptor.geometry, color);
      break;
    case ShapeKind::PointCloud:
      renderable = createPointCloudRenderable(engine, materials, descriptor);
      break;
    case ShapeKind::VoxelGrid:
      renderable = createVoxelGridRenderable(engine, materials, descriptor);
      break;
    case ShapeKind::Mesh:
      renderable = createMeshRenderable(
          engine, materials, textureCache, descriptor, color);
      break;
    case ShapeKind::Plane:
      renderable = createPlaneRenderable(
          engine,
          texturedMaterial,
          descriptor.geometry.normal,
          descriptor.geometry.offset,
          color,
          &materials.checkerTexture,
          &materials.fallbackTexture);
      break;
    case ShapeKind::Unsupported:
      break;
  }

  if (renderable) {
    // Retained OSG culls back faces even on open mesh/plane surfaces;
    // SoftMesh is its explicit two-sided exception. Keep shadow-pass culling
    // independent: the SDK's existing PCF caster policy also handles open
    // surfaces and camera-facing point/line proxies.
    for (const auto& material : renderable->materials) {
      // Point clouds blend at every alpha while retaining depth writes.
      if (descriptor.geometry.kind != ShapeKind::Mesh)
        material.instance->setDepthWrite(
            descriptor.geometry.kind == ShapeKind::PointCloud || !transparent);
      material.instance->setCullingMode(
          descriptor.geometry.kind == ShapeKind::SoftMesh
              ? ::filament::MaterialInstance::CullingMode::NONE
              : ::filament::MaterialInstance::CullingMode::BACK,
          material.instance->getShadowCullingMode());
    }
    renderable->wireframeEligible
        = descriptor.geometry.kind != ShapeKind::LineSegments
          && !(
              descriptor.geometry.kind == ShapeKind::Mesh
              && descriptor.geometry.triangleIndices.empty())
          && !(
              descriptor.geometry.kind == ShapeKind::PointCloud
              && descriptor.geometry.pointShapeType
                     == dart::gui::PointShapeType::Point);
    applyRenderableShadowSettings(engine, *renderable, descriptor.material);
    // Apply per-shape PBR overrides only to lit primitives. Unlit line/point/
    // voxel renderables have no PBR parameters, and asset meshes
    // (ShapeKind::Mesh) carry their own per-part materials.
    if (shapeUsesLitMaterialOverride(descriptor.geometry.kind)) {
      applyDescriptorMaterialOverride(*renderable, descriptor.material);
    }
  }

  return renderable;
}

void updateScreenSpaceGeometry(
    ::filament::Engine& engine,
    Renderable& renderable,
    const RenderableDescriptor& descriptor,
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up,
    double fovy,
    int viewportHeight)
{
  const bool billboard
      = descriptor.geometry.kind == ShapeKind::PointCloud
        && descriptor.geometry.pointShapeType != dart::gui::PointShapeType::Box;
  const bool thickLine = descriptor.geometry.kind == ShapeKind::LineSegments;
  const bool meshPixels = renderable.meshScreenVertexOffset != 0;
  if (!billboard && !thickLine && !meshPixels)
    return;
  if (renderable.screenGeometryState) {
    const auto& cached = *renderable.screenGeometryState;
    if ((cached.worldTransform.array()
         == descriptor.worldTransform.matrix().array())
            .all()
        && (cached.eye.array() == eye.array()).all()
        && (cached.center.array() == center.array()).all()
        && (cached.up.array() == up.array()).all()
        && (cached.rgba.array() == descriptor.material.rgba.array()).all()
        && cached.fovy == fovy && cached.viewportHeight == viewportHeight)
      return;
  }
  if (meshPixels) {
    std::vector<Vertex> vertices;
    for (const auto& part : renderable.meshScreenParts) {
      const auto mesh = makeMeshPointLineGeometry(
          descriptor, part, eye, center, up, fovy, viewportHeight);
      const auto pixels = makeMeshPixelVertices(descriptor, part, mesh);
      vertices.insert(vertices.end(), pixels.begin(), pixels.end());
    }
    if (!vertices.empty()) {
      auto bounds = computeBounds(vertices);
      const auto& geometry = descriptor.geometry;
      if (geometry.hasLocalBounds) {
        bounds.min = min(bounds.min, toFloat3(geometry.localBoundsMin));
        bounds.max = max(bounds.max, toFloat3(geometry.localBoundsMax));
      }
      auto& manager = engine.getRenderableManager();
      manager.setAxisAlignedBoundingBox(
          manager.getInstance(renderable.entity),
          makeFilamentBounds(bounds.min, bounds.max));
      renderable.vertexBuffer->setBufferAt(
          engine,
          0,
          makeBufferDescriptor(std::move(vertices)),
          renderable.meshScreenVertexOffset * sizeof(Vertex));
    }
    renderable.screenGeometryState = Renderable::ScreenGeometryState{
        descriptor.worldTransform.matrix(),
        eye,
        center,
        up,
        descriptor.material.rgba,
        fovy,
        viewportHeight};
    return;
  }
  auto triangles = billboard ? dart::gui::makePointCloudVisualTriangles(
                       descriptor, eye, center, up, fovy, viewportHeight)
                             : dart::gui::makeLineSegmentVisualTriangles(
                                 descriptor, eye, center, fovy, viewportHeight);
  std::vector<DebugVertex> vertices;
  std::vector<std::uint32_t> indices;
  vertices.reserve(triangles.size() * 3);
  for (const auto& triangle : triangles)
    appendDebugTriangle(vertices, indices, triangle);
  if (!vertices.empty()) {
    const auto bounds = computeDebugBounds(vertices);
    auto& manager = engine.getRenderableManager();
    manager.setAxisAlignedBoundingBox(
        manager.getInstance(renderable.entity),
        makeFilamentBounds(bounds.min, bounds.max));
    renderable.vertexBuffer->setBufferAt(
        engine, 0, makeBufferDescriptor(std::move(vertices)));
  }
  renderable.screenGeometryState = Renderable::ScreenGeometryState{
      descriptor.worldTransform.matrix(),
      eye,
      center,
      up,
      descriptor.material.rgba,
      fovy,
      viewportHeight};
}

} // namespace dart::gui::detail
