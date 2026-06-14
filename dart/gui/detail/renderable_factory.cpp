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

#include <dart/gui/geometry.hpp>

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

constexpr int kDebugLineTubeSegments = 8;
constexpr double kTwoPi = 6.2831853071795864769;
constexpr float kDefaultDielectricMetallic = 0.0f;
constexpr float kDefaultMatteRoughness = 0.82f;

struct Vertex
{
  float3 position;
  ::filament::math::short4 tangent;
  ::filament::math::float2 uv = {0.0f, 0.0f};
};

struct DebugVertex
{
  ::filament::math::float3 position;
  std::uint32_t color = 0;
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

float4 toFloat4(const Eigen::Vector4d& vector)
{
  return {
      static_cast<float>(vector.x()),
      static_cast<float>(vector.y()),
      static_cast<float>(vector.z()),
      static_cast<float>(vector.w())};
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
  if (hasUsableTextureCoordinates(vertices)) {
    uvs.reserve(vertices.size());
    positions.reserve(vertices.size());
    std::transform(
        vertices.begin(),
        vertices.end(),
        std::back_inserter(uvs),
        [](const Vertex& vertex) { return vertex.uv; });
    std::transform(
        vertices.begin(),
        vertices.end(),
        std::back_inserter(positions),
        [](const Vertex& vertex) { return vertex.position; });
  }

  ::filament::geometry::SurfaceOrientation::Builder builder;
  builder.vertexCount(vertices.size())
      .positions(
          positions.empty() ? &vertices[0].position : positions.data(),
          positions.empty() ? sizeof(Vertex) : 0)
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
    std::exit(1);
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));
}

float4 toRgba(const Eigen::Vector4d& rgba)
{
  return {
      static_cast<float>(rgba.x()),
      static_cast<float>(rgba.y()),
      static_cast<float>(rgba.z()),
      static_cast<float>(rgba.w())};
}

float3 rgb(const float4& color)
{
  return {color.x, color.y, color.z};
}

float luminance(const float3& color)
{
  return 0.2126f * color.x + 0.7152f * color.y + 0.0722f * color.z;
}

bool isNearBlackTextureTint(const float4& color)
{
  return luminance(rgb(color)) < 0.05f;
}

float4 ensureReadableDisplayColor(const float4& color)
{
  constexpr float kMinLuminance = 0.16f;
  const float3 colorRgb = rgb(color);
  const float currentLuminance = luminance(colorRgb);
  if (currentLuminance >= kMinLuminance || color.w <= 0.0f) {
    return color;
  }

  constexpr float3 kNeutralMeshColor{0.28f, 0.30f, 0.34f};
  const float blend = std::clamp(
      (kMinLuminance - currentLuminance) / kMinLuminance, 0.0f, 1.0f);
  return {
      std::clamp(
          color.x * (1.0f - blend) + kNeutralMeshColor.x * blend, 0.0f, 1.0f),
      std::clamp(
          color.y * (1.0f - blend) + kNeutralMeshColor.y * blend, 0.0f, 1.0f),
      std::clamp(
          color.z * (1.0f - blend) + kNeutralMeshColor.z * blend, 0.0f, 1.0f),
      color.w};
}

float3 ensureReadableEmissiveColor(const float3& emissive, bool sourceWasDark)
{
  if (!sourceWasDark || luminance(emissive) > 0.0f) {
    return emissive;
  }

  return {0.025f, 0.027f, 0.030f};
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
      0,  1,  2,  0,  2,  3,  4,  6,  5,  4,  7,  6,  8,  9,  10, 8,  10, 11,
      12, 14, 13, 12, 15, 14, 16, 17, 18, 16, 18, 19, 20, 22, 21, 20, 23, 22};
  static constexpr std::array<::filament::math::ushort3, 12> triangles = {{
      {0, 1, 2},
      {0, 2, 3},
      {4, 6, 5},
      {4, 7, 6},
      {8, 9, 10},
      {8, 10, 11},
      {12, 14, 13},
      {12, 15, 14},
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

  std::unique_ptr<::filament::geometry::SurfaceOrientation> orientation(
      ::filament::geometry::SurfaceOrientation::Builder()
          .vertexCount(vertices.size())
          .positions(&vertices[0].position, sizeof(Vertex))
          .triangleCount(triangles.size())
          .triangles(triangles.data())
          .build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    std::exit(1);
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
            .normalized(::filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(vertices));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::USHORT)
            .build(engine);
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
          0,
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

std::uint8_t toByte(double channel)
{
  return static_cast<std::uint8_t>(std::clamp(channel, 0.0, 1.0) * 255.0 + 0.5);
}

std::uint32_t packColor(const Eigen::Vector4d& rgba)
{
  const auto red = static_cast<std::uint32_t>(toByte(rgba.x()));
  const auto green = static_cast<std::uint32_t>(toByte(rgba.y()));
  const auto blue = static_cast<std::uint32_t>(toByte(rgba.z()));
  const auto alpha = static_cast<std::uint32_t>(toByte(rgba.w()));
  return red | (green << 8u) | (blue << 16u) | (alpha << 24u);
}

void appendThinDebugLine(
    std::vector<DebugVertex>& vertices,
    std::vector<std::uint32_t>& indices,
    const DebugLineDescriptor& line)
{
  const auto start = static_cast<std::uint32_t>(vertices.size());
  const std::uint32_t color = packColor(line.rgba);
  vertices.push_back({toFloat3(line.from), color});
  vertices.push_back({toFloat3(line.to), color});
  indices.push_back(start);
  indices.push_back(start + 1u);
}

bool appendThickDebugLine(
    std::vector<DebugVertex>& vertices,
    std::vector<std::uint32_t>& triangleIndices,
    const DebugLineDescriptor& line)
{
  if (line.thickness <= 0.0 || !std::isfinite(line.thickness)
      || !line.from.allFinite() || !line.to.allFinite()) {
    return false;
  }

  const Eigen::Vector3d segment = line.to - line.from;
  const double length = segment.norm();
  if (!std::isfinite(length) || length < 1e-9) {
    return false;
  }

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
  const std::uint32_t color = packColor(line.rgba);
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

  const auto start = static_cast<std::uint32_t>(vertices.size());
  const std::uint32_t color = packColor(triangle.rgba);
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
                ::filament::VertexBuffer::AttributeType::UBYTE4,
                offsetof(DebugVertex, color),
                sizeof(DebugVertex))
            .normalized(::filament::VertexAttribute::COLOR)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::UINT)
            .build(engine);
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
            0,
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
            triangleIndexOffset,
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
  std::vector<DebugLineDescriptor> lines;
  lines.reserve(descriptor.geometry.lineConnections.size());
  for (const Eigen::Vector2i& connection :
       descriptor.geometry.lineConnections) {
    if (connection.x() < 0 || connection.y() < 0) {
      continue;
    }
    const auto first = static_cast<std::size_t>(connection.x());
    const auto second = static_cast<std::size_t>(connection.y());
    if (first >= descriptor.geometry.lineVertices.size()
        || second >= descriptor.geometry.lineVertices.size()) {
      continue;
    }

    DebugLineDescriptor line;
    line.from = descriptor.geometry.lineVertices[first];
    line.to = descriptor.geometry.lineVertices[second];
    line.rgba = descriptor.material.rgba;
    line.label = descriptor.shapeFrameName;
    lines.push_back(line);
  }

  return createDebugLineRenderable(engine, material, lines);
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
            .normalized(::filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::UINT)
            .build(engine);
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
          0,
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

  float4 color = toRgba(descriptor.material.rgba);
  if (descriptor.geometry.pointCloudColors.size() == 1u) {
    color = toRgba(descriptor.geometry.pointCloudColors.front());
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
              .normalized(::filament::VertexAttribute::TANGENTS)
              .build(engine);
    renderable.vertexBuffer->setBufferAt(
        engine, 0, makeBufferDescriptor(std::move(buffers.vertices)));

    renderable.indexBuffer
        = ::filament::IndexBuffer::Builder()
              .indexCount(buffers.indices.size())
              .bufferType(::filament::IndexBuffer::IndexType::UINT)
              .build(engine);
    renderable.indexBuffer->setBuffer(
        engine, makeBufferDescriptor(std::move(buffers.indices)));

    renderable.entity = EntityManager::get().create();
    auto builder = ::filament::RenderableManager::Builder(pointRanges.size());
    builder.boundingBox(makeFilamentBounds(bounds.min, bounds.max))
        .castShadows(true)
        .receiveShadows(true);
    for (std::size_t i = 0; i < pointRanges.size(); ++i) {
      const float4 pointColor = ensureReadableDisplayColor(
          toRgba(descriptor.geometry.pointCloudColors[i]));
      auto* materialInstance = addRenderableMaterial(
          renderable,
          selectLitMaterial(materials, false, pointColor),
          pointColor,
          false);
      configureLitMaterialInstance(
          *materialInstance, pointColor, 0.0f, 0.58f, {0.0f, 0.0f, 0.0f});
      builder.material(i, materialInstance)
          .geometry(
              i,
              ::filament::RenderableManager::PrimitiveType::TRIANGLES,
              renderable.vertexBuffer,
              renderable.indexBuffer,
              pointRanges[i].indexOffset,
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
      selectLitMaterial(materials, false, color),
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

  const float4 color = toRgba(descriptor.material.rgba);
  const double voxelSize = std::max(descriptor.geometry.voxelSize, 1e-4);
  MeshGeometry geometry;
  geometry.vertices.reserve(descriptor.geometry.voxelCenters.size() * 24u);
  geometry.indices.reserve(descriptor.geometry.voxelCenters.size() * 36u);
  geometry.triangles.reserve(descriptor.geometry.voxelCenters.size() * 12u);

  for (const Eigen::Vector3d& center : descriptor.geometry.voxelCenters) {
    appendBoxMeshGeometry(
        geometry, center, Eigen::Vector3d::Constant(voxelSize));
  }

  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  const Bounds bounds = buffers.bounds;
  return createTriangleMeshRenderable(
      engine,
      selectLitMaterial(materials, false, color),
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

std::optional<Renderable> createMeshRenderable(
    ::filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const GeometryDescriptor& geometry,
    const float4& color)
{
  const auto& meshVertices = geometry.triangleVertices;
  const auto& meshTriangles = geometry.triangleIndices;
  if (meshVertices.empty() || meshTriangles.empty()) {
    return std::nullopt;
  }
  if (meshVertices.size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const ::filament::math::short4 tangent = {0, 0, 0, 32767};
  const bool hasTextureCoords
      = geometry.meshTextureCoordComponents >= 2
        && geometry.meshTextureCoordinates.size() == meshVertices.size();

  std::vector<Vertex> vertices;
  vertices.reserve(meshVertices.size());
  for (std::size_t i = 0; i < meshVertices.size(); ++i) {
    if (!meshVertices[i].allFinite()) {
      return std::nullopt;
    }

    ::filament::math::float2 uv = {0.0f, 0.0f};
    if (hasTextureCoords) {
      const Eigen::Vector3d& textureCoordinate
          = geometry.meshTextureCoordinates[i];
      if (!textureCoordinate.allFinite()) {
        return std::nullopt;
      }
      uv
          = {static_cast<float>(textureCoordinate.x()),
             static_cast<float>(textureCoordinate.y())};
    }
    vertices.push_back(Vertex{toFloat3(meshVertices[i]), tangent, uv});
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
    bool sourceWasDark = false;
  };

  const auto makeMaterialState = [&](unsigned int materialIndex) {
    MeshMaterialState state;
    state.sourceWasDark = luminance(rgb(color)) < 0.12f;
    state.baseColor = ensureReadableDisplayColor(color);

    if (!geometry.meshUsesMaterialColors || geometry.meshMaterials.empty()) {
      state.emissiveColor = ensureReadableEmissiveColor(
          state.emissiveColor, state.sourceWasDark);
      return state;
    }

    if (materialIndex < geometry.meshMaterials.size()) {
      const MeshMaterialDescriptor& meshMaterial
          = geometry.meshMaterials[materialIndex];
      state.followsDescriptorColor = false;
      state.baseColor = toFloat4(meshMaterial.diffuse);
      state.sourceWasDark = luminance(rgb(state.baseColor)) < 0.12f;
      state.baseColor.w = resolveMeshMaterialAlpha(
          color.w, state.baseColor.w, geometry.meshAlphaMode);
      state.metallic = static_cast<float>(meshMaterial.metallicFactor);
      state.roughness = static_cast<float>(meshMaterial.roughnessFactor);
      state.emissiveColor
          = {static_cast<float>(meshMaterial.emissive.x()),
             static_cast<float>(meshMaterial.emissive.y()),
             static_cast<float>(meshMaterial.emissive.z())};

      if (hasTextureCoords) {
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

      if (state.textures.baseColor != nullptr
          && isNearBlackTextureTint(state.baseColor)) {
        state.baseColor = {1.0f, 1.0f, 1.0f, state.baseColor.w};
      } else {
        state.baseColor = ensureReadableDisplayColor(state.baseColor);
      }
    }

    state.emissiveColor
        = ensureReadableEmissiveColor(state.emissiveColor, state.sourceWasDark);
    return state;
  };

  const auto makeMaterialInstance
      = [&](Renderable& renderable,
            const MeshMaterialState& state) -> ::filament::MaterialInstance* {
    const bool usesTextures = hasTextureBindings(state.textures);
    auto* materialInstance = addRenderableMaterial(
        renderable,
        selectLitMaterial(materials, usesTextures, state.baseColor),
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
    return materialInstance;
  };

  const Bounds bounds = resolveDescriptorBounds(geometry, vertices);
  std::vector<MeshPartDescriptor> parts;
  parts.reserve(geometry.meshParts.size());
  std::size_t coveredTriangleCount = 0;
  bool useSubMeshes = !geometry.meshParts.empty();
  for (const MeshPartDescriptor& range : geometry.meshParts) {
    if (range.triangleCount == 0) {
      continue;
    }
    if (range.triangleOffset + range.triangleCount > meshTriangles.size()) {
      useSubMeshes = false;
      break;
    }
    coveredTriangleCount += range.triangleCount;
    parts.push_back(range);
  }
  useSubMeshes = useSubMeshes && !parts.empty()
                 && coveredTriangleCount == meshTriangles.size();
  if (!useSubMeshes) {
    const MeshMaterialState state = makeMaterialState(0u);
    const bool usesTextures = hasTextureBindings(state.textures);
    return createTriangleMeshRenderable(
        engine,
        selectLitMaterial(materials, usesTextures, state.baseColor),
        std::move(vertices),
        std::move(indices),
        std::move(triangles),
        std::move(normals),
        state.baseColor,
        bounds.min,
        bounds.max,
        state.textures,
        usesTextures ? &materials.fallbackTexture : nullptr,
        state.metallic,
        state.roughness,
        state.emissiveColor,
        state.followsDescriptorColor);
  }

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
            .normalized(::filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = ::filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(::filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  renderable.entity = EntityManager::get().create();
  auto builder = ::filament::RenderableManager::Builder(parts.size());
  builder.boundingBox(makeFilamentBounds(bounds.min, bounds.max))
      .castShadows(true)
      .receiveShadows(true);
  for (std::size_t partIndex = 0; partIndex < parts.size(); ++partIndex) {
    const auto& part = parts[partIndex];
    auto* materialInstance = makeMaterialInstance(
        renderable, makeMaterialState(part.materialIndex));
    builder.material(partIndex, materialInstance)
        .geometry(
            partIndex,
            ::filament::RenderableManager::PrimitiveType::TRIANGLES,
            renderable.vertexBuffer,
            renderable.indexBuffer,
            part.triangleOffset * 3u,
            part.triangleCount * 3u);
  }
  builder.build(engine, renderable.entity);

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
  const float4 color = toRgba(descriptor.material.rgba);
  auto& solidMaterial = selectLitMaterial(materials, false, color);
  auto& texturedMaterial = selectLitMaterial(materials, true, color);
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
          engine, materials.debugColor, descriptor);
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
          engine, materials, textureCache, descriptor.geometry, color);
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

} // namespace dart::gui::detail
