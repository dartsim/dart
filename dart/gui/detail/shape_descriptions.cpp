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

#include <dart/config.hpp>

#include <dart/gui/detail/geometry.hpp>
#include <dart/gui/detail/renderable.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/HeightmapShape.hpp>
#include <dart/dynamics/LineSegmentShape.hpp>
#include <dart/dynamics/MeshMaterial.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/MultiSphereConvexHullShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PointCloudShape.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/PyramidShape.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/VoxelGridShape.hpp>

#include <dart/math/Geometry.hpp>
#include <dart/math/PolygonMesh.hpp>
#include <dart/math/TriMesh.hpp>

#include <dart/common/Deprecated.hpp>

#include <assimp/material.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
#include <utility>

#include <cmath>

namespace dart::gui {
namespace {

void setLocalBounds(
    GeometryDescriptor& descriptor,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max)
{
  descriptor.localBoundsMin = min;
  descriptor.localBoundsMax = max;
  descriptor.hasLocalBounds = true;
}

MeshAlphaMode makeMeshAlphaMode(dynamics::MeshShape::AlphaMode mode)
{
  switch (mode) {
    case dynamics::MeshShape::BLEND:
      return MeshAlphaMode::Blend;
    case dynamics::MeshShape::AUTO:
      return MeshAlphaMode::Auto;
    case dynamics::MeshShape::SHAPE_ALPHA:
      return MeshAlphaMode::ShapeAlpha;
  }

  return MeshAlphaMode::Blend;
}

template <typename Derived>
Eigen::Vector3d makeScaledTriangleNormal(
    const Eigen::MatrixBase<Derived>& sourceNormal,
    const Eigen::Vector3d& scale)
{
  const Eigen::Vector3d source = sourceNormal.template cast<double>();
  if (!source.allFinite()) {
    return Eigen::Vector3d::UnitZ();
  }

  Eigen::Vector3d scaledNormal = source;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(scale[axis]) > 1e-12) {
      scaledNormal[axis] /= scale[axis];
    } else {
      scaledNormal[axis] = 0.0;
    }
  }

  if (!scaledNormal.allFinite() || scaledNormal.squaredNorm() <= 1e-12) {
    scaledNormal = source;
  }

  return scaledNormal.squaredNorm() > 1e-12 ? scaledNormal.normalized()
                                            : Eigen::Vector3d::UnitZ();
}

template <typename S>
void setTriangleMeshData(
    GeometryDescriptor& descriptor,
    const math::TriMesh<S>& triMesh,
    const Eigen::Vector3d& scale = Eigen::Vector3d::Ones())
{
  const auto& vertices = triMesh.getVertices();
  const auto& triangles = triMesh.getTriangles();
  if (vertices.empty() || triangles.empty()
      || vertices.size()
             > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }

  descriptor.triangleVertices.clear();
  descriptor.triangleIndices.clear();
  descriptor.triangleNormals.clear();
  descriptor.triangleVertices.reserve(vertices.size());
  descriptor.triangleIndices.reserve(triangles.size());

  for (const auto& vertex : vertices) {
    const Eigen::Vector3d point
        = scale.cwiseProduct(vertex.template cast<double>());
    if (!point.allFinite()) {
      descriptor.triangleVertices.clear();
      descriptor.triangleIndices.clear();
      descriptor.triangleNormals.clear();
      return;
    }
    descriptor.triangleVertices.push_back(point);
  }

  if (triMesh.hasVertexNormals()
      && triMesh.getVertexNormals().size() == vertices.size()) {
    descriptor.triangleNormals.reserve(triMesh.getVertexNormals().size());
    for (const auto& normal : triMesh.getVertexNormals()) {
      descriptor.triangleNormals.push_back(
          makeScaledTriangleNormal(normal, scale));
    }
  }

  for (const auto& triangle : triangles) {
    if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size()
        || triangle[2] >= vertices.size()) {
      continue;
    }

    descriptor.triangleIndices.emplace_back(
        static_cast<int>(triangle[0]),
        static_cast<int>(triangle[1]),
        static_cast<int>(triangle[2]));
  }
}

template <typename S>
void setHeightmapTriangleData(
    GeometryDescriptor& descriptor,
    const dynamics::HeightmapShape<S>& heightmapShape)
{
  const auto& heightmap = heightmapShape.getHeightField();
  const Eigen::Index rows = heightmap.rows();
  const Eigen::Index cols = heightmap.cols();
  if (rows < 2 || cols < 2
      || static_cast<std::size_t>(heightmap.size())
             > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }

  const auto scale = heightmapShape.getScale().template cast<double>();
  descriptor.triangleVertices.clear();
  descriptor.triangleIndices.clear();
  descriptor.triangleNormals.clear();
  descriptor.triangleVertices.reserve(
      static_cast<std::size_t>(heightmap.size()));
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index col = 0; col < cols; ++col) {
      descriptor.triangleVertices.emplace_back(
          static_cast<double>(col) * scale.x(),
          -static_cast<double>(row) * scale.y(),
          static_cast<double>(heightmap(row, col)) * scale.z());
    }
  }

  const auto vertexIndex = [cols](Eigen::Index row, Eigen::Index col) {
    return static_cast<int>(row * cols + col);
  };

  descriptor.triangleIndices.reserve(
      static_cast<std::size_t>((rows - 1) * (cols - 1) * 2));
  for (Eigen::Index row = 1; row < rows; ++row) {
    for (Eigen::Index col = 1; col < cols; ++col) {
      const int p1 = vertexIndex(row - 1, col - 1);
      const int p2 = vertexIndex(row - 1, col);
      const int p3 = vertexIndex(row, col - 1);
      const int current = vertexIndex(row, col);
      descriptor.triangleIndices.emplace_back(p1, p3, p2);
      descriptor.triangleIndices.emplace_back(p2, p3, current);
    }
  }

  // Match OSG's cardinal-neighbor wedges rather than the mesh diagonals.
  descriptor.triangleNormals.reserve(descriptor.triangleVertices.size());
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index col = 0; col < cols; ++col) {
      const auto& current = descriptor.triangleVertices[vertexIndex(row, col)];
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      const auto addNormal = [&](int first, int second) {
        const Eigen::Vector3d normal
            = (descriptor.triangleVertices[first] - current)
                  .cross(descriptor.triangleVertices[second] - current);
        const double length = normal.stableNorm();
        if (normal.allFinite() && std::isfinite(length) && length > 0)
          sum += normal / length;
      };
      if (row > 0 && col > 0)
        addNormal(vertexIndex(row - 1, col), vertexIndex(row, col - 1));
      if (row + 1 < rows && col > 0)
        addNormal(vertexIndex(row, col - 1), vertexIndex(row + 1, col));
      if (row + 1 < rows && col + 1 < cols)
        addNormal(vertexIndex(row + 1, col), vertexIndex(row, col + 1));
      if (row > 0 && col + 1 < cols)
        addNormal(vertexIndex(row, col + 1), vertexIndex(row - 1, col));
      descriptor.triangleNormals.push_back(
          sum.allFinite() && sum.squaredNorm() > 1e-12
              ? Eigen::Vector3d(sum.normalized())
              : Eigen::Vector3d::UnitZ());
    }
  }
}

void setSoftMeshTriangleData(
    GeometryDescriptor& descriptor, const dynamics::SoftMeshShape& softMesh)
{
  const auto* softBody = softMesh.getSoftBodyNode();
  if (!softBody || softBody->getNumPointMasses() == 0u
      || softBody->getNumPointMasses()
             > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    return;

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i)
    descriptor.triangleVertices.push_back(
        softBody->getPointMass(i)->getLocalPosition());
  for (std::size_t i = 0; i < softBody->getNumFaces(); ++i) {
    const auto& face = softBody->getFace(i);
    if ((face.array() >= 0).all()
        && (face.array() < static_cast<int>(descriptor.triangleVertices.size()))
               .all())
      descriptor.triangleIndices.push_back(face);
  }
}

void setVertexBounds(GeometryDescriptor& descriptor)
{
  if (descriptor.triangleVertices.empty())
    return;
  Eigen::Vector3d min = descriptor.triangleVertices.front();
  Eigen::Vector3d max = min;
  for (const auto& vertex : descriptor.triangleVertices) {
    min = min.cwiseMin(vertex);
    max = max.cwiseMax(vertex);
  }
  descriptor.size = max - min;
  setLocalBounds(descriptor, min, max);
}

#if HAVE_OCTOMAP
void setVoxelGridData(
    GeometryDescriptor& descriptor, const dynamics::VoxelGridShape& voxelGrid)
{
  const auto tree = voxelGrid.getOctree();
  if (!tree)
    return;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  bool hasBounds = false;
  for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
    if (!tree->isNodeOccupied(*it))
      continue;
    const double size = it.getSize();
    const Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
    const Eigen::Vector3d halfExtent = Eigen::Vector3d::Constant(size * 0.5);
    descriptor.voxelCenters.push_back(center);
    descriptor.voxelSizes.push_back(size);
    descriptor.voxelSize = size;
    if (!hasBounds) {
      min = center - halfExtent;
      max = center + halfExtent;
      hasBounds = true;
    } else {
      min = min.cwiseMin(center - halfExtent);
      max = max.cwiseMax(center + halfExtent);
    }
  }
  if (hasBounds) {
    descriptor.size = max - min;
    setLocalBounds(descriptor, min, max);
  }
}
#endif

Eigen::Matrix4d toMatrix(const aiMatrix4x4& m)
{
  Eigen::Matrix4d matrix;
  matrix << m.a1, m.a2, m.a3, m.a4, m.b1, m.b2, m.b3, m.b4, m.c1, m.c2, m.c3,
      m.c4, m.d1, m.d2, m.d3, m.d4;
  return matrix;
}

std::string texturePath(
    const aiMaterial& material,
    aiTextureType type,
    const std::filesystem::path& base)
{
  aiString path;
  if (material.GetTexture(type, 0, &path) != AI_SUCCESS)
    return {};
  std::string name = path.C_Str();
  std::replace(name.begin(), name.end(), '\\', '/');
  if (name.empty() || name.front() == '*')
    return name;
  return (base / std::filesystem::path(name)).lexically_normal().string();
}

MeshMaterialDescriptor describeMaterial(
    const aiMaterial& material, const std::filesystem::path& base)
{
  MeshMaterialDescriptor descriptor;
  const auto readColor = [&](const char* key,
                             unsigned int type,
                             unsigned int index,
                             Eigen::Vector4d& value) {
    aiColor4D color;
    if (material.Get(key, type, index, color) == AI_SUCCESS)
      value = Eigen::Vector4d(color.r, color.g, color.b, color.a);
  };
  readColor(AI_MATKEY_COLOR_AMBIENT, descriptor.ambient);
  readColor(AI_MATKEY_COLOR_DIFFUSE, descriptor.diffuse);
  readColor(AI_MATKEY_COLOR_SPECULAR, descriptor.specular);
  readColor(AI_MATKEY_COLOR_EMISSIVE, descriptor.emissive);
  float value;
  if (material.Get(AI_MATKEY_SHININESS, value) == AI_SUCCESS)
    descriptor.shininess = value;
  if (material.Get(AI_MATKEY_SHININESS_STRENGTH, value) == AI_SUCCESS)
    descriptor.shininessStrength = value;
  if (material.Get(AI_MATKEY_METALLIC_FACTOR, value) == AI_SUCCESS) {
    descriptor.metallicFactor = value;
    descriptor.hasPbrMetallic = true;
  }
  if (material.Get(AI_MATKEY_ROUGHNESS_FACTOR, value) == AI_SUCCESS) {
    descriptor.roughnessFactor = value;
    descriptor.hasPbrRoughness = true;
  }
  descriptor.baseColorTexturePath
      = texturePath(material, aiTextureType_BASE_COLOR, base);
  if (descriptor.baseColorTexturePath.empty())
    descriptor.baseColorTexturePath
        = texturePath(material, aiTextureType_DIFFUSE, base);
  descriptor.metallicTexturePath
      = texturePath(material, aiTextureType_METALNESS, base);
  descriptor.roughnessTexturePath
      = texturePath(material, aiTextureType_DIFFUSE_ROUGHNESS, base);
  descriptor.normalTexturePath
      = texturePath(material, aiTextureType_NORMALS, base);
  descriptor.occlusionTexturePath
      = texturePath(material, aiTextureType_LIGHTMAP, base);
  descriptor.emissiveTexturePath
      = texturePath(material, aiTextureType_EMISSIVE, base);
  // Retained OSG rejects all-white (or greater) Phong specular RGBA, including
  // Assimp's synthesized STL coefficient, and keeps its default black value.
  // Explicit PBR materials and every other authored coefficient remain intact.
  const bool legacy = !descriptor.hasPbrMetallic && !descriptor.hasPbrRoughness
                      && descriptor.metallicTexturePath.empty()
                      && descriptor.roughnessTexturePath.empty()
                      && descriptor.metallicRoughnessTexturePath.empty();
  if (legacy && (descriptor.specular.array() >= 1.0).all())
    descriptor.specular = {0, 0, 0, 1};
  if (!descriptor.baseColorTexturePath.empty())
    descriptor.textureImagePaths.push_back(descriptor.baseColorTexturePath);
  return descriptor;
}

void setAssimpMeshData(
    GeometryDescriptor& descriptor, const dynamics::MeshShape& shape)
{
  // The Assimp tree carries instance transforms, UVs and colors absent from
  // DART 6's TriMesh cache.
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto* scene = shape.getMesh();
  DART_SUPPRESS_DEPRECATED_END
  if (!scene)
    return;
  std::string meshPath = shape.getMeshPath();
  if (meshPath.empty()
      && shape.getMeshUri2().mScheme.get_value_or("file") == "file")
    meshPath = shape.getMeshUri2().getFilesystemPath();
  const auto base = std::filesystem::path(meshPath).parent_path();
  for (unsigned int i = 0; i < scene->mNumMaterials; ++i) {
    descriptor.meshMaterials.push_back(
        scene->mMaterials[i] ? describeMaterial(*scene->mMaterials[i], base)
                             : MeshMaterialDescriptor{});
  }
  descriptor.meshUsesMaterialColors
      = shape.getColorMode() == dynamics::MeshShape::MATERIAL_COLOR;
  const auto appendMesh = [&](unsigned int meshIndex,
                              const Eigen::Matrix4d& nodeTransform) {
    if (meshIndex >= scene->mNumMeshes || !scene->mMeshes[meshIndex])
      return;
    const auto& mesh = *scene->mMeshes[meshIndex];
    if (!mesh.mVertices
        || descriptor.triangleVertices.size() + mesh.mNumVertices
               > static_cast<std::size_t>(std::numeric_limits<int>::max()))
      return;
    Eigen::Matrix4d transform = nodeTransform;
    transform.topRows<3>()
        = shape.getScale().asDiagonal() * transform.topRows<3>();
    const Eigen::Matrix3d linear = transform.topLeftCorner<3, 3>();
    const Eigen::Matrix3d normalTransform
        = std::abs(linear.determinant()) > 1e-12
              ? Eigen::Matrix3d(linear.inverse().transpose())
              : Eigen::Matrix3d::Identity();
    const aiColor4D* colors = nullptr;
    if (shape.getColorMode() == dynamics::MeshShape::COLOR_INDEX) {
      int channel
          = std::min(shape.getColorIndex(), AI_MAX_NUMBER_OF_COLOR_SETS - 1);
      while (channel >= 0 && !colors)
        colors = mesh.mColors[channel--];
    }
    MeshPartDescriptor part;
    part.vertexOffset = descriptor.triangleVertices.size();
    part.vertexCount = mesh.mNumVertices;
    part.triangleOffset = descriptor.triangleIndices.size();
    part.pointOffset = descriptor.meshPointIndices.size();
    part.lineOffset = descriptor.meshLineIndices.size();
    part.materialIndex = mesh.mMaterialIndex;
    part.usesVertexColors = colors != nullptr;
    for (unsigned int i = 0; i < mesh.mNumVertices; ++i) {
      const auto& v = mesh.mVertices[i];
      const Eigen::Vector4d point
          = transform * Eigen::Vector4d(v.x, v.y, v.z, 1.0);
      if (!point.allFinite()) {
        descriptor.unsupportedReason
            = "MeshShape contains non-finite transformed vertices";
        descriptor.triangleVertices.resize(part.vertexOffset);
        descriptor.triangleNormals.resize(part.vertexOffset);
        descriptor.meshTextureCoordinates.resize(part.vertexOffset);
        descriptor.meshVertexColors.resize(part.vertexOffset);
        return;
      }
      descriptor.triangleVertices.push_back(point.head<3>());
      Eigen::Vector3d normal = Eigen::Vector3d::Zero();
      if (mesh.mNormals) {
        const auto& n = mesh.mNormals[i];
        normal = normalTransform * Eigen::Vector3d(n.x, n.y, n.z);
        if (normal.allFinite() && normal.squaredNorm() > 1e-12)
          normal.normalize();
        else
          normal.setZero();
      }
      descriptor.triangleNormals.push_back(normal);
      Eigen::Vector3d uv = Eigen::Vector3d::Zero();
      if (mesh.mTextureCoords[0]) {
        const auto& source = mesh.mTextureCoords[0][i];
        uv = Eigen::Vector3d(source.x, source.y, source.z);
        descriptor.meshTextureCoordComponents = std::max(
            descriptor.meshTextureCoordComponents,
            static_cast<int>(mesh.mNumUVComponents[0]));
      }
      descriptor.meshTextureCoordinates.push_back(uv);
      descriptor.meshVertexColors.push_back(
          colors ? Eigen::Vector4d(
              colors[i].r, colors[i].g, colors[i].b, colors[i].a)
                 : Eigen::Vector4d::Ones());
    }
    descriptor.meshUsesVertexColors = descriptor.meshUsesVertexColors || colors;
    std::vector<int> polygon;
    std::vector<Eigen::Vector3i> triangles;
    for (unsigned int i = 0; mesh.mFaces && i < mesh.mNumFaces; ++i) {
      const auto& face = mesh.mFaces[i];
      if (face.mNumIndices == 0 || !face.mIndices
          || std::any_of(
              face.mIndices,
              face.mIndices + face.mNumIndices,
              [&](unsigned int index) { return index >= mesh.mNumVertices; }))
        continue;
      const int offset = static_cast<int>(part.vertexOffset);
      if (face.mNumIndices == 1) {
        descriptor.meshPointIndices.push_back(offset + face.mIndices[0]);
        continue;
      }
      if (face.mNumIndices == 2) {
        descriptor.meshLineIndices.emplace_back(
            offset + face.mIndices[0], offset + face.mIndices[1]);
        continue;
      }
      polygon.clear();
      for (unsigned int j = 0; j < face.mNumIndices; ++j)
        polygon.push_back(offset + face.mIndices[j]);
      if (!math::detail::triangulateFaceEarClipping(
              polygon, descriptor.triangleVertices, triangles)) {
        // Match OSG's fallback for degenerate or non-simple asset polygons.
        triangles.clear();
        for (std::size_t j = 1; j + 1 < polygon.size(); ++j)
          triangles.emplace_back(polygon[0], polygon[j], polygon[j + 1]);
      }
      descriptor.triangleIndices.insert(
          descriptor.triangleIndices.end(), triangles.begin(), triangles.end());
    }
    part.triangleCount
        = descriptor.triangleIndices.size() - part.triangleOffset;
    part.pointCount = descriptor.meshPointIndices.size() - part.pointOffset;
    part.lineCount = descriptor.meshLineIndices.size() - part.lineOffset;
    descriptor.meshParts.push_back(part);
  };
  std::function<void(const aiNode&, const Eigen::Matrix4d&)> visit;
  visit = [&](const aiNode& node, const Eigen::Matrix4d& parent) {
    const Eigen::Matrix4d transform = parent * toMatrix(node.mTransformation);
    for (unsigned int i = 0; i < node.mNumMeshes; ++i)
      appendMesh(node.mMeshes[i], transform);
    for (unsigned int i = 0; i < node.mNumChildren; ++i) {
      if (node.mChildren[i])
        visit(*node.mChildren[i], transform);
    }
  };
  if (scene->mRootNode)
    visit(*scene->mRootNode, Eigen::Matrix4d::Identity());
  else {
    for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
      appendMesh(i, Eigen::Matrix4d::Identity());
  }
  std::vector<Eigen::Vector3d> normals(
      descriptor.triangleVertices.size(), Eigen::Vector3d::Zero());
  for (const auto& triangle : descriptor.triangleIndices) {
    const Eigen::Vector3d normal
        = (descriptor.triangleVertices[triangle.y()]
           - descriptor.triangleVertices[triangle.x()])
              .cross(
                  descriptor.triangleVertices[triangle.z()]
                  - descriptor.triangleVertices[triangle.x()]);
    for (int index : triangle)
      normals[index] += normal;
  }
  for (std::size_t i = 0; i < descriptor.triangleNormals.size(); ++i) {
    if (descriptor.triangleNormals[i].squaredNorm() < 1e-12)
      descriptor.triangleNormals[i]
          = normals[i].squaredNorm() > 1e-12
                ? Eigen::Vector3d(normals[i].normalized())
                : Eigen::Vector3d::UnitZ();
  }
  setVertexBounds(descriptor);
}

void setSymmetricLocalBounds(
    GeometryDescriptor& descriptor, const Eigen::Vector3d& halfExtents)
{
  setLocalBounds(descriptor, -halfExtents, halfExtents);
}

void setPlaneProxyBounds(
    GeometryDescriptor& descriptor,
    const Eigen::Vector3d& normal,
    double offset)
{
  static constexpr double halfExtent = 1.0;
  static constexpr double halfThickness = 0.02;

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

  Eigen::Vector3d min = center;
  Eigen::Vector3d max = center;
  const std::array<Eigen::Vector3d, 6> points
      = {center - axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent + axisV * halfExtent,
         center - axisU * halfExtent + axisV * halfExtent,
         center - unitNormal * halfThickness,
         center + unitNormal * halfThickness};
  for (const Eigen::Vector3d& point : points) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }

  setLocalBounds(descriptor, min, max);
}

void setDiagnosticIfMissingGeometry(
    GeometryDescriptor& descriptor, const std::string& reason)
{
  if (descriptor.unsupportedReason.empty()) {
    descriptor.unsupportedReason = reason;
  }
}

} // namespace

std::optional<GeometryDescriptor> describeShape(const dynamics::Shape& shape)
{
  GeometryDescriptor descriptor;
  descriptor.shapeType = std::string(shape.getType());

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(&shape)) {
    descriptor.kind = ShapeKind::Box;
    descriptor.size = box->getSize();
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(&shape)) {
    descriptor.kind = ShapeKind::Sphere;
    descriptor.radius = sphere->getRadius();
    descriptor.size = Eigen::Vector3d::Constant(sphere->getRadius() * 2.0);
    setSymmetricLocalBounds(
        descriptor, Eigen::Vector3d::Constant(sphere->getRadius()));
    return descriptor;
  }

  if (const auto* ellipsoid
      = dynamic_cast<const dynamics::EllipsoidShape*>(&shape)) {
    descriptor.kind = ShapeKind::Ellipsoid;
    descriptor.size = ellipsoid->getDiameters();
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(&shape)) {
    descriptor.kind = ShapeKind::Cylinder;
    descriptor.radius = cylinder->getRadius();
    descriptor.height = cylinder->getHeight();
    descriptor.size = Eigen::Vector3d(
        cylinder->getRadius() * 2.0,
        cylinder->getRadius() * 2.0,
        cylinder->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* capsule
      = dynamic_cast<const dynamics::CapsuleShape*>(&shape)) {
    descriptor.kind = ShapeKind::Capsule;
    descriptor.radius = capsule->getRadius();
    descriptor.height = capsule->getHeight();
    descriptor.size = Eigen::Vector3d(
        capsule->getRadius() * 2.0,
        capsule->getRadius() * 2.0,
        capsule->getHeight() + capsule->getRadius() * 2.0);
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* cone = dynamic_cast<const dynamics::ConeShape*>(&shape)) {
    descriptor.kind = ShapeKind::Cone;
    descriptor.radius = cone->getRadius();
    descriptor.height = cone->getHeight();
    descriptor.size = Eigen::Vector3d(
        cone->getRadius() * 2.0, cone->getRadius() * 2.0, cone->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* pyramid
      = dynamic_cast<const dynamics::PyramidShape*>(&shape)) {
    descriptor.kind = ShapeKind::Pyramid;
    descriptor.height = pyramid->getHeight();
    descriptor.size = Eigen::Vector3d(
        pyramid->getBaseWidth(), pyramid->getBaseDepth(), pyramid->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* multiSphere
      = dynamic_cast<const dynamics::MultiSphereConvexHullShape*>(&shape)) {
    descriptor.kind = ShapeKind::MultiSphere;
    const auto& spheres = multiSphere->getSpheres();
    descriptor.sphereCenters.reserve(spheres.size());
    descriptor.sphereRadii.reserve(spheres.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    for (const auto& [radius, center] : spheres) {
      if (radius <= 0.0) {
        continue;
      }
      descriptor.sphereRadii.push_back(radius);
      descriptor.sphereCenters.push_back(center);
      descriptor.radius = std::max(descriptor.radius, radius);
      const Eigen::Vector3d extent = Eigen::Vector3d::Constant(radius);
      if (!hasBounds) {
        min = center - extent;
        max = center + extent;
        hasBounds = true;
      } else {
        min = min.cwiseMin(center - extent);
        max = max.cwiseMax(center + extent);
      }
    }
    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    } else {
      setDiagnosticIfMissingGeometry(
          descriptor,
          "MultiSphereConvexHullShape has no positive-radius spheres");
    }
    const auto hull = makeMultiSphereMeshGeometry(
        descriptor.sphereCenters, descriptor.sphereRadii);
    descriptor.triangleVertices.reserve(hull.vertices.size());
    descriptor.triangleNormals.reserve(hull.vertices.size());
    for (const auto& vertex : hull.vertices) {
      descriptor.triangleVertices.push_back(vertex.position.cast<double>());
      descriptor.triangleNormals.push_back(vertex.normal.cast<double>());
    }
    descriptor.triangleIndices.reserve(hull.triangles.size());
    for (const auto& triangle : hull.triangles)
      descriptor.triangleIndices.emplace_back(
          triangle.a, triangle.b, triangle.c);
    return descriptor;
  }

  if (const auto* lineSegments
      = dynamic_cast<const dynamics::LineSegmentShape*>(&shape)) {
    descriptor.kind = ShapeKind::LineSegments;
    descriptor.lineThickness = lineSegments->getThickness();
    const auto vertices = lineSegments->getVertices();
    descriptor.lineVertices.reserve(vertices.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    for (const Eigen::Vector3d& vertex : vertices) {
      descriptor.lineVertices.push_back(vertex);
      if (!hasBounds) {
        min = vertex;
        max = vertex;
        hasBounds = true;
      } else {
        min = min.cwiseMin(vertex);
        max = max.cwiseMax(vertex);
      }
    }

    const auto connections = lineSegments->getConnections();
    descriptor.lineConnections.reserve(connections.size());
    for (const Eigen::Vector2i& connection : connections) {
      if (connection.x() < 0 || connection.y() < 0) {
        continue;
      }
      const auto first = static_cast<std::size_t>(connection.x());
      const auto second = static_cast<std::size_t>(connection.y());
      if (first >= descriptor.lineVertices.size()
          || second >= descriptor.lineVertices.size()) {
        continue;
      }
      descriptor.lineConnections.push_back(connection);
    }

    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    } else {
      setDiagnosticIfMissingGeometry(
          descriptor, "LineSegmentShape has no vertices");
    }
    if (descriptor.lineConnections.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "LineSegmentShape has no valid line connections");
    }
    return descriptor;
  }

  if (const auto* convexMesh
      = dynamic_cast<const dynamics::ConvexMeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::ConvexMesh;
    const auto& mesh = convexMesh->getMesh();
    if (mesh != nullptr && !mesh->getVertices().empty()) {
      setTriangleMeshData(descriptor, *mesh);
      Eigen::Vector3d min = mesh->getVertices().front();
      Eigen::Vector3d max = min;
      for (const Eigen::Vector3d& vertex : mesh->getVertices()) {
        min = min.cwiseMin(vertex);
        max = max.cwiseMax(vertex);
      }
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    }
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "ConvexMeshShape has no triangle mesh data");
    }
    return descriptor;
  }

  if (const auto* pointCloud
      = dynamic_cast<const dynamics::PointCloudShape*>(&shape)) {
    descriptor.kind = ShapeKind::PointCloud;
    descriptor.pointSize = pointCloud->getVisualSize();
    descriptor.pointShapeType
        = static_cast<PointShapeType>(pointCloud->getPointShapeType());
    const auto points = pointCloud->getPoints();
    descriptor.pointCloudPoints.reserve(points.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    const Eigen::Vector3d extent
        = Eigen::Vector3d::Constant(std::max(0.0, descriptor.pointSize) * 0.5);
    for (const Eigen::Vector3d& point : points) {
      descriptor.pointCloudPoints.push_back(point);
      const Eigen::Vector3d pointMin = point - extent;
      const Eigen::Vector3d pointMax = point + extent;
      if (!hasBounds) {
        min = pointMin;
        max = pointMax;
        hasBounds = true;
      } else {
        min = min.cwiseMin(pointMin);
        max = max.cwiseMax(pointMax);
      }
    }

    const auto colors = pointCloud->getColors();
    if (pointCloud->getColorMode() == dynamics::PointCloudShape::BIND_OVERALL
        && !colors.empty()) {
      descriptor.pointCloudColors.push_back(colors.front());
    } else if (
        pointCloud->getColorMode() == dynamics::PointCloudShape::BIND_PER_POINT
        && colors.size() == points.size()) {
      descriptor.pointCloudColors.assign(colors.begin(), colors.end());
    }

    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    } else {
      setDiagnosticIfMissingGeometry(
          descriptor, "PointCloudShape has no points");
    }
    return descriptor;
  }

  if (const auto* heightmap
      = dynamic_cast<const dynamics::HeightmapShapef*>(&shape)) {
    descriptor.kind = ShapeKind::Heightmap;
    setHeightmapTriangleData(descriptor, *heightmap);
    setVertexBounds(descriptor);
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "HeightmapShape has no triangle mesh data");
    }
    return descriptor;
  }

  if (const auto* softMesh
      = dynamic_cast<const dynamics::SoftMeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::SoftMesh;
    setSoftMeshTriangleData(descriptor, *softMesh);
    setVertexBounds(descriptor);
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "SoftMeshShape has no triangle mesh data");
    }
    return descriptor;
  }

#if HAVE_OCTOMAP
  if (const auto* voxelGrid
      = dynamic_cast<const dynamics::VoxelGridShape*>(&shape)) {
    descriptor.kind = ShapeKind::VoxelGrid;
    setVoxelGridData(descriptor, *voxelGrid);
    if (descriptor.voxelCenters.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "VoxelGridShape has no occupied voxels");
    }
    return descriptor;
  }

#endif

  if (const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::Mesh;
    descriptor.scale = mesh->getScale();
    descriptor.meshUri = mesh->getMeshUri();
    descriptor.meshAlphaMode = makeMeshAlphaMode(mesh->getAlphaMode());
    setAssimpMeshData(descriptor, *mesh);
    if (descriptor.triangleVertices.empty()
        || (descriptor.triangleIndices.empty()
            && descriptor.meshPointIndices.empty()
            && descriptor.meshLineIndices.empty())) {
      std::string reason = "MeshShape has no visual mesh data";
      if (!descriptor.meshUri.empty()) {
        reason += " for '" + descriptor.meshUri + "'";
      }
      setDiagnosticIfMissingGeometry(descriptor, reason);
    }
    return descriptor;
  }

  if (const auto* plane = dynamic_cast<const dynamics::PlaneShape*>(&shape)) {
    descriptor.kind = ShapeKind::Plane;
    descriptor.normal = plane->getNormal();
    descriptor.offset = plane->getOffset();
    setPlaneProxyBounds(descriptor, descriptor.normal, descriptor.offset);
    return descriptor;
  }

  descriptor.kind = ShapeKind::Unsupported;
  descriptor.unsupportedReason = "Shape type '" + descriptor.shapeType
                                 + "' is not supported by dart::gui";
  return descriptor;
}

} // namespace dart::gui
