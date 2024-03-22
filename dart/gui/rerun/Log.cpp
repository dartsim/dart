/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include "dart/gui/rerun/Log.hpp"

#include "dart/common/Logging.hpp"
#include "dart/common/String.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PyramidShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"

namespace dart::gui::rerun {

namespace {

::rerun::datatypes::Vec3D toRerunVec3D(const Eigen::Vector3d& vec)
{
  return {
      static_cast<float>(vec[0]),
      static_cast<float>(vec[1]),
      static_cast<float>(vec[2])};
}

::rerun::datatypes::Quaternion toRerunQuaternion(const Eigen::Quaterniond& quat)
{
  return ::rerun::datatypes::Quaternion{
      static_cast<float>(quat.x()),
      static_cast<float>(quat.y()),
      static_cast<float>(quat.z()),
      static_cast<float>(quat.w())};
}

::rerun::datatypes::Quaternion toRerunQuaternion(const Eigen::Matrix3d& rot)
{
  return toRerunQuaternion(Eigen::Quaterniond(rot));
}

void logBoxShape(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const dynamics::BoxShape& box,
    const Eigen::Isometry3d& tf)
{
  rec.log(
      name,
      ::rerun::Boxes3D::from_half_sizes({toRerunVec3D(box.getSize() / 2.0)})
          .with_centers({toRerunVec3D(tf.translation())})
          .with_rotations({toRerunQuaternion(tf.linear())}));
}

void logAssimpMesh(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const aiMesh* mesh,
    const Eigen::Isometry3d& tf)
{
  std::vector<::rerun::Position3D> vertex_positions;
  std::vector<::rerun::Position3D> vertex_normals;
  vertex_positions.reserve(mesh->mNumVertices);
  vertex_normals.reserve(mesh->mNumVertices);
  for (auto i = 0u; i < mesh->mNumVertices; ++i) {
    const ::rerun::Position3D pos = toRerunVec3D(
        tf
        * Eigen::Vector3d(
            mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z));
    vertex_positions.push_back(std::move(pos));
    const ::rerun::Position3D normal = toRerunVec3D(
        tf.linear()
        * Eigen::Vector3d(
            mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z));
    vertex_normals.push_back(std::move(normal));
  }

  std::vector<uint32_t> indices;
  indices.reserve(mesh->mNumFaces * 3);
  for (auto i = 0u; i < mesh->mNumFaces; ++i) {
    const auto& face = mesh->mFaces[i];
    if (face.mNumIndices != 3) {
      DART_ERROR("Only triangle faces are supported");
      continue;
    }
    indices.push_back(face.mIndices[0]);
    indices.push_back(face.mIndices[1]);
    indices.push_back(face.mIndices[2]);
  }

  rec.log(
      name,
      ::rerun::Mesh3D(vertex_positions)
          .with_vertex_normals(vertex_normals)
          .with_mesh_properties(
              ::rerun::components::MeshProperties::from_triangle_indices(
                  indices)));
}

void logAssimpNodeRecurse(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const aiNode* node,
    const aiScene* scene,
    const Eigen::Isometry3d& tf)
{
  for (auto i = 0u; i < node->mNumMeshes; ++i) {
    auto mesh = scene->mMeshes[node->mMeshes[i]];
    for (auto j = 0u; j < mesh->mNumVertices; ++j) {
      logAssimpMesh(rec, name, mesh, tf);
    }
  }

  for (auto i = 0u; i < node->mNumChildren; ++i) {
    logAssimpNodeRecurse(rec, name, node->mChildren[i], scene, tf);
  }
}

void logMeshShape(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const dynamics::MeshShape& mesh,
    const Eigen::Isometry3d& tf)
{
  const auto* assimpScene = mesh.getMesh();
  logAssimpNodeRecurse(rec, name, assimpScene->mRootNode, assimpScene, tf);
}

void logShapeNode(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const dynamics::ShapeNode& shapeNode)
{
  auto shape = shapeNode.getShape();
  auto tf = shapeNode.getWorldTransform();

  if (const auto* sphere = shape->as<dynamics::SphereShape>()) {
    (void)sphere;
  } else if (const auto* box = shape->as<dynamics::BoxShape>()) {
    logBoxShape(rec, name, *box, tf);
  } else if (const auto* plane = shape->as<dynamics::PlaneShape>()) {
    (void)plane;
  } else if (const auto* ellipsoid = shape->as<dynamics::EllipsoidShape>()) {
    (void)ellipsoid;
  } else if (const auto* cylinder = shape->as<dynamics::CylinderShape>()) {
    (void)cylinder;
  } else if (const auto* capsule = shape->as<dynamics::CapsuleShape>()) {
    (void)capsule;
  } else if (const auto* cone = shape->as<dynamics::ConeShape>()) {
    (void)cone;
  } else if (const auto* pyramid = shape->as<dynamics::PyramidShape>()) {
    (void)pyramid;
  } else if (
      const auto* multiSphere
      = shape->as<dynamics::MultiSphereConvexHullShape>()) {
    (void)multiSphere;
  } else if (const auto* mesh = shape->as<dynamics::MeshShape>()) {
    logMeshShape(rec, name, *mesh, tf);
  } else if (const auto* softMesh = shape->as<dynamics::SoftMeshShape>()) {
    (void)softMesh;
  } else if (
      const auto* lineSegmentShape = shape->as<dynamics::LineSegmentShape>()) {
    (void)lineSegmentShape;
  } else {
    DART_ERROR("Unsupported shape type: %s", shape->getType().c_str());
  }
}

} // namespace

void logBodyNode(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const dynamics::BodyNode& bodyNode)
{
  (void)name;
  for (auto i = 0u; i < bodyNode.getNumShapeNodes(); ++i) {
    auto shapeNode = bodyNode.getShapeNode(i);
    logShapeNode(
        rec,
        std::string(name) + "/"
            + common::removeWhitespace(shapeNode->getName()),
        *shapeNode);
  }
}

void logSkeleton(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const dynamics::Skeleton& skeleton)
{
  for (auto i = 0u; i < skeleton.getNumBodyNodes(); ++i) {
    auto bodyNode = skeleton.getBodyNode(i);
    logBodyNode(rec, std::string(name) + "/" + bodyNode->getName(), *bodyNode);
  }
}

void logWorld(
    const ::rerun::RecordingStream& rec,
    std::string_view name,
    const simulation::World& world)
{
  rec.set_time_seconds("sim_time", world.getTime());
  for (auto i = 0u; i < world.getNumSkeletons(); ++i) {
    auto skel = world.getSkeleton(i);
    logSkeleton(rec, std::string(name) + "/" + skel->getName(), *skel);
  }
}

} // namespace dart::gui::rerun
