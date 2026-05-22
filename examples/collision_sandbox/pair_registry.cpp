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

#include "pair_registry.hpp"

#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/sdf/dense_sdf_field.hpp>

#include <algorithm>
#include <array>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <cmath>

namespace dart::examples::collision_sandbox {

namespace {

using collision::ShapeType;

constexpr std::array<ShapeFixture, 9> kShapeFixtures{{
    {ShapeType::Sphere, "sphere", "Sphere"},
    {ShapeType::Box, "box", "Box"},
    {ShapeType::Capsule, "capsule", "Capsule"},
    {ShapeType::Cylinder, "cylinder", "Cylinder"},
    {ShapeType::Plane, "plane", "Plane"},
    {ShapeType::Convex, "convex", "Convex"},
    {ShapeType::Mesh, "mesh", "Mesh"},
    {ShapeType::Sdf, "sdf", "SDF"},
    {ShapeType::Compound, "compound", "Compound"},
}};

[[nodiscard]] const ShapeFixture& fixtureFor(ShapeType type)
{
  const auto it = std::ranges::find_if(
      kShapeFixtures,
      [type](const ShapeFixture& fixture) { return fixture.type == type; });
  if (it == kShapeFixtures.end()) {
    throw std::invalid_argument("Unknown collision shape type");
  }
  return *it;
}

[[nodiscard]] std::string makePairId(ShapeType shapeA, ShapeType shapeB)
{
  std::ostringstream stream;
  stream << shapeTypeId(shapeA) << "_" << shapeTypeId(shapeB);
  return stream.str();
}

[[nodiscard]] std::string makePairLabel(ShapeType shapeA, ShapeType shapeB)
{
  std::ostringstream stream;
  stream << shapeTypeLabel(shapeA) << " / " << shapeTypeLabel(shapeB);
  return stream.str();
}

[[nodiscard]] std::string makePairNote(
    ShapeType shapeA, ShapeType shapeB, PairStatus status)
{
  if (status == PairStatus::AdaptedFallback) {
    if (shapeA == ShapeType::Compound || shapeB == ShapeType::Compound) {
      return "Contact query uses compound child recursion.";
    }
    return "Contact query uses a native convex, mesh, or adapter fallback "
           "path.";
  }

  if (status == PairStatus::DistanceOnly) {
    return "Contact query is not implemented; distance query is available.";
  }

  if (status == PairStatus::Unsupported) {
    return "Native contact and distance queries are not implemented for this "
           "pair.";
  }

  return "Contact query uses a direct native narrow-phase path.";
}

[[nodiscard]] PairStatus classifyPair(ShapeType shapeA, ShapeType shapeB)
{
  if (collision::NarrowPhase::isSupported(shapeA, shapeB)) {
    if (isAdaptedFallbackPair(shapeA, shapeB)) {
      return PairStatus::AdaptedFallback;
    }
    return PairStatus::Contact;
  }

  if (collision::NarrowPhase::isDistanceSupported(shapeA, shapeB)) {
    return PairStatus::DistanceOnly;
  }

  return PairStatus::Unsupported;
}

[[nodiscard]] const std::vector<PairCase>& pairCaseStorage()
{
  static const std::vector<PairCase> cases = [] {
    std::vector<PairCase> out;
    out.reserve(kShapeFixtures.size() * (kShapeFixtures.size() + 1) / 2);

    for (std::size_t i = 0; i < kShapeFixtures.size(); ++i) {
      for (std::size_t j = i; j < kShapeFixtures.size(); ++j) {
        const ShapeType shapeA = kShapeFixtures[i].type;
        const ShapeType shapeB = kShapeFixtures[j].type;
        const PairStatus status = classifyPair(shapeA, shapeB);
        out.push_back(
            PairCase{
                out.size(),
                shapeA,
                shapeB,
                status,
                makePairId(shapeA, shapeB),
                makePairLabel(shapeA, shapeB),
                makePairNote(shapeA, shapeB, status)});
      }
    }

    return out;
  }();

  return cases;
}

[[nodiscard]] std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

[[nodiscard]] std::vector<Eigen::Vector3d> makeCubeVertices(
    const Eigen::Vector3d& halfExtents)
{
  return {
      {-halfExtents.x(), -halfExtents.y(), -halfExtents.z()},
      {halfExtents.x(), -halfExtents.y(), -halfExtents.z()},
      {halfExtents.x(), halfExtents.y(), -halfExtents.z()},
      {-halfExtents.x(), halfExtents.y(), -halfExtents.z()},
      {-halfExtents.x(), -halfExtents.y(), halfExtents.z()},
      {halfExtents.x(), -halfExtents.y(), halfExtents.z()},
      {halfExtents.x(), halfExtents.y(), halfExtents.z()},
      {-halfExtents.x(), halfExtents.y(), halfExtents.z()}};
}

[[nodiscard]] std::vector<collision::MeshShape::Triangle> makeCubeTriangles()
{
  return {
      {0, 2, 1},
      {0, 3, 2},
      {4, 5, 6},
      {4, 6, 7},
      {0, 1, 5},
      {0, 5, 4},
      {2, 3, 7},
      {2, 7, 6},
      {0, 4, 7},
      {0, 7, 3},
      {1, 2, 6},
      {1, 6, 5}};
}

[[nodiscard]] std::shared_ptr<collision::DenseSdfField> makeSphereSdf(
    double scale)
{
  constexpr double voxelSize = 0.25;
  const Eigen::Vector3i dims(9, 9, 9);
  const Eigen::Vector3d origin(-1.0, -1.0, -1.0);
  auto field = std::make_shared<collision::DenseSdfField>(
      origin * scale, dims, voxelSize * scale, 10.0 * scale);

  const double radius = 0.45 * scale;
  for (int z = 0; z < dims.z(); ++z) {
    for (int y = 0; y < dims.y(); ++y) {
      for (int x = 0; x < dims.x(); ++x) {
        const Eigen::Vector3i index(x, y, z);
        const Eigen::Vector3d point
            = field->origin() + Eigen::Vector3d(x, y, z) * field->voxelSize();
        field->setDistance(index, point.norm() - radius);
        field->setObserved(index, true);
      }
    }
  }

  return field;
}

[[nodiscard]] Eigen::Isometry3d translated(
    double x, double y = 0.0, double z = 0.0)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

[[nodiscard]] ShapeParameters sanitizedParameters(ShapeParameters params)
{
  params.radius = std::clamp(params.radius, 0.05, 5.0);
  params.height = std::clamp(params.height, 0.05, 10.0);
  params.halfExtents
      = params.halfExtents.cwiseMax(Eigen::Vector3d::Constant(0.025))
            .cwiseMin(Eigen::Vector3d::Constant(5.0));
  return params;
}

} // namespace

std::span<const ShapeFixture> shapeFixtures()
{
  return kShapeFixtures;
}

std::span<const PairCase> pairCases()
{
  return pairCaseStorage();
}

const PairCase& pairCaseAt(std::size_t index)
{
  const auto cases = pairCases();
  if (index >= cases.size()) {
    throw std::out_of_range(
        "Collision sandbox pair case index is out of range");
  }
  return cases[index];
}

const PairCase* findPairCase(std::string_view id)
{
  const auto cases = pairCases();
  const auto it = std::ranges::find_if(
      cases, [id](const PairCase& pair) { return pair.id == id; });
  return it == cases.end() ? nullptr : &*it;
}

std::string_view shapeTypeId(ShapeType type)
{
  return fixtureFor(type).id;
}

std::string_view shapeTypeLabel(ShapeType type)
{
  return fixtureFor(type).label;
}

std::string_view pairStatusLabel(PairStatus status)
{
  switch (status) {
    case PairStatus::Contact:
      return "Contact";
    case PairStatus::AdaptedFallback:
      return "Adapted";
    case PairStatus::DistanceOnly:
      return "Distance-only";
    case PairStatus::Unsupported:
      return "Unsupported";
  }
  return "Unknown";
}

std::string_view pairStatusDescription(PairStatus status)
{
  switch (status) {
    case PairStatus::Contact:
      return "Native contact query runs directly for this pair.";
    case PairStatus::AdaptedFallback:
      return "Native contact query runs through a compound, convex, mesh, or "
             "adapter path.";
    case PairStatus::DistanceOnly:
      return "Native distance query is available, but contact generation is "
             "not implemented.";
    case PairStatus::Unsupported:
      return "Native contact and distance queries are not implemented for this "
             "pair.";
  }
  return "Unknown pair status.";
}

bool isAdaptedFallbackPair(ShapeType shapeA, ShapeType shapeB)
{
  if (shapeA == ShapeType::Compound || shapeB == ShapeType::Compound) {
    return true;
  }

  if (shapeA == ShapeType::Convex || shapeB == ShapeType::Convex) {
    return true;
  }

  if (shapeA == ShapeType::Mesh || shapeB == ShapeType::Mesh) {
    return true;
  }

  return false;
}

ShapeParameters defaultShapeParameters(ShapeType type)
{
  ShapeParameters params;
  switch (type) {
    case ShapeType::Sphere:
      params.radius = 0.45;
      break;
    case ShapeType::Box:
      params.halfExtents = Eigen::Vector3d(0.45, 0.35, 0.3);
      break;
    case ShapeType::Capsule:
      params.radius = 0.25;
      params.height = 0.9;
      break;
    case ShapeType::Cylinder:
      params.radius = 0.35;
      params.height = 0.8;
      break;
    case ShapeType::Plane:
      params.halfExtents = Eigen::Vector3d(1.5, 1.5, 0.0125);
      break;
    case ShapeType::Convex:
      params.radius = 0.55;
      break;
    case ShapeType::Mesh:
      params.halfExtents = Eigen::Vector3d(0.45, 0.45, 0.45);
      break;
    case ShapeType::Sdf:
      params.radius = 0.45;
      break;
    case ShapeType::Compound:
      params.radius = 0.45;
      break;
  }

  return params;
}

std::unique_ptr<collision::Shape> makeShape(ShapeType type, double scale)
{
  ShapeParameters params = defaultShapeParameters(type);
  scale = std::clamp(scale, 0.05, 5.0);
  params.radius *= scale;
  params.height *= scale;
  params.halfExtents *= scale;
  return makeShape(type, params);
}

std::unique_ptr<collision::Shape> makeShape(
    ShapeType type, const ShapeParameters& inputParams)
{
  const ShapeParameters params = sanitizedParameters(inputParams);

  switch (type) {
    case ShapeType::Sphere:
      return std::make_unique<collision::SphereShape>(params.radius);
    case ShapeType::Box:
      return std::make_unique<collision::BoxShape>(params.halfExtents);
    case ShapeType::Capsule:
      return std::make_unique<collision::CapsuleShape>(
          params.radius, params.height);
    case ShapeType::Cylinder:
      return std::make_unique<collision::CylinderShape>(
          params.radius, params.height);
    case ShapeType::Plane:
      return std::make_unique<collision::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0);
    case ShapeType::Convex:
      return std::make_unique<collision::ConvexShape>(
          makeOctahedronVertices(params.radius));
    case ShapeType::Mesh:
      return std::make_unique<collision::MeshShape>(
          makeCubeVertices(params.halfExtents), makeCubeTriangles());
    case ShapeType::Sdf:
      return std::make_unique<collision::SdfShape>(
          makeSphereSdf(params.radius / 0.45));
    case ShapeType::Compound: {
      const double scale = params.radius / 0.45;
      auto compound = std::make_unique<collision::CompoundShape>();
      compound->addChild(
          std::make_unique<collision::SphereShape>(0.28 * scale),
          translated(-0.18 * scale));
      compound->addChild(
          std::make_unique<collision::BoxShape>(
              Eigen::Vector3d(0.25, 0.2, 0.2) * scale),
          translated(0.25 * scale));
      return compound;
    }
  }

  throw std::invalid_argument("Unsupported collision shape type");
}

PairPose defaultPairPose(const PairCase& pair)
{
  PairPose pose;

  if (pair.shapeA == ShapeType::Plane && pair.shapeB == ShapeType::Plane) {
    return pose;
  }

  if (pair.shapeA == ShapeType::Plane) {
    pose.transformB.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);
    return pose;
  }

  if (pair.shapeB == ShapeType::Plane) {
    pose.transformA.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);
    return pose;
  }

  pose.transformB.translation() = Eigen::Vector3d(0.55, 0.0, 0.0);
  return pose;
}

} // namespace dart::examples::collision_sandbox
