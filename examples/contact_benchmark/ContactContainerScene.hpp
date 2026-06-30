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

#ifndef DART_EXAMPLES_CONTACT_BENCHMARK_CONTACT_CONTAINER_SCENE_HPP_
#define DART_EXAMPLES_CONTACT_BENCHMARK_CONTACT_CONTAINER_SCENE_HPP_

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <random>
#include <string>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::examples::contact_benchmark {

constexpr double kDefaultContactContainerSpacing = 1.02;
constexpr std::size_t kDefaultContactContainerLayers = 4;
constexpr double kDefaultContactContainerJitter = 0.08;
constexpr double kDefaultContactContainerWallThickness = 0.2;
constexpr double kDefaultContactContainerDropHeight = 1.75;
constexpr std::uint32_t kDefaultContactContainerSeed = 3056u;

struct ContactContainerOptions
{
  std::size_t objects = 30;
  double spacing = kDefaultContactContainerSpacing;
  std::size_t layers = kDefaultContactContainerLayers;
  std::optional<double> containerSize;
  std::optional<double> wallHeight;
  double containerWallThickness = kDefaultContactContainerWallThickness;
  double jitter = kDefaultContactContainerJitter;
  double dropHeight = 0.0;
  std::uint32_t seed = kDefaultContactContainerSeed;
  double initialSpeed = 0.0;
  double initialAngularSpeed = 0.0;
  collision::CollisionDetectorPtr collisionDetector;
};

namespace detail {

struct ShapeSpec
{
  dynamics::ShapePtr shape;
  double centerHeight = 0.5;
  Eigen::Vector4d color = Eigen::Vector4d::Ones();
  const char* namePrefix = "object";
};

struct ContainerLayout
{
  std::size_t layers = 1;
  std::size_t columns = 1;
  std::size_t rows = 1;
  double innerLength = 4.0;
  double innerWidth = 4.0;
  double wallHeight = 2.0;
};

inline std::size_t divideRoundUp(std::size_t numerator, std::size_t denominator)
{
  return (numerator + denominator - 1u) / denominator;
}

inline std::size_t ceilSqrt(std::size_t value)
{
  if (value <= 1u)
    return value;

  return static_cast<std::size_t>(
      std::ceil(std::sqrt(static_cast<double>(value))));
}

inline ContainerLayout computeLayout(const ContactContainerOptions& options)
{
  ContainerLayout layout;
  layout.layers = std::max<std::size_t>(
      1u, std::min(options.layers, std::max<std::size_t>(1u, options.objects)));
  const std::size_t objectsPerLayer
      = divideRoundUp(options.objects, layout.layers);
  layout.columns = std::max<std::size_t>(1u, ceilSqrt(objectsPerLayer));
  layout.rows = divideRoundUp(objectsPerLayer, layout.columns);

  const double fitLength
      = (static_cast<double>(layout.columns) + 1.5) * options.spacing;
  const double fitWidth
      = (static_cast<double>(layout.rows) + 1.5) * options.spacing;
  if (options.containerSize.has_value()) {
    layout.innerLength = *options.containerSize;
    layout.innerWidth = *options.containerSize;
  } else {
    layout.innerLength = std::max(4.0, fitLength);
    layout.innerWidth = std::max(4.0, fitWidth);
  }

  const double verticalSpacing = std::max(0.9, options.spacing);
  const double fitHeight
      = static_cast<double>(layout.layers) * verticalSpacing + 1.5;
  layout.wallHeight = options.wallHeight.value_or(std::max(2.0, fitHeight));

  return layout;
}

inline void setShapeInertia(
    dynamics::BodyNode* body, const dynamics::ShapePtr& shape, double mass)
{
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);
}

inline ShapeSpec makeShape(std::size_t index)
{
  switch (index % 3u) {
    case 0u:
      return {
          std::make_shared<dynamics::SphereShape>(0.5),
          0.5,
          Eigen::Vector4d(0.0, 0.15, 1.0, 1.0),
          "sphere"};
    case 1u:
      return {
          std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(1.0)),
          0.5,
          Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
          "box"};
    default:
      return {
          std::make_shared<dynamics::CylinderShape>(0.5, 1.0),
          0.5,
          Eigen::Vector4d(0.0, 0.75, 0.0, 1.0),
          "cylinder"};
  }
}

inline void addStaticBox(
    dynamics::BodyNode* body,
    const Eigen::Vector3d& dimensions,
    const Eigen::Vector3d& translation,
    const Eigen::Vector4d& color)
{
  auto shape = std::make_shared<dynamics::BoxShape>(dimensions);
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  shapeNode->setRelativeTransform(transform);
  shapeNode->getVisualAspect()->setColor(color);
}

inline void addContainer(
    const simulation::WorldPtr& world,
    const ContainerLayout& layout,
    const ContactContainerOptions& options)
{
  auto container = dynamics::Skeleton::create("container");
  auto* body
      = container->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr)
            .second;

  const double t = options.containerWallThickness;
  const double floorLength = layout.innerLength + 2.0 * t;
  const double floorWidth = layout.innerWidth + 2.0 * t;
  const double halfLength = 0.5 * layout.innerLength;
  const double halfWidth = 0.5 * layout.innerWidth;
  const double halfWallHeight = 0.5 * layout.wallHeight;
  const Eigen::Vector4d floorColor(0.45, 0.45, 0.45, 1.0);
  const Eigen::Vector4d wallColor(0.35, 0.35, 0.38, 0.42);

  addStaticBox(
      body,
      Eigen::Vector3d(floorLength, floorWidth, t),
      Eigen::Vector3d(0.0, 0.0, -0.5 * t),
      floorColor);
  addStaticBox(
      body,
      Eigen::Vector3d(t, floorWidth, layout.wallHeight),
      Eigen::Vector3d(-(halfLength + 0.5 * t), 0.0, halfWallHeight),
      wallColor);
  addStaticBox(
      body,
      Eigen::Vector3d(t, floorWidth, layout.wallHeight),
      Eigen::Vector3d(halfLength + 0.5 * t, 0.0, halfWallHeight),
      wallColor);
  addStaticBox(
      body,
      Eigen::Vector3d(layout.innerLength, t, layout.wallHeight),
      Eigen::Vector3d(0.0, -(halfWidth + 0.5 * t), halfWallHeight),
      wallColor);
  addStaticBox(
      body,
      Eigen::Vector3d(layout.innerLength, t, layout.wallHeight),
      Eigen::Vector3d(0.0, halfWidth + 0.5 * t, halfWallHeight),
      wallColor);

  container->setMobile(false);
  world->addSkeleton(container);
}

} // namespace detail

[[nodiscard]] inline simulation::WorldPtr createContactContainerWorld(
    const ContactContainerOptions& options)
{
  const auto layout = detail::computeLayout(options);
  const std::size_t objectsPerLayer = layout.columns * layout.rows;

  auto world = simulation::World::create("generated_contact_container");
  world->setTimeStep(0.001);
  if (options.collisionDetector)
    world->getConstraintSolver()->setCollisionDetector(
        options.collisionDetector);

  detail::addContainer(world, layout, options);

  std::mt19937 rng(options.seed);
  const double jitterLimit = 0.45 * options.spacing;
  const double jitter = std::min(options.jitter, jitterLimit);
  std::uniform_real_distribution<double> jitterDist(-jitter, jitter);
  std::uniform_real_distribution<double> angleDist(-0.25, 0.25);
  const double verticalSpacing = std::max(0.9, options.spacing);

  for (std::size_t i = 0; i < options.objects; ++i) {
    const std::size_t layer = i / objectsPerLayer;
    const std::size_t inLayer = i % objectsPerLayer;
    const std::size_t row = inLayer / layout.columns;
    const std::size_t column = inLayer % layout.columns;
    const auto spec = detail::makeShape(i);

    auto skeleton = dynamics::Skeleton::create(
        std::string("container_") + spec.namePrefix + "_" + std::to_string(i));
    auto pair
        = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr);
    auto* joint = pair.first;
    auto* body = pair.second;
    auto* shapeNode = body->createShapeNodeWith<
        dynamics::VisualAspect,
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(spec.shape);
    shapeNode->getVisualAspect()->setColor(spec.color);
    detail::setShapeInertia(body, spec.shape, 1.0);

    const double x = (static_cast<double>(column)
                      - 0.5 * static_cast<double>(layout.columns - 1u))
                         * options.spacing
                     + jitterDist(rng);
    const double y = (static_cast<double>(row)
                      - 0.5 * static_cast<double>(layout.rows - 1u))
                         * options.spacing
                     + jitterDist(rng);
    const double z = spec.centerHeight
                     + static_cast<double>(layer) * verticalSpacing
                     + 0.05 * static_cast<double>((row + column) % 2u)
                     + std::max(0.0, options.dropHeight);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear()
        = (Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();
    transform.translation() = Eigen::Vector3d(x, y, z);
    dynamics::FreeJoint::setTransformOf(body, transform);

    if (options.initialSpeed > 0.0) {
      const double phase = static_cast<double>((i % 17u) + 1u);
      joint->setLinearVelocity(
          options.initialSpeed
          * Eigen::Vector3d(std::sin(phase), std::cos(phase * 0.7), 0.0));
    }
    if (options.initialAngularSpeed > 0.0) {
      const double phase = static_cast<double>((i % 19u) + 1u);
      joint->setAngularVelocity(
          options.initialAngularSpeed
          * Eigen::Vector3d(
              std::cos(phase * 0.5), std::sin(phase), std::cos(phase)));
    }

    world->addSkeleton(skeleton);
  }

  return world;
}

} // namespace dart::examples::contact_benchmark

#endif // DART_EXAMPLES_CONTACT_BENCHMARK_CONTACT_CONTAINER_SCENE_HPP_
