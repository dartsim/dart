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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Tracks the public rigid-body fixed-joint facade and the AVBD contact-stage
// projection path it activates. The benchmark is a DART-internal baseline for
// regression tracking, not a solver-completeness or paper-parity claim.
// Historical `BM_AvbdArticulated*` rows exercise the Variational multibody
// family, not AVBD; every stepping row asserts and reports its resolved runtime
// identity so those labels cannot silently become solver claims.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/multibody/multibody_options.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <dart/capture_source_provenance.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <bit>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <cmath>
#include <cstdint>

// Reaching the process environment is the only complete way to prove that no
// BENCHMARK_* variable is steering this run; std::getenv can only answer for
// names the guard already knows.
#if defined(_WIN32)
  #include <stdlib.h>
#elif defined(__APPLE__)
  #include <crt_externs.h>
#else
  #include <unistd.h>
#endif

namespace sx = dart::simulation;
namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

sx::JointSpec makePointJointSpec(
    std::string_view name,
    sx::JointType type,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  sx::JointSpec spec;
  spec.name = std::string(name);
  spec.type = type;
  spec.axis = axis;
  spec.parentAnchor = parentAnchor;
  spec.childAnchor = childAnchor;
  return spec;
}

sx::Joint addPointJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& parent,
    const sx::Frame& child,
    sx::JointType type,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return world.addJoint(
      parent,
      child,
      makePointJointSpec(name, type, axis, parentAnchor, childAnchor));
}

sx::Joint addPointJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& child,
    sx::JointType type,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return world.addJoint(
      child, makePointJointSpec(name, type, axis, parentAnchor, childAnchor));
}

sx::Joint addFixedJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& parent,
    const sx::Frame& child,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      parent,
      child,
      sx::JointType::Fixed,
      Eigen::Vector3d::UnitZ(),
      parentAnchor,
      childAnchor);
}

sx::Joint addRevoluteJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& parent,
    const sx::Frame& child,
    const Eigen::Vector3d& axis,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      parent,
      child,
      sx::JointType::Revolute,
      axis,
      parentAnchor,
      childAnchor);
}

sx::Joint addRevoluteJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& child,
    const Eigen::Vector3d& axis,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      child,
      sx::JointType::Revolute,
      axis,
      parentAnchor,
      childAnchor);
}

sx::Joint addPrismaticJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& parent,
    const sx::Frame& child,
    const Eigen::Vector3d& axis,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      parent,
      child,
      sx::JointType::Prismatic,
      axis,
      parentAnchor,
      childAnchor);
}

sx::Joint addPrismaticJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& child,
    const Eigen::Vector3d& axis,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      child,
      sx::JointType::Prismatic,
      axis,
      parentAnchor,
      childAnchor);
}

sx::Joint addSphericalJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& parent,
    const sx::Frame& child,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      parent,
      child,
      sx::JointType::Spherical,
      Eigen::Vector3d::UnitZ(),
      parentAnchor,
      childAnchor);
}

sx::Joint addSphericalJoint(
    sx::World& world,
    std::string_view name,
    const sx::Frame& child,
    std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
    std::optional<Eigen::Vector3d> childAnchor = std::nullopt)
{
  return addPointJoint(
      world,
      name,
      child,
      sx::JointType::Spherical,
      Eigen::Vector3d::UnitZ(),
      parentAnchor,
      childAnchor);
}

void setProjectionStiffness(
    sx::Joint& joint, double linearStiffness, double angularStiffness)
{
  auto policy = joint.getConstraintProjectionPolicy();
  policy.linearStiffness = linearStiffness;
  policy.angularStiffness = angularStiffness;
  joint.setConstraintProjectionPolicy(policy);
}

std::unique_ptr<sx::World> makeAvbdEmptyWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);
  return world;
}

std::unique_ptr<sx::World> makeRigidFixedJointWorld(std::size_t linkCount)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("base", baseOptions);

  std::vector<sx::RigidBody> links;
  links.reserve(linkCount);
  for (std::size_t i = 0; i < linkCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position = Eigen::Vector3d(
        static_cast<double>(i + 1), 0.1 * static_cast<double>(i % 3), 0.0);
    bodyOptions.linearVelocity = Eigen::Vector3d(0.15, 0.0, 0.0);
    bodyOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.2);

    auto child = world->addRigidBody("link_" + std::to_string(i), bodyOptions);
    addFixedJoint(*world, "fixed_" + std::to_string(i), parent, child);
    parent = child;
    links.push_back(child);
  }

  benchmark::DoNotOptimize(links.data());
  return world;
}

std::unique_ptr<sx::World> makeRigidRevoluteMotorWorld(std::size_t motorCount)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("motor_base", baseOptions);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.mass = 1.0;
    bodyOptions.position = Eigen::Vector3d(
        0.75 * static_cast<double>(i + 1),
        0.08 * static_cast<double>(i % 2),
        0.0);

    auto child
        = world->addRigidBody("motor_link_" + std::to_string(i), bodyOptions);
    auto joint = addRevoluteJoint(
        *world,
        "motor_hinge_" + std::to_string(i),
        parent,
        child,
        Eigen::Vector3d::UnitZ());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.75 + 0.05 * static_cast<double>(i % 3)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -600.0),
        Eigen::VectorXd::Constant(1, 600.0));

    parent = child;
    links.push_back(child);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeRigidPrismaticMotorWorld(std::size_t motorCount)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("prismatic_motor_base", baseOptions);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.mass = 1.0;
    bodyOptions.position = Eigen::Vector3d(
        0.65 * static_cast<double>(i + 1),
        0.08 * static_cast<double>(i % 2),
        0.0);

    auto child = world->addRigidBody(
        "prismatic_motor_link_" + std::to_string(i), bodyOptions);
    auto joint = addPrismaticJoint(
        *world,
        "prismatic_motor_axis_" + std::to_string(i),
        parent,
        child,
        Eigen::Vector3d::UnitX());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.6 + 0.04 * static_cast<double>(i % 3)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -700.0),
        Eigen::VectorXd::Constant(1, 700.0));

    parent = child;
    links.push_back(child);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dMotorWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, -10.0, 0.0);
  auto ground = world->addRigidBody("demo2d_motor_ground", groundOptions);

  sx::RigidBodyOptions barOptions;
  barOptions.mass = 1.0;
  barOptions.position = Eigen::Vector3d::Zero();
  auto bar = world->addRigidBody("demo2d_motor_bar", barOptions);

  auto joint = addRevoluteJoint(
      *world, "demo2d_motor_pin", ground, bar, Eigen::Vector3d::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 20.0));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -50.0), Eigen::VectorXd::Constant(1, 50.0));

  return world;
}

Eigen::Matrix3d fullBoxInertia(const Eigen::Vector3d& size, const double mass)
{
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = mass * (size.y() * size.y() + size.z() * size.z()) / 12.0;
  inertia(1, 1) = mass * (size.x() * size.x() + size.z() * size.z()) / 12.0;
  inertia(2, 2) = mass * (size.x() * size.x() + size.y() * size.y()) / 12.0;
  return inertia;
}

sx::RigidBody addAvbdDemo3dSourceBox(
    sx::World& world,
    const std::string& name,
    const Eigen::Vector3d& size,
    const double density,
    const Eigen::Vector3d& position,
    const bool isStatic = false,
    const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero())
{
  sx::RigidBodyOptions options;
  options.position = position;
  options.linearVelocity = linearVelocity;
  options.isStatic = isStatic;
  if (!isStatic) {
    options.mass = size.prod() * density;
    options.inertia = fullBoxInertia(size, options.mass);
  }

  auto body = world.addRigidBody(name, options);
  body.setFriction(0.5);
  body.setCollisionShape(sx::CollisionShape::makeBox(0.5 * size));
  return body;
}

sx::RigidBody addAvbdDemo2dSourceBox(
    sx::World& world,
    const std::string& name,
    const Eigen::Vector2d& size2d,
    const double density,
    const Eigen::Vector3d& position,
    const bool isStatic = false,
    const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero(),
    const double friction = 0.5,
    const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity())
{
  constexpr double kDepth = 0.2;
  const Eigen::Vector3d size(size2d.x(), size2d.y(), kDepth);

  sx::RigidBodyOptions options;
  options.position = position;
  options.orientation = orientation;
  options.linearVelocity = linearVelocity;
  options.isStatic = isStatic;
  if (!isStatic) {
    options.mass = size2d.prod() * density;
    options.inertia = fullBoxInertia(size, options.mass);
  }

  auto body = world.addRigidBody(name, options);
  body.setFriction(friction);
  body.setCollisionShape(sx::CollisionShape::makeBox(0.5 * size));
  return body;
}

std::unique_ptr<sx::World> makeAvbdDemo2dGroundWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_ground_slab",
      Eigen::Vector2d(100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dDynamicFrictionWorld(
    const double maxFriction = 5.0)
{
  constexpr int kBoxCount = 11;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_dynamic_friction_ground",
      Eigen::Vector2d(100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true,
      Eigen::Vector3d::Zero(),
      0.5);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(kBoxCount);
  for (int i = 0; i < kBoxCount; ++i) {
    boxes.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_dynamic_friction_box_" + std::to_string(i),
        Eigen::Vector2d(1.0, 0.5),
        1.0,
        Eigen::Vector3d(-30.0 + 2.0 * static_cast<double>(i), 0.75, 0.0),
        false,
        Eigen::Vector3d(10.0, 0.0, 0.0),
        maxFriction - static_cast<double>(i) / 10.0 * maxFriction));
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dStaticFrictionWorld()
{
  constexpr int kBoxCount = 11;
  constexpr double kRampAngle = 3.14159 / 6.0;
  const Eigen::Quaterniond rampOrientation(
      Eigen::AngleAxisd(kRampAngle, Eigen::Vector3d::UnitZ()));

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_static_friction_ground",
      Eigen::Vector2d(100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true,
      Eigen::Vector3d::Zero(),
      1.0,
      rampOrientation);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(kBoxCount);
  for (int i = 0; i < kBoxCount; ++i) {
    boxes.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_static_friction_box_" + std::to_string(i),
        Eigen::Vector2d(5.0, 0.5),
        1.0,
        Eigen::Vector3d(0.0, static_cast<double>(i) + 1.0, 0.0),
        false,
        Eigen::Vector3d::Zero(),
        1.0,
        rampOrientation));
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dPyramidWorld()
{
  constexpr int kPyramidSize = 20;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_pyramid_ground",
      Eigen::Vector2d(100.0, 0.5),
      0.0,
      Eigen::Vector3d(0.0, -2.0, 0.0),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(kPyramidSize * (kPyramidSize + 1) / 2);
  for (int row = 0; row < kPyramidSize; ++row) {
    for (int column = 0; column < kPyramidSize - row; ++column) {
      boxes.push_back(addAvbdDemo2dSourceBox(
          *world,
          "demo2d_pyramid_box_" + std::to_string(column) + "_"
              + std::to_string(row),
          Eigen::Vector2d(1.0, 0.5),
          1.0,
          Eigen::Vector3d(
              static_cast<double>(column) * 1.1 + static_cast<double>(row) * 0.5
                  - static_cast<double>(kPyramidSize) / 2.0,
              static_cast<double>(row) * 0.85,
              0.0)));
    }
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dCardsWorld()
{
  constexpr int kLevels = 5;
  constexpr double kCardHeight = 0.2 * 2.0;
  constexpr double kCardThickness = 0.001 * 2.0;
  constexpr double kAnglePositive = 25.0 * 3.14159 / 180.0;
  constexpr double kAngleNegative = -25.0 * 3.14159 / 180.0;
  constexpr double kAngleHorizontal = 0.5 * 3.14159;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_cards_ground",
      Eigen::Vector2d(80.0, 4.0),
      0.0,
      Eigen::Vector3d(0.0, -2.0, 0.0),
      true,
      Eigen::Vector3d::Zero(),
      0.7);

  std::vector<sx::RigidBody> cards;
  cards.reserve(40);
  int remaining = kLevels;
  int level = 0;
  double x0 = 0.0;
  double y = kCardHeight * 0.5 - 0.02;
  while (remaining > 0) {
    double x = x0;
    for (int i = 0; i < remaining; ++i) {
      if (i != remaining - 1) {
        const Eigen::Quaterniond orientation(
            Eigen::AngleAxisd(kAngleHorizontal, Eigen::Vector3d::UnitZ()));
        cards.push_back(addAvbdDemo2dSourceBox(
            *world,
            "demo2d_cards_horizontal_" + std::to_string(level) + "_"
                + std::to_string(i),
            Eigen::Vector2d(kCardThickness, kCardHeight),
            1.0,
            Eigen::Vector3d(x + 0.25, y + kCardHeight * 0.5 - 0.02, 0.0),
            false,
            Eigen::Vector3d::Zero(),
            0.7,
            orientation));
      }

      const Eigen::Quaterniond negativeOrientation(
          Eigen::AngleAxisd(kAngleNegative, Eigen::Vector3d::UnitZ()));
      cards.push_back(addAvbdDemo2dSourceBox(
          *world,
          "demo2d_cards_negative_" + std::to_string(level) + "_"
              + std::to_string(i),
          Eigen::Vector2d(kCardThickness, kCardHeight),
          1.0,
          Eigen::Vector3d(x, y, 0.0),
          false,
          Eigen::Vector3d::Zero(),
          0.7,
          negativeOrientation));

      x += 0.175;

      const Eigen::Quaterniond positiveOrientation(
          Eigen::AngleAxisd(kAnglePositive, Eigen::Vector3d::UnitZ()));
      cards.push_back(addAvbdDemo2dSourceBox(
          *world,
          "demo2d_cards_positive_" + std::to_string(level) + "_"
              + std::to_string(i),
          Eigen::Vector2d(kCardThickness, kCardHeight),
          1.0,
          Eigen::Vector3d(x, y, 0.0),
          false,
          Eigen::Vector3d::Zero(),
          0.7,
          positiveOrientation));

      x += 0.175;
    }

    y += kCardHeight - 0.04;
    x0 += 0.175;
    --remaining;
    ++level;
  }

  benchmark::DoNotOptimize(cards.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dStackWorld()
{
  constexpr int kBoxCount = 20;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_stack_ground",
      Eigen::Vector2d(100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(kBoxCount);
  for (int i = 0; i < kBoxCount; ++i) {
    boxes.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_stack_box_" + std::to_string(i),
        Eigen::Vector2d(1.0, 1.0),
        1.0,
        Eigen::Vector3d(0.0, static_cast<double>(i) * 2.0 + 1.0, 0.0)));
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dStackRatioWorld()
{
  constexpr int kBoxCount = 6;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_stack_ratio_ground",
      Eigen::Vector2d(100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(kBoxCount);
  for (int i = 0, y = 1, size = 1; i < kBoxCount; ++i) {
    boxes.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_stack_ratio_box_" + std::to_string(i),
        Eigen::Vector2d(static_cast<double>(size), static_cast<double>(size)),
        1.0,
        Eigen::Vector3d(0.0, static_cast<double>(y), 0.0)));
    y += size * 3 / 2;
    size *= 2;
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dRodWorld()
{
  constexpr int kRodLinks = 20;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kRodLinks);
  joints.reserve(kRodLinks - 1);
  for (int i = 0; i < kRodLinks; ++i) {
    links.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_rod_link_" + std::to_string(i),
        Eigen::Vector2d(1.0, 0.5),
        i == 0 ? 0.0 : 1.0,
        Eigen::Vector3d(static_cast<double>(i), 10.0, 0.0),
        i == 0));
  }

  for (int i = 0; i + 1 < kRodLinks; ++i) {
    joints.push_back(addFixedJoint(
        *world,
        "demo2d_rod_fixed_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)]));
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dJointGridWorld()
{
  constexpr int kGridWidth = 25;
  constexpr int kGridHeight = 25;
  constexpr int kGridCells = kGridWidth * kGridHeight;
  constexpr int kGridJoints
      = (kGridWidth - 1) * kGridHeight + kGridWidth * (kGridHeight - 1);

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  std::vector<sx::RigidBody> cells;
  std::vector<sx::Joint> joints;
  cells.reserve(kGridCells);
  joints.reserve(kGridJoints);

  auto cellIndex = [](const int x, const int y) {
    return static_cast<std::size_t>(x * 25 + y);
  };

  for (int x = 0; x < kGridWidth; ++x) {
    for (int y = 0; y < kGridHeight; ++y) {
      const bool isStatic
          = y == kGridHeight - 1 && (x == 0 || x == kGridWidth - 1);
      cells.push_back(addAvbdDemo2dSourceBox(
          *world,
          "demo2d_joint_grid_cell_" + std::to_string(x) + "_"
              + std::to_string(y),
          Eigen::Vector2d(1.0, 1.0),
          isStatic ? 0.0 : 1.0,
          Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), 0.0),
          isStatic));
    }
  }

  for (int x = 1; x < kGridWidth; ++x) {
    for (int y = 0; y < kGridHeight; ++y) {
      joints.push_back(addFixedJoint(
          *world,
          "demo2d_joint_grid_fixed_h_" + std::to_string(x - 1) + "_"
              + std::to_string(y),
          cells[cellIndex(x - 1, y)],
          cells[cellIndex(x, y)]));
    }
  }
  for (int x = 0; x < kGridWidth; ++x) {
    for (int y = 1; y < kGridHeight; ++y) {
      joints.push_back(addFixedJoint(
          *world,
          "demo2d_joint_grid_fixed_v_" + std::to_string(x) + "_"
              + std::to_string(y - 1),
          cells[cellIndex(x, y - 1)],
          cells[cellIndex(x, y)]));
    }
  }

  for (int x = 0; x + 1 < kGridWidth; ++x) {
    for (int y = 0; y + 1 < kGridHeight; ++y) {
      world->setCollisionPairIgnored(
          cells[cellIndex(x, y)], cells[cellIndex(x + 1, y + 1)]);
      world->setCollisionPairIgnored(
          cells[cellIndex(x + 1, y)], cells[cellIndex(x, y + 1)]);
    }
  }

  benchmark::DoNotOptimize(cells.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dSoftBodyWorld()
{
  constexpr int kGridWidth = 15;
  constexpr int kGridHeight = 5;
  constexpr int kGridStacks = 2;
  constexpr int kGridCells = kGridStacks * kGridWidth * kGridHeight;
  constexpr int kGridJoints
      = kGridStacks
        * ((kGridWidth - 1) * kGridHeight + kGridWidth * (kGridHeight - 1));
  constexpr double kLinearStiffness = 1000.0;
  constexpr double kAngularStiffness = 100.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_soft_body_ground",
      Eigen::Vector2d(100.0, 0.5),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> cells;
  std::vector<sx::Joint> joints;
  cells.reserve(kGridCells);
  joints.reserve(kGridJoints);

  auto cellIndex = [](const int stack, const int x, const int y) {
    return static_cast<std::size_t>((stack * 15 * 5) + (x * 5) + y);
  };

  for (int stack = 0; stack < kGridStacks; ++stack) {
    for (int x = 0; x < kGridWidth; ++x) {
      for (int y = 0; y < kGridHeight; ++y) {
        cells.push_back(addAvbdDemo2dSourceBox(
            *world,
            "demo2d_soft_body_cell_" + std::to_string(stack) + "_"
                + std::to_string(x) + "_" + std::to_string(y),
            Eigen::Vector2d(1.0, 1.0),
            1.0,
            Eigen::Vector3d(
                static_cast<double>(x),
                static_cast<double>(y) + 10.0 * static_cast<double>(stack)
                    + 5.0,
                0.0)));
      }
    }
  }

  auto addFiniteFixedJoint = [&](const std::string& name,
                                 const sx::RigidBody& parent,
                                 const sx::RigidBody& child) {
    auto joint = addFixedJoint(*world, name, parent, child);
    setProjectionStiffness(joint, kLinearStiffness, kAngularStiffness);
    joints.push_back(joint);
  };

  for (int stack = 0; stack < kGridStacks; ++stack) {
    for (int x = 1; x < kGridWidth; ++x) {
      for (int y = 0; y < kGridHeight; ++y) {
        addFiniteFixedJoint(
            "demo2d_soft_body_fixed_h_" + std::to_string(stack) + "_"
                + std::to_string(x - 1) + "_" + std::to_string(y),
            cells[cellIndex(stack, x - 1, y)],
            cells[cellIndex(stack, x, y)]);
      }
    }
    for (int x = 0; x < kGridWidth; ++x) {
      for (int y = 1; y < kGridHeight; ++y) {
        addFiniteFixedJoint(
            "demo2d_soft_body_fixed_v_" + std::to_string(stack) + "_"
                + std::to_string(x) + "_" + std::to_string(y - 1),
            cells[cellIndex(stack, x, y - 1)],
            cells[cellIndex(stack, x, y)]);
      }
    }
  }

  for (int stack = 0; stack < kGridStacks; ++stack) {
    for (int x = 0; x + 1 < kGridWidth; ++x) {
      for (int y = 0; y + 1 < kGridHeight; ++y) {
        world->setCollisionPairIgnored(
            cells[cellIndex(stack, x, y)],
            cells[cellIndex(stack, x + 1, y + 1)]);
        world->setCollisionPairIgnored(
            cells[cellIndex(stack, x + 1, y)],
            cells[cellIndex(stack, x, y + 1)]);
      }
    }
  }

  benchmark::DoNotOptimize(cells.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dRopeWorld()
{
  constexpr int kRopeLinks = 20;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kRopeLinks);
  joints.reserve(kRopeLinks - 1);
  for (int i = 0; i < kRopeLinks; ++i) {
    links.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_rope_link_" + std::to_string(i),
        Eigen::Vector2d(1.0, 0.5),
        i == 0 ? 0.0 : 1.0,
        Eigen::Vector3d(static_cast<double>(i), 10.0, 0.0),
        i == 0));
  }

  for (int i = 0; i + 1 < kRopeLinks; ++i) {
    joints.push_back(addSphericalJoint(
        *world,
        "demo2d_rope_point_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(-0.5, 0.0, 0.0)));
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dSpringWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  auto anchor = addAvbdDemo2dSourceBox(
      *world,
      "demo2d_spring_anchor",
      Eigen::Vector2d(1.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);
  auto block = addAvbdDemo2dSourceBox(
      *world,
      "demo2d_spring_block",
      Eigen::Vector2d(4.0, 4.0),
      1.0,
      Eigen::Vector3d(0.0, -8.0, 0.0));

  world->addRigidBodyDistanceSpring(
      "demo2d_spring_radial",
      anchor,
      block,
      4.0,
      100.0,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  world->setCollisionPairIgnored(anchor, block);
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dSpringRatioWorld()
{
  constexpr int kLinks = 8;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  std::vector<sx::RigidBody> links;
  links.reserve(kLinks);
  for (int i = 0; i < kLinks; ++i) {
    const bool isStatic = i == 0 || i == kLinks - 1;
    links.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_spring_ratio_link_" + std::to_string(i),
        Eigen::Vector2d(1.0, 0.5),
        isStatic ? 0.0 : 1.0,
        Eigen::Vector3d(4.0 * static_cast<double>(i), 10.0, 0.0),
        isStatic));
  }

  for (int i = 0; i + 1 < kLinks; ++i) {
    const int childIndex = i + 1;
    const double stiffness = childIndex % 2 == 0 ? 1.0e3 : 1.0e6;
    world->addRigidBodyDistanceSpring(
        "demo2d_spring_ratio_radial_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        0.1,
        stiffness,
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(-0.5, 0.0, 0.0));
    world->setCollisionPairIgnored(
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)]);
  }

  benchmark::DoNotOptimize(links.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dNetWorld()
{
  constexpr int kNetLinks = 40;
  constexpr int kFallingColumns = kNetLinks / 4;
  constexpr int kFallingRows = kNetLinks / 8;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_net_ground",
      Eigen::Vector2d(100.0, 0.5),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kNetLinks);
  joints.reserve(kNetLinks - 1);
  for (int i = 0; i < kNetLinks; ++i) {
    const bool isStatic = i == 0 || i == kNetLinks - 1;
    links.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_net_link_" + std::to_string(i),
        Eigen::Vector2d(1.0, 0.5),
        isStatic ? 0.0 : 1.0,
        Eigen::Vector3d(static_cast<double>(i) - kNetLinks / 2.0, 10.0, 0.0),
        isStatic));
  }

  for (int i = 0; i + 1 < kNetLinks; ++i) {
    joints.push_back(addSphericalJoint(
        *world,
        "demo2d_net_point_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(-0.5, 0.0, 0.0)));
  }

  std::vector<sx::RigidBody> fallingBlocks;
  fallingBlocks.reserve(kFallingColumns * kFallingRows);
  for (int x = 0; x < kFallingColumns; ++x) {
    for (int y = 0; y < kFallingRows; ++y) {
      fallingBlocks.push_back(addAvbdDemo2dSourceBox(
          *world,
          "demo2d_net_falling_block_" + std::to_string(x) + "_"
              + std::to_string(y),
          Eigen::Vector2d(1.0, 1.0),
          1.0,
          Eigen::Vector3d(
              static_cast<double>(x) - kNetLinks / 8.0,
              static_cast<double>(y) + 15.0,
              0.0)));
    }
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  benchmark::DoNotOptimize(fallingBlocks.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dHeavyRopeWorld()
{
  constexpr int kRopeLinks = 20;
  constexpr double kHeavySize = 30.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kRopeLinks);
  joints.reserve(kRopeLinks - 1);
  for (int i = 0; i < kRopeLinks; ++i) {
    const bool isHeavy = i == kRopeLinks - 1;
    links.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_heavy_rope_link_" + std::to_string(i),
        isHeavy ? Eigen::Vector2d(kHeavySize, kHeavySize)
                : Eigen::Vector2d(1.0, 0.5),
        i == 0 ? 0.0 : 1.0,
        Eigen::Vector3d(
            static_cast<double>(i) + (isHeavy ? kHeavySize * 0.5 : 0.0),
            10.0,
            0.0),
        i == 0));
  }

  for (int i = 0; i + 1 < kRopeLinks; ++i) {
    const bool childIsHeavy = i == kRopeLinks - 2;
    joints.push_back(addSphericalJoint(
        *world,
        "demo2d_heavy_rope_point_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(childIsHeavy ? -kHeavySize * 0.5 : -0.5, 0.0, 0.0)));
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dHangingRopeWorld()
{
  constexpr int kRopeLinks = 50;
  constexpr double kHeavySize = 10.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kRopeLinks);
  joints.reserve(kRopeLinks - 1);
  for (int i = 0; i < kRopeLinks; ++i) {
    const bool isHeavy = i == kRopeLinks - 1;
    links.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_hanging_rope_link_" + std::to_string(i),
        isHeavy ? Eigen::Vector2d(kHeavySize, kHeavySize)
                : Eigen::Vector2d(0.5, 1.0),
        i == 0 ? 0.0 : 1.0,
        Eigen::Vector3d(
            0.0,
            10.0 - static_cast<double>(i) - (isHeavy ? kHeavySize * 0.5 : 0.0),
            0.0),
        i == 0));
  }

  for (int i = 0; i + 1 < kRopeLinks; ++i) {
    const bool childIsHeavy = i == kRopeLinks - 2;
    joints.push_back(addSphericalJoint(
        *world,
        "demo2d_hanging_rope_point_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(0.0, -0.5, 0.0),
        Eigen::Vector3d(0.0, childIsHeavy ? kHeavySize * 0.5 : 0.5, 0.0)));
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo2dFractureWorld()
{
  constexpr int kBreakableJoints = 10;
  constexpr int kChainLinks = kBreakableJoints + 1;
  constexpr int kFallingBlocks = 15;
  constexpr double kBreakForce = 500.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -10.0, 0.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo2dSourceBox(
      *world,
      "demo2d_fracture_ground",
      Eigen::Vector2d(100.0, 0.5),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> chain;
  chain.reserve(kChainLinks);
  for (int i = 0; i < kChainLinks; ++i) {
    chain.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_fracture_chain_" + std::to_string(i),
        Eigen::Vector2d(1.0, 0.5),
        1.0,
        Eigen::Vector3d(
            static_cast<double>(i) - kBreakableJoints / 2.0, 6.0, 0.0)));
  }

  std::vector<sx::Joint> joints;
  joints.reserve(kBreakableJoints);
  for (int i = 0; i < kBreakableJoints; ++i) {
    auto joint = addFixedJoint(
        *world,
        "demo2d_fracture_joint_" + std::to_string(i),
        chain[static_cast<std::size_t>(i)],
        chain[static_cast<std::size_t>(i + 1)]);
    joint.setBreakForce(kBreakForce);
    joints.push_back(joint);
  }

  std::vector<sx::RigidBody> obstacles;
  obstacles.reserve(2 + kFallingBlocks);
  obstacles.push_back(addAvbdDemo2dSourceBox(
      *world,
      "demo2d_fracture_left_support",
      Eigen::Vector2d(1.0, 5.0),
      1.0,
      Eigen::Vector3d(-kBreakableJoints / 2.0, 2.5, 0.0)));
  obstacles.push_back(addAvbdDemo2dSourceBox(
      *world,
      "demo2d_fracture_right_support",
      Eigen::Vector2d(1.0, 5.0),
      1.0,
      Eigen::Vector3d(kBreakableJoints / 2.0, 2.5, 0.0)));

  for (int i = 0; i < kFallingBlocks; ++i) {
    obstacles.push_back(addAvbdDemo2dSourceBox(
        *world,
        "demo2d_fracture_falling_block_" + std::to_string(i),
        Eigen::Vector2d(2.0, 1.0),
        1.0,
        Eigen::Vector3d(0.0, static_cast<double>(i) * 2.0 + 8.0, 0.0)));
  }

  benchmark::DoNotOptimize(chain.data());
  benchmark::DoNotOptimize(joints.data());
  benchmark::DoNotOptimize(obstacles.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dGroundWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_ground_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);
  auto box = addAvbdDemo3dSourceBox(
      *world,
      "demo3d_ground_box",
      Eigen::Vector3d(1.0, 1.0, 1.0),
      1.0,
      Eigen::Vector3d(0.0, 0.0, 4.0));

  benchmark::DoNotOptimize(box);
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dDynamicFrictionWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_dynamic_friction_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(11);
  for (int i = 0; i <= 10; ++i) {
    auto box = addAvbdDemo3dSourceBox(
        *world,
        "demo3d_dynamic_friction_box_" + std::to_string(i),
        Eigen::Vector3d(1.0, 1.0, 0.5),
        1.0,
        Eigen::Vector3d(0.0, -30.0 + 2.0 * static_cast<double>(i), 0.75),
        false,
        Eigen::Vector3d(10.0, 0.0, 0.0));
    box.setFriction(5.0 - static_cast<double>(i) / 10.0 * 5.0);
    boxes.push_back(box);
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dStaticFrictionWorld()
{
  constexpr double kRampAngle = std::numbers::pi / 6.0;
  const Eigen::AngleAxisd rampRotation(kRampAngle, Eigen::Vector3d::UnitY());
  const Eigen::Vector3d rampPosition(0.0, 0.0, 3.0);
  const Eigen::Vector3d rampTangent = rampRotation * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d rampNormal = rampRotation * Eigen::Vector3d::UnitZ();

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_static_friction_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  sx::RigidBodyOptions rampOptions;
  rampOptions.position = rampPosition;
  rampOptions.orientation = Eigen::Quaterniond(rampRotation);
  rampOptions.isStatic = true;
  auto ramp = world->addRigidBody("demo3d_static_friction_ramp", rampOptions);
  ramp.setFriction(1.0);
  ramp.setCollisionShape(
      sx::CollisionShape::makeBox(0.5 * Eigen::Vector3d(40.0, 24.0, 1.0)));

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(11);
  for (int i = 0; i <= 10; ++i) {
    const double friction = static_cast<double>(i) / 10.0 * 0.25 + 0.25;
    const double y = -10.0 + 2.0 * static_cast<double>(i);
    auto box = addAvbdDemo3dSourceBox(
        *world,
        "demo3d_static_friction_box_" + std::to_string(i),
        Eigen::Vector3d(1.0, 1.0, 1.0),
        1.0,
        rampPosition + rampTangent * -12.0 + Eigen::Vector3d(0.0, y, 0.0)
            + rampNormal * 1.05);
    box.setFriction(friction);
    boxes.push_back(box);
  }

  benchmark::DoNotOptimize(ramp);
  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dPyramidWorld()
{
  constexpr int kPyramidSize = 16;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_pyramid_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d(0.0, 0.0, -0.5),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(kPyramidSize * (kPyramidSize + 1) / 2);
  for (int row = 0; row < kPyramidSize; ++row) {
    for (int column = 0; column < kPyramidSize - row; ++column) {
      auto box = addAvbdDemo3dSourceBox(
          *world,
          "demo3d_pyramid_box_" + std::to_string(column) + "_"
              + std::to_string(row),
          Eigen::Vector3d(1.0, 0.5, 0.5),
          1.0,
          Eigen::Vector3d(
              static_cast<double>(column) * 1.01
                  + static_cast<double>(row) * 0.5
                  - static_cast<double>(kPyramidSize) / 2.0,
              0.0,
              static_cast<double>(row) * 0.85 + 0.5));
      box.setFriction(0.5);
      boxes.push_back(box);
    }
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dRopeWorld()
{
  constexpr int kRopeLinks = 20;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_rope_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d(0.0, 0.0, -20.0),
      true);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kRopeLinks);
  joints.reserve(kRopeLinks - 1);
  for (int i = 0; i < kRopeLinks; ++i) {
    links.push_back(addAvbdDemo3dSourceBox(
        *world,
        "demo3d_rope_link_" + std::to_string(i),
        Eigen::Vector3d(1.0, 0.5, 0.5),
        i == 0 ? 0.0 : 1.0,
        Eigen::Vector3d(static_cast<double>(i), 0.0, 10.0),
        i == 0));
  }

  for (int i = 0; i + 1 < kRopeLinks; ++i) {
    joints.push_back(addSphericalJoint(
        *world,
        "demo3d_rope_point_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(-0.5, 0.0, 0.0)));
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dHeavyRopeWorld()
{
  constexpr int kRopeLinks = 20;
  constexpr double kHeavySize = 5.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_heavy_rope_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d(0.0, 0.0, -20.0),
      true);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(kRopeLinks);
  joints.reserve(kRopeLinks - 1);
  for (int i = 0; i < kRopeLinks; ++i) {
    const bool isHeavy = i == kRopeLinks - 1;
    links.push_back(addAvbdDemo3dSourceBox(
        *world,
        "demo3d_heavy_rope_link_" + std::to_string(i),
        isHeavy ? Eigen::Vector3d(kHeavySize, kHeavySize, kHeavySize)
                : Eigen::Vector3d(1.0, 0.5, 0.5),
        i == 0 ? 0.0 : 1.0,
        Eigen::Vector3d(
            static_cast<double>(i) + (isHeavy ? kHeavySize * 0.5 : 0.0),
            0.0,
            10.0),
        i == 0));
  }

  for (int i = 0; i + 1 < kRopeLinks; ++i) {
    const bool childIsHeavy = i == kRopeLinks - 2;
    joints.push_back(addSphericalJoint(
        *world,
        "demo3d_heavy_rope_point_joint_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(childIsHeavy ? -kHeavySize * 0.5 : -0.5, 0.0, 0.0)));
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dSpringWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_spring_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);
  auto anchor = addAvbdDemo3dSourceBox(
      *world,
      "demo3d_spring_anchor",
      Eigen::Vector3d(1.0, 1.0, 1.0),
      0.0,
      Eigen::Vector3d(0.0, 0.0, 14.0),
      true);
  auto block = addAvbdDemo3dSourceBox(
      *world,
      "demo3d_spring_block",
      Eigen::Vector3d(2.0, 2.0, 2.0),
      1.0,
      Eigen::Vector3d(0.0, 0.0, 8.0));

  world->addRigidBodyDistanceSpring(
      "demo3d_spring_radial",
      anchor,
      block,
      4.0,
      100.0,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  world->setCollisionPairIgnored(anchor, block);
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dSpringRatioWorld()
{
  constexpr int kLinks = 8;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_spring_ratio_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d(0.0, 0.0, -10.0),
      true);

  std::vector<sx::RigidBody> links;
  links.reserve(kLinks);
  for (int i = 0; i < kLinks; ++i) {
    const bool isStatic = i == 0 || i == kLinks - 1;
    links.push_back(addAvbdDemo3dSourceBox(
        *world,
        "demo3d_spring_ratio_link_" + std::to_string(i),
        Eigen::Vector3d(1.0, 0.75, 0.75),
        isStatic ? 0.0 : 1.0,
        Eigen::Vector3d(
            (static_cast<double>(i) - (kLinks - 1) * 0.5) * 3.0, 0.0, 12.0),
        isStatic));
  }

  for (int i = 0; i + 1 < kLinks; ++i) {
    const int childIndex = i + 1;
    const double stiffness = childIndex % 2 == 0 ? 10.0 : 1.0e4;
    world->addRigidBodyDistanceSpring(
        "demo3d_spring_ratio_radial_" + std::to_string(i),
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)],
        3.0,
        stiffness,
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(-0.5, 0.0, 0.0));
    world->setCollisionPairIgnored(
        links[static_cast<std::size_t>(i)],
        links[static_cast<std::size_t>(i + 1)]);
  }

  benchmark::DoNotOptimize(links.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dStackWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_stack_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(10);
  for (int i = 0; i < 10; ++i) {
    auto box = addAvbdDemo3dSourceBox(
        *world,
        "demo3d_stack_box_" + std::to_string(i),
        Eigen::Vector3d(1.0, 1.0, 1.0),
        1.0,
        Eigen::Vector3d(0.0, 0.0, static_cast<double>(i) * 1.5 + 1.0));
    box.setFriction(0.5);
    boxes.push_back(box);
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dStackRatioWorld()
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kGroundThickness = 1.0;
  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_stack_ratio_ground",
      Eigen::Vector3d(100.0, 100.0, kGroundThickness),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> boxes;
  boxes.reserve(4);
  double topZ = kGroundThickness * 0.5;
  double size = 1.0;
  for (int i = 0; i < 4; ++i) {
    const double half = size * 0.5;
    const double centerZ = topZ + half;
    auto box = addAvbdDemo3dSourceBox(
        *world,
        "demo3d_stack_ratio_box_" + std::to_string(i),
        Eigen::Vector3d(size, size, size),
        1.0,
        Eigen::Vector3d(0.0, 0.0, centerZ));
    box.setFriction(0.5);
    boxes.push_back(box);
    topZ = centerZ + half;
    size *= 2.0;
  }

  benchmark::DoNotOptimize(boxes.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dSoftBodyWorld()
{
  constexpr int kGridWidth = 4;
  constexpr int kGridDepth = 4;
  constexpr int kGridHeight = 4;
  constexpr int kGridStacks = 3;
  constexpr int kGridCells
      = kGridStacks * kGridWidth * kGridDepth * kGridHeight;
  constexpr int kGridJoints = kGridStacks
                              * ((kGridWidth - 1) * kGridDepth * kGridHeight
                                 + kGridWidth * (kGridDepth - 1) * kGridHeight
                                 + kGridWidth * kGridDepth * (kGridHeight - 1));
  constexpr double kCellSize = 0.8;
  constexpr double kBaseZ = 8.0;
  constexpr double kStackGap = 2.0;
  constexpr double kLinearStiffness = 1000.0;
  constexpr double kAngularStiffness = 250.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_soft_body_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> cells;
  std::vector<sx::Joint> joints;
  cells.reserve(kGridCells);
  joints.reserve(kGridJoints);

  auto cellIndex = [](const int stack, const int x, const int y, const int z) {
    return static_cast<std::size_t>(((stack * 4 + x) * 4 + y) * 4 + z);
  };
  auto cellPosition
      = [=](const int stack, const int x, const int y, const int z) {
          const double stackZ
              = static_cast<double>(stack) * (4.0 * kCellSize + kStackGap);
          return Eigen::Vector3d(
              (static_cast<double>(x) - 1.5) * kCellSize,
              (static_cast<double>(y) - 1.5) * kCellSize,
              kBaseZ + stackZ + static_cast<double>(z) * kCellSize);
        };

  for (int stack = 0; stack < kGridStacks; ++stack) {
    for (int x = 0; x < kGridWidth; ++x) {
      for (int y = 0; y < kGridDepth; ++y) {
        for (int z = 0; z < kGridHeight; ++z) {
          cells.push_back(addAvbdDemo3dSourceBox(
              *world,
              "demo3d_soft_body_cell_" + std::to_string(stack) + "_"
                  + std::to_string(x) + "_" + std::to_string(y) + "_"
                  + std::to_string(z),
              Eigen::Vector3d(kCellSize, kCellSize, kCellSize),
              1.0,
              cellPosition(stack, x, y, z)));
        }
      }
    }
  }

  auto addFiniteFixedJoint = [&](const std::string& name,
                                 const sx::RigidBody& parent,
                                 const sx::RigidBody& child) {
    auto joint = addFixedJoint(*world, name, parent, child);
    setProjectionStiffness(joint, kLinearStiffness, kAngularStiffness);
    joints.push_back(joint);
  };

  for (int stack = 0; stack < kGridStacks; ++stack) {
    for (int x = 1; x < kGridWidth; ++x) {
      for (int y = 0; y < kGridDepth; ++y) {
        for (int z = 0; z < kGridHeight; ++z) {
          addFiniteFixedJoint(
              "demo3d_soft_body_fixed_x_" + std::to_string(stack) + "_"
                  + std::to_string(x - 1) + "_" + std::to_string(y) + "_"
                  + std::to_string(z),
              cells[cellIndex(stack, x - 1, y, z)],
              cells[cellIndex(stack, x, y, z)]);
        }
      }
    }
    for (int x = 0; x < kGridWidth; ++x) {
      for (int y = 1; y < kGridDepth; ++y) {
        for (int z = 0; z < kGridHeight; ++z) {
          addFiniteFixedJoint(
              "demo3d_soft_body_fixed_y_" + std::to_string(stack) + "_"
                  + std::to_string(x) + "_" + std::to_string(y - 1) + "_"
                  + std::to_string(z),
              cells[cellIndex(stack, x, y - 1, z)],
              cells[cellIndex(stack, x, y, z)]);
        }
      }
    }
    for (int x = 0; x < kGridWidth; ++x) {
      for (int y = 0; y < kGridDepth; ++y) {
        for (int z = 1; z < kGridHeight; ++z) {
          addFiniteFixedJoint(
              "demo3d_soft_body_fixed_z_" + std::to_string(stack) + "_"
                  + std::to_string(x) + "_" + std::to_string(y) + "_"
                  + std::to_string(z - 1),
              cells[cellIndex(stack, x, y, z - 1)],
              cells[cellIndex(stack, x, y, z)]);
        }
      }
    }
  }

  for (int stack = 0; stack < kGridStacks; ++stack) {
    for (int x = 1; x < kGridWidth; ++x) {
      for (int y = 0; y < kGridDepth; ++y) {
        for (int z = 1; z < kGridHeight; ++z) {
          world->setCollisionPairIgnored(
              cells[cellIndex(stack, x - 1, y, z - 1)],
              cells[cellIndex(stack, x, y, z)]);
          world->setCollisionPairIgnored(
              cells[cellIndex(stack, x, y, z - 1)],
              cells[cellIndex(stack, x - 1, y, z)]);
        }
      }
    }
    for (int x = 0; x < kGridWidth; ++x) {
      for (int y = 1; y < kGridDepth; ++y) {
        for (int z = 1; z < kGridHeight; ++z) {
          world->setCollisionPairIgnored(
              cells[cellIndex(stack, x, y - 1, z - 1)],
              cells[cellIndex(stack, x, y, z)]);
          world->setCollisionPairIgnored(
              cells[cellIndex(stack, x, y, z - 1)],
              cells[cellIndex(stack, x, y - 1, z)]);
        }
      }
    }
    for (int x = 1; x < kGridWidth; ++x) {
      for (int y = 1; y < kGridDepth; ++y) {
        for (int z = 0; z < kGridHeight; ++z) {
          world->setCollisionPairIgnored(
              cells[cellIndex(stack, x - 1, y - 1, z)],
              cells[cellIndex(stack, x, y, z)]);
          world->setCollisionPairIgnored(
              cells[cellIndex(stack, x, y - 1, z)],
              cells[cellIndex(stack, x - 1, y, z)]);
        }
      }
    }
  }

  benchmark::DoNotOptimize(cells.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dBridgeWorld()
{
  constexpr int kPlanks = 40;
  constexpr int kLoadX = kPlanks / 4;
  constexpr int kLoadZ = kPlanks / 8;
  constexpr double kPlankLength = 1.0;
  constexpr double kPlankWidth = 4.0;
  constexpr double kPlankHeight = 0.5;
  constexpr double kHalfLength = kPlankLength * 0.5;
  constexpr double kHalfWidth = kPlankWidth * 0.5;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_bridge_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> planks;
  std::vector<sx::RigidBody> loadBoxes;
  std::vector<sx::Joint> joints;
  planks.reserve(kPlanks);
  loadBoxes.reserve(kLoadX * kLoadZ);
  joints.reserve((kPlanks - 1) * 2);
  for (int i = 0; i < kPlanks; ++i) {
    const bool isStatic = i == 0 || i == kPlanks - 1;
    planks.push_back(addAvbdDemo3dSourceBox(
        *world,
        "demo3d_bridge_plank_" + std::to_string(i),
        Eigen::Vector3d(kPlankLength, kPlankWidth, kPlankHeight),
        isStatic ? 0.0 : 1.0,
        Eigen::Vector3d(static_cast<double>(i) - kPlanks * 0.5, 0.0, 10.0),
        isStatic));
  }

  for (int i = 0; i + 1 < kPlanks; ++i) {
    joints.push_back(addSphericalJoint(
        *world,
        "demo3d_bridge_joint_" + std::to_string(i) + "_0",
        planks[static_cast<std::size_t>(i)],
        planks[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(kHalfLength, kHalfWidth, 0.0),
        Eigen::Vector3d(-kHalfLength, kHalfWidth, 0.0)));
    joints.push_back(addSphericalJoint(
        *world,
        "demo3d_bridge_joint_" + std::to_string(i) + "_1",
        planks[static_cast<std::size_t>(i)],
        planks[static_cast<std::size_t>(i + 1)],
        Eigen::Vector3d(kHalfLength, -kHalfWidth, 0.0),
        Eigen::Vector3d(-kHalfLength, -kHalfWidth, 0.0)));
  }

  for (int x = 0; x < kLoadX; ++x) {
    for (int z = 0; z < kLoadZ; ++z) {
      loadBoxes.push_back(addAvbdDemo3dSourceBox(
          *world,
          "demo3d_bridge_load_" + std::to_string(x) + "_" + std::to_string(z),
          Eigen::Vector3d(1.0, 1.0, 1.0),
          1.0,
          Eigen::Vector3d(
              static_cast<double>(x) - kPlanks / 8.0,
              0.0,
              static_cast<double>(z) + 12.0),
          false));
    }
  }

  benchmark::DoNotOptimize(planks.data());
  benchmark::DoNotOptimize(loadBoxes.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeAvbdDemo3dBreakableWorld()
{
  constexpr int kBreakableJoints = 10;
  constexpr int kChainLinks = kBreakableJoints + 1;
  constexpr int kFallingBlocks = 5;
  constexpr double kBreakForce = 90.0;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -10.0);
  options.timeStep = 1.0 / 60.0;
  auto world = std::make_unique<sx::World>(options);

  addAvbdDemo3dSourceBox(
      *world,
      "demo3d_breakable_ground",
      Eigen::Vector3d(100.0, 100.0, 1.0),
      0.0,
      Eigen::Vector3d::Zero(),
      true);

  std::vector<sx::RigidBody> chain;
  chain.reserve(kChainLinks);
  for (int i = 0; i < kChainLinks; ++i) {
    chain.push_back(addAvbdDemo3dSourceBox(
        *world,
        "demo3d_breakable_chain_" + std::to_string(i),
        Eigen::Vector3d(1.0, 1.0, 0.5),
        1.0,
        Eigen::Vector3d(
            static_cast<double>(i) - kBreakableJoints / 2.0, 0.0, 6.0)));
  }

  std::vector<sx::Joint> joints;
  joints.reserve(kBreakableJoints);
  for (int i = 0; i < kBreakableJoints; ++i) {
    auto joint = addFixedJoint(
        *world,
        "demo3d_breakable_joint_" + std::to_string(i),
        chain[static_cast<std::size_t>(i)],
        chain[static_cast<std::size_t>(i + 1)]);
    joint.setBreakForce(kBreakForce);
    joints.push_back(joint);
  }

  std::vector<sx::RigidBody> fixtures;
  fixtures.reserve(2 + kFallingBlocks);
  fixtures.push_back(addAvbdDemo3dSourceBox(
      *world,
      "demo3d_breakable_left_support",
      Eigen::Vector3d(1.0, 1.0, 5.0),
      0.0,
      Eigen::Vector3d(-kBreakableJoints / 2.0, 0.0, 2.5),
      true));
  fixtures.push_back(addAvbdDemo3dSourceBox(
      *world,
      "demo3d_breakable_right_support",
      Eigen::Vector3d(1.0, 1.0, 5.0),
      0.0,
      Eigen::Vector3d(kBreakableJoints / 2.0, 0.0, 2.5),
      true));

  for (int i = 0; i < kFallingBlocks; ++i) {
    fixtures.push_back(addAvbdDemo3dSourceBox(
        *world,
        "demo3d_breakable_falling_block_" + std::to_string(i),
        Eigen::Vector3d(2.0, 1.0, 1.0),
        1.0,
        Eigen::Vector3d(0.0, 0.0, static_cast<double>(i) * 2.0 + 8.0)));
  }

  benchmark::DoNotOptimize(chain.data());
  benchmark::DoNotOptimize(joints.data());
  benchmark::DoNotOptimize(fixtures.data());
  return world;
}

class Fnv1a64
{
public:
  void updateUint64(std::uint64_t value)
  {
    for (int byte = 0; byte < 8; ++byte) {
      updateByte(static_cast<std::uint8_t>(value & 0xffu));
      value >>= 8u;
    }
  }

  void updateDouble(double value)
  {
    updateUint64(std::bit_cast<std::uint64_t>(value));
  }

  void updateString(std::string_view value)
  {
    updateUint64(static_cast<std::uint64_t>(value.size()));
    for (const char character : value) {
      updateByte(static_cast<std::uint8_t>(character));
    }
  }

  [[nodiscard]] std::uint64_t value() const
  {
    return m_value;
  }

private:
  void updateByte(std::uint8_t value)
  {
    m_value ^= value;
    m_value *= 1099511628211ull;
  }

  std::uint64_t m_value{14695981039346656037ull};
};

struct PaperBreakableWallFixture
{
  std::unique_ptr<sx::World> world;
  std::uint64_t sceneSpecFingerprint{0};
  std::uint64_t effectiveSceneContractFailureMask{0};
  bool effectiveSceneContractPassed{false};
  std::uint64_t mutationAuditFailureMask{0};
  bool mutationAuditPassed{false};
  bool matchesPythonSceneSpecFingerprint{false};
};

constexpr double kPaperBreakableWallStartStiffness = 1e5;
constexpr std::uint64_t kPaperBreakableWallPythonSceneSpecFingerprint
    = 0x8ca3fbfa00c3dce9ull;
constexpr double kPaperBreakableWallTimeStep = 1.0 / 60.0;
constexpr double kPaperBreakableWallGravity = -9.81;
constexpr std::uint64_t kPaperBreakableWallConstraintIterations = 20u;
constexpr std::uint64_t kPaperBreakableWallColumns = 21u;
constexpr std::uint64_t kPaperBreakableWallRows = 12u;
constexpr std::array<double, 3> kPaperBreakableWallBrickSize{0.60, 0.30, 0.25};
constexpr std::array<double, 10> kPaperBreakableWallBodyParameters{
    200.0, 0.50, 0.62, 0.27, 0.02, 0.48, 40.0, 0.30, -5.0, 24.0};
constexpr std::array<std::array<double, 2>, 3> kPaperBreakableWallImpactTargets{
    std::array<double, 2>{-3.10, 1.55},
    std::array<double, 2>{-0.31, 1.75},
    std::array<double, 2>{3.10, 2.35}};
constexpr std::array<double, 3> kPaperBreakableWallGroundSize{16.0, 16.0, 0.50};
constexpr double kPaperBreakableWallGroundFriction = 0.60;
constexpr double kPaperBreakableWallBreakForce = 5000.0;

struct PaperBreakableWallEffectiveScene
{
  double timeStep{0.0};
  double gravity{0.0};
  std::uint64_t constraintIterations{0};
  std::uint64_t columns{0};
  std::uint64_t rows{0};
  std::array<double, 3> brickSize{};
  std::array<double, 10> bodyParameters{};
  std::array<std::array<double, 2>, 3> impactTargets{};
  std::array<double, 3> groundSize{};
  double groundFriction{0.0};
  double breakForce{0.0};
  std::uint64_t contractFailureMask{0};
  bool contractPassed{false};
};

bool paperBreakableWallIsIdentityTransform(const Eigen::Isometry3d& transform)
{
  return transform.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-12);
}

std::optional<Eigen::Vector3d> paperBreakableWallBoxSize(
    const sx::RigidBody& body)
{
  const auto shapes = body.getCollisionShapes();
  if (shapes.size() != 1u || shapes[0].type != sx::CollisionShapeType::Box
      || !paperBreakableWallIsIdentityTransform(shapes[0].localTransform)) {
    return std::nullopt;
  }
  return 2.0 * shapes[0].halfExtents;
}

std::optional<double> paperBreakableWallSphereRadius(const sx::RigidBody& body)
{
  const auto shapes = body.getCollisionShapes();
  if (shapes.size() != 1u || shapes[0].type != sx::CollisionShapeType::Sphere
      || !paperBreakableWallIsIdentityTransform(shapes[0].localTransform)) {
    return std::nullopt;
  }
  return shapes[0].radius;
}

PaperBreakableWallEffectiveScene inspectPaperBreakableWallEffectiveScene(
    const sx::World& world,
    const sx::RigidBody& ground,
    const std::vector<sx::RigidBody>& bricks,
    const std::vector<sx::RigidBody>& balls,
    const std::vector<sx::Joint>& joints,
    const std::vector<std::array<std::uint64_t, 3>>& topologyRecords,
    const std::uint64_t columns,
    const std::uint64_t rows)
{
  PaperBreakableWallEffectiveScene scene;
  scene.timeStep = world.getTimeStep();
  scene.gravity = world.getGravity().z();
  scene.constraintIterations = world.getRigidConstraintOptions().iterations;
  scene.columns = columns;
  scene.rows = rows;

  const auto groundSize = paperBreakableWallBoxSize(ground);
  const auto brickSize
      = bricks.empty() ? std::nullopt : paperBreakableWallBoxSize(bricks[0]);
  const auto ballRadius
      = balls.empty() ? std::nullopt : paperBreakableWallSphereRadius(balls[0]);
  if (!groundSize.has_value() || !brickSize.has_value()
      || !ballRadius.has_value() || bricks.size() < columns + 1u
      || balls.size() != scene.impactTargets.size() || joints.empty()) {
    scene.contractFailureMask = 1u;
    return scene;
  }

  for (int axis = 0; axis < 3; ++axis) {
    scene.groundSize[axis] = (*groundSize)[axis];
    scene.brickSize[axis] = (*brickSize)[axis];
  }
  const double brickVolume = brickSize->prod();
  if (!(brickVolume > 0.0)) {
    return scene;
  }
  scene.bodyParameters = {
      bricks[0].getMass() / brickVolume,
      bricks[0].getFriction(),
      bricks[1].getTransform().translation().x()
          - bricks[0].getTransform().translation().x(),
      bricks[columns].getTransform().translation().z()
          - bricks[0].getTransform().translation().z(),
      bricks[0].getTransform().translation().z() - 0.5 * brickSize->z(),
      *ballRadius,
      balls[0].getMass(),
      balls[0].getFriction(),
      balls[0].getTransform().translation().y(),
      balls[0].getLinearVelocity().y(),
  };
  for (std::size_t index = 0; index < balls.size(); ++index) {
    scene.impactTargets[index]
        = {balls[index].getTransform().translation().x(),
           balls[index].getTransform().translation().z()};
  }
  scene.groundFriction = ground.getFriction();
  scene.breakForce = joints[0].getBreakForce();

  const auto sameScalar = [](double actual, double expected) {
    return std::isfinite(actual) && std::isfinite(expected)
           && std::abs(actual - expected)
                  <= 1e-12
                         * std::max(
                             {1.0, std::abs(actual), std::abs(expected)});
  };
  const auto sameVector
      = [&](const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
          return actual.allFinite() && actual.isApprox(expected, 1e-12);
        };
  const auto identityRotation = [&](const sx::RigidBody& body) {
    return body.getTransform().linear().isApprox(
        Eigen::Matrix3d::Identity(), 1e-12);
  };
  const auto hasDefaultDynamicState = [&](const sx::RigidBody& body) {
    return !body.isKinematic() && sameScalar(body.getRestitution(), 0.0)
           && sameVector(body.getForce(), Eigen::Vector3d::Zero())
           && sameVector(body.getTorque(), Eigen::Vector3d::Zero());
  };
  const bool worldAndGroundValid
      = sameScalar(world.getGravity().x(), 0.0)
        && sameScalar(world.getGravity().y(), 0.0)
        && world.getRigidBodyCount() == 1u + bricks.size() + balls.size()
        && world.getJointCount() == joints.size() && ground.isStatic()
        && hasDefaultDynamicState(ground) && identityRotation(ground)
        && sameVector(
            ground.getTransform().translation(),
            Eigen::Vector3d(0.0, 0.0, -0.5 * groundSize->z()))
        && sameVector(ground.getLinearVelocity(), Eigen::Vector3d::Zero())
        && sameVector(ground.getAngularVelocity(), Eigen::Vector3d::Zero())
        && bricks.size() == columns * rows
        && topologyRecords.size() == joints.size();
  if (!worldAndGroundValid) {
    scene.contractFailureMask |= 2u;
  }

  const double brickDensity = scene.bodyParameters[0];
  const double brickFriction = scene.bodyParameters[1];
  const double spacingX = scene.bodyParameters[2];
  const double spacingZ = scene.bodyParameters[3];
  const double baseClearance = scene.bodyParameters[4];
  const double brickMass = brickVolume * brickDensity;
  const Eigen::Matrix3d brickInertia = fullBoxInertia(*brickSize, brickMass);
  bool bricksValid = true;
  for (std::uint64_t row = 0; row < rows && bricksValid; ++row) {
    for (std::uint64_t column = 0; column < columns && bricksValid; ++column) {
      const sx::RigidBody& brick = bricks[row * columns + column];
      const auto effectiveSize = paperBreakableWallBoxSize(brick);
      const double courseOffset = row % 2u == 0u ? 0.0 : 0.5;
      const Eigen::Vector3d expectedPosition(
          (static_cast<double>(column) - 0.5 * static_cast<double>(columns - 1u)
           + courseOffset)
              * spacingX,
          0.0,
          baseClearance + 0.5 * brickSize->z()
              + static_cast<double>(row) * spacingZ);
      bricksValid
          = effectiveSize.has_value() && sameVector(*effectiveSize, *brickSize)
            && !brick.isStatic() && hasDefaultDynamicState(brick)
            && identityRotation(brick)
            && sameVector(brick.getTransform().translation(), expectedPosition)
            && sameVector(brick.getLinearVelocity(), Eigen::Vector3d::Zero())
            && sameVector(brick.getAngularVelocity(), Eigen::Vector3d::Zero())
            && sameScalar(brick.getMass(), brickMass)
            && brick.getInertia().isApprox(brickInertia, 1e-12)
            && sameScalar(brick.getFriction(), brickFriction);
    }
  }
  if (!bricksValid) {
    scene.contractFailureMask |= 4u;
  }

  const double expectedBallRadius = scene.bodyParameters[5];
  const double expectedBallMass = scene.bodyParameters[6];
  const double expectedBallFriction = scene.bodyParameters[7];
  const double expectedBallStartY = scene.bodyParameters[8];
  const double expectedBallSpeed = scene.bodyParameters[9];
  const Eigen::Matrix3d expectedBallInertia
      = Eigen::Matrix3d::Identity()
        * (2.0 / 5.0 * expectedBallMass * expectedBallRadius
           * expectedBallRadius);
  bool ballsValid = true;
  for (std::size_t index = 0; index < balls.size() && ballsValid; ++index) {
    const sx::RigidBody& ball = balls[index];
    const auto effectiveRadius = paperBreakableWallSphereRadius(ball);
    ballsValid
        = effectiveRadius.has_value()
          && sameScalar(*effectiveRadius, expectedBallRadius)
          && !ball.isStatic() && hasDefaultDynamicState(ball)
          && identityRotation(ball)
          && sameVector(
              ball.getTransform().translation(),
              Eigen::Vector3d(
                  scene.impactTargets[index][0],
                  expectedBallStartY,
                  scene.impactTargets[index][1]))
          && sameVector(
              ball.getLinearVelocity(),
              Eigen::Vector3d(0.0, expectedBallSpeed, 0.0))
          && sameVector(ball.getAngularVelocity(), Eigen::Vector3d::Zero())
          && sameScalar(ball.getMass(), expectedBallMass)
          && ball.getInertia().isApprox(expectedBallInertia, 1e-12)
          && sameScalar(ball.getFriction(), expectedBallFriction);
  }
  if (!ballsValid) {
    scene.contractFailureMask |= 8u;
  }

  std::vector<sx::RigidBody> jointBodies;
  jointBodies.reserve(1u + bricks.size());
  jointBodies.push_back(ground);
  jointBodies.insert(jointBodies.end(), bricks.begin(), bricks.end());
  const auto& registry = sx::detail::registryOf(world);
  bool jointsValid = true;
  for (std::size_t index = 0; index < joints.size() && jointsValid; ++index) {
    const auto& topology = topologyRecords[index];
    const sx::RigidBody parent = joints[index].getParentRigidBody();
    const sx::RigidBody child = joints[index].getChildRigidBody();
    const auto* config = registry.try_get<vbd::AvbdRigidWorldPointJointConfig>(
        sx::detail::toRegistryEntity(joints[index].getEntity()));
    const auto policy = joints[index].getConstraintProjectionPolicy();
    const bool finiteVbdRows
        = world.getRigidBodySolver() == sx::RigidBodySolver::Vbd;
    const Eigen::Vector3d expectedParentAnchor
        = parent.getTransform().inverse() * child.getTransform().translation();
    const Eigen::Matrix3d expectedRelativeRotation
        = parent.getTransform().linear().transpose()
          * child.getTransform().linear();
    jointsValid
        = topology[0] >= 1u && topology[0] <= 3u
          && topology[1] < jointBodies.size()
          && topology[2] < jointBodies.size()
          && joints[index].getType() == sx::JointType::Fixed
          && !joints[index].isBroken()
          && sameScalar(joints[index].getBreakForce(), scene.breakForce)
          && parent.getName() == jointBodies[topology[1]].getName()
          && child.getName() == jointBodies[topology[2]].getName()
          && sameScalar(
              policy.startStiffness, kPaperBreakableWallStartStiffness)
          && (finiteVbdRows ? sameScalar(
                                  policy.linearStiffness,
                                  kPaperBreakableWallStartStiffness)
                            : std::isinf(policy.linearStiffness)
                                  && policy.linearStiffness > 0.0)
          && (finiteVbdRows ? sameScalar(
                                  policy.angularStiffness,
                                  kPaperBreakableWallStartStiffness)
                            : std::isinf(policy.angularStiffness)
                                  && policy.angularStiffness > 0.0)
          && config != nullptr && config->enabled
          && sameVector(config->localAnchorA, expectedParentAnchor)
          && sameVector(config->localAnchorB, Eigen::Vector3d::Zero())
          && config->targetRelativeOrientation.toRotationMatrix().isApprox(
              expectedRelativeRotation, 1e-12)
          && config->linearAxes.isApprox(Eigen::Matrix3d::Identity(), 1e-12)
          && config->angularAxes.isApprox(Eigen::Matrix3d::Identity(), 1e-12)
          && config->linearAxisMask == vbd::kAvbdRigidJointAllAxesMask
          && config->angularAxisMask == vbd::kAvbdRigidJointAllAxesMask
          && std::isinf(config->maxStiffness) && config->maxStiffness > 0.0;
  }
  if (!jointsValid) {
    scene.contractFailureMask |= 16u;
  }

  bool intendedValuesValid
      = sameScalar(scene.timeStep, kPaperBreakableWallTimeStep)
        && sameScalar(scene.gravity, kPaperBreakableWallGravity)
        && scene.constraintIterations == kPaperBreakableWallConstraintIterations
        && scene.columns == kPaperBreakableWallColumns
        && scene.rows == kPaperBreakableWallRows
        && sameScalar(scene.groundFriction, kPaperBreakableWallGroundFriction)
        && sameScalar(scene.breakForce, kPaperBreakableWallBreakForce);
  for (std::size_t index = 0u; index < scene.brickSize.size(); ++index) {
    intendedValuesValid
        = intendedValuesValid
          && sameScalar(
              scene.brickSize[index], kPaperBreakableWallBrickSize[index])
          && sameScalar(
              scene.groundSize[index], kPaperBreakableWallGroundSize[index]);
  }
  for (std::size_t index = 0u; index < scene.bodyParameters.size(); ++index) {
    intendedValuesValid = intendedValuesValid
                          && sameScalar(
                              scene.bodyParameters[index],
                              kPaperBreakableWallBodyParameters[index]);
  }
  for (std::size_t impact = 0u; impact < scene.impactTargets.size(); ++impact) {
    for (std::size_t axis = 0u; axis < scene.impactTargets[impact].size();
         ++axis) {
      intendedValuesValid = intendedValuesValid
                            && sameScalar(
                                scene.impactTargets[impact][axis],
                                kPaperBreakableWallImpactTargets[impact][axis]);
    }
  }
  if (!intendedValuesValid) {
    scene.contractFailureMask |= 1u << 5u;
  }
  scene.contractPassed = scene.contractFailureMask == 0u;
  return scene;
}

std::uint64_t paperBreakableWallFingerprint(
    const std::vector<std::array<std::uint64_t, 3>>& topologyRecords)
{
  constexpr std::string_view kTag = "avbd-paper-breakable-wall/v3";

  Fnv1a64 fingerprint;
  fingerprint.updateString(kTag);
  // Effective values are checked above with one tight cross-language
  // tolerance, then serialized from canonical intended values. This avoids
  // binding the fingerprint to cancellation such as `(x + spacing) - x`.
  fingerprint.updateDouble(kPaperBreakableWallTimeStep);
  fingerprint.updateDouble(kPaperBreakableWallGravity);
  fingerprint.updateUint64(kPaperBreakableWallConstraintIterations);
  fingerprint.updateUint64(kPaperBreakableWallColumns);
  fingerprint.updateUint64(kPaperBreakableWallRows);
  for (const double value : kPaperBreakableWallBrickSize) {
    fingerprint.updateDouble(value);
  }
  for (const double value : kPaperBreakableWallBodyParameters) {
    fingerprint.updateDouble(value);
  }
  fingerprint.updateUint64(kPaperBreakableWallImpactTargets.size());
  for (const auto& target : kPaperBreakableWallImpactTargets) {
    fingerprint.updateDouble(target[0]);
    fingerprint.updateDouble(target[1]);
  }
  for (const double value : kPaperBreakableWallGroundSize) {
    fingerprint.updateDouble(value);
  }
  fingerprint.updateDouble(kPaperBreakableWallGroundFriction);
  fingerprint.updateDouble(kPaperBreakableWallBreakForce);
  fingerprint.updateUint64(topologyRecords.size());
  for (const auto& record : topologyRecords) {
    for (const std::uint64_t value : record) {
      fingerprint.updateUint64(value);
    }
  }
  return fingerprint.value();
}

std::uint64_t paperBreakableWallSolverConfigurationFingerprint(
    const sx::World& world, const std::vector<sx::Joint>& joints)
{
  constexpr std::string_view kTag
      = "paper-breakable-wall-solver-configuration/v2";
  constexpr double kBlockRegularization = 1e-12;
  constexpr double kBlockConvergenceDisplacement = 0.0;
  constexpr double kContactMaxStiffness
      = std::numeric_limits<double>::infinity();

  Fnv1a64 fingerprint;
  fingerprint.updateString(kTag);
  fingerprint.updateUint64(world.getRigidConstraintOptions().iterations);
  switch (world.getContactSolverMethod()) {
    case sx::ContactSolverMethod::SequentialImpulse:
      fingerprint.updateString("contact-method/sequential-impulse");
      break;
    case sx::ContactSolverMethod::BoxedLcp:
      fingerprint.updateString("contact-method/boxed-lcp");
      break;
  }

  const sx::RigidBodySolver solver = world.getRigidBodySolver();
  switch (solver) {
    case sx::RigidBodySolver::Avbd: {
      constexpr auto profile = vbd::kAvbdRigidPaper2025Profile;
      fingerprint.updateString("avbd/augmented-lagrangian/paper-2025-table-2");
      fingerprint.updateDouble(profile.alpha);
      fingerprint.updateDouble(profile.beta);
      fingerprint.updateDouble(profile.gamma);
      fingerprint.updateDouble(kBlockRegularization);
      fingerprint.updateDouble(kBlockConvergenceDisplacement);
      fingerprint.updateDouble(kPaperBreakableWallStartStiffness);
      fingerprint.updateDouble(kContactMaxStiffness);
      fingerprint.updateDouble(vbd::kAvbdRigidStaticFrictionTolerance);
      break;
    }
    case sx::RigidBodySolver::Vbd: {
      fingerprint.updateString("vbd/fixed-penalty");
      // The public VBD family uses the same contact descriptor defaults, but
      // holds the descriptor stiffness fixed and clears dual continuation.
      fingerprint.updateDouble(kBlockRegularization);
      fingerprint.updateDouble(kBlockConvergenceDisplacement);
      fingerprint.updateDouble(kPaperBreakableWallStartStiffness);
      fingerprint.updateDouble(kContactMaxStiffness);
      fingerprint.updateDouble(vbd::kAvbdRigidStaticFrictionTolerance);
      break;
    }
    case sx::RigidBodySolver::SequentialImpulse:
      fingerprint.updateString("sequential-impulse");
      break;
    case sx::RigidBodySolver::Ipc:
      fingerprint.updateString("ipc");
      break;
  }

  // Bind the effective solver-facing policy of every breakable attachment.
  // This matters because the matched VBD fixture intentionally materializes
  // finite penalty rows while AVBD and Sequential Impulse retain hard rows.
  // The source provenance digest guards compiled implementation constants;
  // this fingerprint guards the runtime configuration supplied to that code.
  fingerprint.updateUint64(joints.size());
  for (const sx::Joint& joint : joints) {
    const auto policy = joint.getConstraintProjectionPolicy();
    fingerprint.updateDouble(policy.startStiffness);
    fingerprint.updateDouble(policy.linearStiffness);
    fingerprint.updateDouble(policy.angularStiffness);
  }
  return fingerprint.value();
}

PaperBreakableWallFixture makePaperBreakableWallWorld(
    sx::RigidBodySolver solver)
{
  constexpr int kColumns = static_cast<int>(kPaperBreakableWallColumns);
  constexpr int kRows = static_cast<int>(kPaperBreakableWallRows);
  constexpr int kBrickCount = kColumns * kRows;
  constexpr int kHorizontalJoints = kRows * (kColumns - 1);
  constexpr int kVerticalJoints = (kRows - 1) * (2 * kColumns - 1);
  constexpr int kBaseJoints = kColumns;
  constexpr int kJointCount = kHorizontalJoints + kVerticalJoints + kBaseJoints;
  constexpr double kBrickDensity = kPaperBreakableWallBodyParameters[0];
  constexpr double kSpacingX = kPaperBreakableWallBodyParameters[2];
  constexpr double kSpacingZ = kPaperBreakableWallBodyParameters[3];
  constexpr double kBaseClearance = kPaperBreakableWallBodyParameters[4];
  constexpr double kBreakForce = kPaperBreakableWallBreakForce;
  constexpr double kBallRadius = kPaperBreakableWallBodyParameters[5];
  constexpr double kBallMass = kPaperBreakableWallBodyParameters[6];
  constexpr double kBallLaunchSpeed = kPaperBreakableWallBodyParameters[9];
  const Eigen::Vector3d brickSize(
      kPaperBreakableWallBrickSize[0],
      kPaperBreakableWallBrickSize[1],
      kPaperBreakableWallBrickSize[2]);
  const std::array<Eigen::Vector2d, 3> impactTargets{
      Eigen::Vector2d(
          kPaperBreakableWallImpactTargets[0][0],
          kPaperBreakableWallImpactTargets[0][1]),
      Eigen::Vector2d(
          kPaperBreakableWallImpactTargets[1][0],
          kPaperBreakableWallImpactTargets[1][1]),
      Eigen::Vector2d(
          kPaperBreakableWallImpactTargets[2][0],
          kPaperBreakableWallImpactTargets[2][1])};

  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d(0.0, 0.0, kPaperBreakableWallGravity);
  options.timeStep = kPaperBreakableWallTimeStep;
  options.rigidBodySolver = solver;
  options.rigidConstraintOptions.iterations
      = kPaperBreakableWallConstraintIterations;
  auto world = std::make_unique<sx::World>(options);
  const std::string solverPrefix
      = solver == sx::RigidBodySolver::SequentialImpulse ? "sequential_impulse"
        : solver == sx::RigidBodySolver::Vbd             ? "vbd"
                                                         : "avbd";

  auto ground = addAvbdDemo3dSourceBox(
      *world,
      solverPrefix + "_paper_breakable_wall_ground",
      Eigen::Vector3d(
          kPaperBreakableWallGroundSize[0],
          kPaperBreakableWallGroundSize[1],
          kPaperBreakableWallGroundSize[2]),
      0.0,
      Eigen::Vector3d(0.0, 0.0, -0.5 * kPaperBreakableWallGroundSize[2]),
      true);
  ground.setFriction(kPaperBreakableWallGroundFriction);

  const auto brickPosition = [&](const int row, const int column) {
    const double courseOffset = row % 2 == 0 ? 0.0 : 0.5;
    return Eigen::Vector3d(
        (static_cast<double>(column) - 0.5 * (kColumns - 1) + courseOffset)
            * kSpacingX,
        0.0,
        kBaseClearance + 0.5 * brickSize.z()
            + static_cast<double>(row) * kSpacingZ);
  };
  const auto brickIndex = [=](const int row, const int column) {
    return static_cast<std::size_t>(row * kColumns + column);
  };

  std::vector<sx::RigidBody> bricks;
  bricks.reserve(kBrickCount);
  for (int row = 0; row < kRows; ++row) {
    for (int column = 0; column < kColumns; ++column) {
      bricks.push_back(addAvbdDemo3dSourceBox(
          *world,
          solverPrefix + "_paper_wall_brick_" + std::to_string(row) + "_"
              + std::to_string(column),
          brickSize,
          kBrickDensity,
          brickPosition(row, column)));
    }
  }

  std::vector<sx::Joint> joints;
  joints.reserve(kJointCount);
  std::vector<std::array<std::uint64_t, 3>> topologyRecords;
  topologyRecords.reserve(kJointCount);
  const auto addBreakable = [&](const std::string& name,
                                const sx::RigidBody& parent,
                                const sx::RigidBody& child,
                                const std::uint64_t kind,
                                const std::uint64_t parentIndex,
                                const std::uint64_t childIndex) {
    auto joint = addFixedJoint(*world, name, parent, child);
    auto policy = joint.getConstraintProjectionPolicy();
    policy.startStiffness = kPaperBreakableWallStartStiffness;
    if (solver == sx::RigidBodySolver::Vbd) {
      policy.linearStiffness = policy.startStiffness;
      policy.angularStiffness = policy.startStiffness;
    }
    joint.setConstraintProjectionPolicy(policy);
    joint.setBreakForce(kBreakForce);
    joints.push_back(joint);
    topologyRecords.push_back({kind, parentIndex, childIndex});
  };

  for (int row = 0; row < kRows; ++row) {
    for (int column = 0; column + 1 < kColumns; ++column) {
      addBreakable(
          solverPrefix + "_paper_wall_horizontal_" + std::to_string(row) + "_"
              + std::to_string(column),
          bricks[brickIndex(row, column)],
          bricks[brickIndex(row, column + 1)],
          1u,
          1u + brickIndex(row, column),
          1u + brickIndex(row, column + 1));
    }
  }
  for (int row = 1; row < kRows; ++row) {
    for (int upperColumn = 0; upperColumn < kColumns; ++upperColumn) {
      const double upperX = brickPosition(row, upperColumn).x();
      for (int lowerColumn = 0; lowerColumn < kColumns; ++lowerColumn) {
        const double lowerX = brickPosition(row - 1, lowerColumn).x();
        if (std::abs(upperX - lowerX) < brickSize.x()) {
          addBreakable(
              solverPrefix + "_paper_wall_vertical_" + std::to_string(row - 1)
                  + "_" + std::to_string(lowerColumn) + "_"
                  + std::to_string(row) + "_" + std::to_string(upperColumn),
              bricks[brickIndex(row - 1, lowerColumn)],
              bricks[brickIndex(row, upperColumn)],
              2u,
              1u + brickIndex(row - 1, lowerColumn),
              1u + brickIndex(row, upperColumn));
        }
      }
    }
  }
  for (int column = 0; column < kColumns; ++column) {
    addBreakable(
        solverPrefix + "_paper_wall_base_" + std::to_string(column),
        ground,
        bricks[brickIndex(0, column)],
        3u,
        0u,
        1u + brickIndex(0, column));
  }

  std::vector<sx::RigidBody> balls;
  balls.reserve(impactTargets.size());
  for (std::size_t index = 0; index < impactTargets.size(); ++index) {
    sx::RigidBodyOptions ballOptions;
    ballOptions.mass = kBallMass;
    ballOptions.inertia = Eigen::Matrix3d::Identity()
                          * (2.0 / 5.0 * kBallMass * kBallRadius * kBallRadius);
    ballOptions.position = Eigen::Vector3d(
        impactTargets[index].x(),
        kPaperBreakableWallBodyParameters[8],
        impactTargets[index].y());
    ballOptions.linearVelocity = Eigen::Vector3d(0.0, kBallLaunchSpeed, 0.0);
    auto ball = world->addRigidBody(
        solverPrefix + "_paper_wall_ball_" + std::to_string(index),
        ballOptions);
    ball.setFriction(kPaperBreakableWallBodyParameters[7]);
    ball.setCollisionShape(sx::CollisionShape::makeSphere(kBallRadius));
    balls.push_back(ball);
  }

  benchmark::DoNotOptimize(bricks.data());
  benchmark::DoNotOptimize(joints.data());
  benchmark::DoNotOptimize(balls.data());
  const PaperBreakableWallEffectiveScene effectiveScene
      = inspectPaperBreakableWallEffectiveScene(
          *world,
          ground,
          bricks,
          balls,
          joints,
          topologyRecords,
          kColumns,
          kRows);
  const std::uint64_t sceneSpecFingerprint
      = paperBreakableWallFingerprint(topologyRecords);
  const bool matchesPythonSceneSpecFingerprint
      = sceneSpecFingerprint == kPaperBreakableWallPythonSceneSpecFingerprint;

  // Exercise the effective-scene guard with deliberate runtime mutations
  // during fixture setup. These checks run before benchmark timing begins and
  // restore every handle before the fixture is returned. A mutation is
  // detected when it either invalidates the readback contract or changes the
  // corresponding fingerprint.
  std::uint64_t mutationAuditFailureMask = 0u;
  const auto effectiveMutationDetected
      = [&](const std::vector<std::array<std::uint64_t, 3>>& records) {
          const PaperBreakableWallEffectiveScene mutatedScene
              = inspectPaperBreakableWallEffectiveScene(
                  *world,
                  ground,
                  bricks,
                  balls,
                  joints,
                  records,
                  kColumns,
                  kRows);
          return !mutatedScene.contractPassed
                 || paperBreakableWallFingerprint(records)
                        != sceneSpecFingerprint;
        };
  const auto requireMutationDetected
      = [&](const bool detected, const std::uint64_t bit) {
          if (!detected) {
            mutationAuditFailureMask |= bit;
          }
        };

  const double originalGroundFriction = ground.getFriction();
  ground.setFriction(originalGroundFriction + 0.125);
  requireMutationDetected(effectiveMutationDetected(topologyRecords), 1u << 0u);
  ground.setFriction(originalGroundFriction);

  const std::size_t representativeBrickIndex = bricks.size() / 2u;
  const double originalBrickRestitution
      = bricks[representativeBrickIndex].getRestitution();
  bricks[representativeBrickIndex].setRestitution(0.25);
  requireMutationDetected(effectiveMutationDetected(topologyRecords), 1u << 1u);
  bricks[representativeBrickIndex].setRestitution(originalBrickRestitution);

  std::vector<double> originalBrickFrictions;
  originalBrickFrictions.reserve(bricks.size());
  for (sx::RigidBody& brick : bricks) {
    originalBrickFrictions.push_back(brick.getFriction());
    brick.setFriction(brick.getFriction() + 0.125);
  }
  requireMutationDetected(effectiveMutationDetected(topologyRecords), 1u << 2u);
  for (std::size_t index = 0; index < bricks.size(); ++index) {
    bricks[index].setFriction(originalBrickFrictions[index]);
  }

  const Eigen::Isometry3d originalBallTransform = balls[0].getTransform();
  Eigen::Isometry3d mutatedBallTransform = originalBallTransform;
  mutatedBallTransform.translation().x() += 0.125;
  balls[0].setTransform(mutatedBallTransform);
  requireMutationDetected(effectiveMutationDetected(topologyRecords), 1u << 3u);
  balls[0].setTransform(originalBallTransform);

  const Eigen::Vector3d originalBallVelocity = balls[0].getLinearVelocity();
  balls[0].setLinearVelocity(
      originalBallVelocity + Eigen::Vector3d(0.0, 1.0, 0.0));
  requireMutationDetected(effectiveMutationDetected(topologyRecords), 1u << 4u);
  balls[0].setLinearVelocity(originalBallVelocity);

  auto mutatedTopologyRecords = topologyRecords;
  mutatedTopologyRecords[0][2] = mutatedTopologyRecords[1][2];
  requireMutationDetected(
      effectiveMutationDetected(mutatedTopologyRecords), 1u << 5u);

  const std::uint64_t solverConfigurationFingerprint
      = paperBreakableWallSolverConfigurationFingerprint(*world, joints);
  const auto originalProjectionPolicy
      = joints[0].getConstraintProjectionPolicy();
  auto mutatedProjectionPolicy = originalProjectionPolicy;
  mutatedProjectionPolicy.startStiffness += 1000.0;
  joints[0].setConstraintProjectionPolicy(mutatedProjectionPolicy);
  const bool projectionMutationDetected
      = paperBreakableWallSolverConfigurationFingerprint(*world, joints)
            != solverConfigurationFingerprint
        && !inspectPaperBreakableWallEffectiveScene(
                *world,
                ground,
                bricks,
                balls,
                joints,
                topologyRecords,
                kColumns,
                kRows)
                .contractPassed;
  requireMutationDetected(projectionMutationDetected, 1u << 6u);
  joints[0].setConstraintProjectionPolicy(originalProjectionPolicy);

  const PaperBreakableWallEffectiveScene restoredScene
      = inspectPaperBreakableWallEffectiveScene(
          *world,
          ground,
          bricks,
          balls,
          joints,
          topologyRecords,
          kColumns,
          kRows);
  const bool restorationPassed
      = restoredScene.contractPassed
        && paperBreakableWallFingerprint(topologyRecords)
               == sceneSpecFingerprint
        && paperBreakableWallSolverConfigurationFingerprint(*world, joints)
               == solverConfigurationFingerprint;
  if (!restorationPassed) {
    mutationAuditFailureMask |= 1u << 7u;
  }

  std::uint64_t effectiveSceneContractFailureMask
      = effectiveScene.contractFailureMask;
  if (!matchesPythonSceneSpecFingerprint) {
    effectiveSceneContractFailureMask |= 1u << 6u;
  }
  return PaperBreakableWallFixture{
      .world = std::move(world),
      .sceneSpecFingerprint = sceneSpecFingerprint,
      .effectiveSceneContractFailureMask = effectiveSceneContractFailureMask,
      .effectiveSceneContractPassed
      = effectiveScene.contractPassed && matchesPythonSceneSpecFingerprint,
      .mutationAuditFailureMask = mutationAuditFailureMask,
      .mutationAuditPassed = mutationAuditFailureMask == 0u,
      .matchesPythonSceneSpecFingerprint = matchesPythonSceneSpecFingerprint};
}

std::unique_ptr<sx::World> makeRigidBreakableJointWorld(std::size_t jointCount)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -9.81, 0.0);
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("breakable_base", baseOptions);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(jointCount);
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.mass = 1.0 + 0.1 * static_cast<double>(i % 3);
    bodyOptions.position = Eigen::Vector3d(
        0.6 * static_cast<double>(i + 1),
        0.05 * static_cast<double>(i % 2),
        0.0);

    auto child = world->addRigidBody(
        "breakable_link_" + std::to_string(i), bodyOptions);
    auto joint = addFixedJoint(
        *world, "breakable_fixed_" + std::to_string(i), parent, child);
    joint.setBreakForce(1.0e12);

    parent = child;
    links.push_back(child);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeRigidSphericalBreakableJointWorld(
    std::size_t jointCount)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.gravity = Eigen::Vector3d(0.0, -9.81, 0.0);
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("spherical_breakable_base", baseOptions);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(jointCount);
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.mass = 1.0 + 0.1 * static_cast<double>(i % 3);
    bodyOptions.position = Eigen::Vector3d(
        0.6 * static_cast<double>(i + 1),
        0.05 * static_cast<double>(i % 2),
        0.0);

    auto child = world->addRigidBody(
        "spherical_breakable_link_" + std::to_string(i), bodyOptions);
    auto joint = addSphericalJoint(
        *world,
        "spherical_breakable_socket_" + std::to_string(i),
        parent,
        child);
    joint.setBreakForce(1.0e12);

    parent = child;
    links.push_back(child);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedRevoluteMotorWorld(
    std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_revolute_motor_robot");
  auto base = robot.addLink("articulated_revolute_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    const Eigen::Vector3d anchor(
        0.45 * static_cast<double>(i + 1),
        0.08 * static_cast<double>(i % 2),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name = "articulated_revolute_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_revolute_link_" + std::to_string(i), base, floatingSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addRevoluteJoint(
        *world,
        "articulated_revolute_motor_" + std::to_string(i),
        base,
        link,
        Eigen::Vector3d::UnitZ(),
        anchor,
        Eigen::Vector3d::Zero());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.5 + 0.04 * static_cast<double>(i % 4)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -800.0),
        Eigen::VectorXd::Constant(1, 800.0));

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedBreakableMotorWorld(
    std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_breakable_motor_robot");
  auto base = robot.addLink("articulated_breakable_motor_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    const Eigen::Vector3d anchor(
        0.45 * static_cast<double>(i + 1),
        0.08 * static_cast<double>(i % 2),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name
        = "articulated_breakable_motor_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_breakable_motor_link_" + std::to_string(i),
        base,
        floatingSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addRevoluteJoint(
        *world,
        "articulated_breakable_motor_" + std::to_string(i),
        base,
        link,
        Eigen::Vector3d::UnitZ(),
        anchor,
        Eigen::Vector3d::Zero());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.5 + 0.04 * static_cast<double>(i % 4)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -800.0),
        Eigen::VectorXd::Constant(1, 800.0));
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedPrismaticMotorWorld(
    std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_prismatic_motor_robot");
  auto base = robot.addLink("articulated_prismatic_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    const Eigen::Vector3d anchor(
        0.35 * static_cast<double>(i + 1),
        0.07 * static_cast<double>(i % 3),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name = "articulated_prismatic_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_prismatic_link_" + std::to_string(i), base, floatingSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addPrismaticJoint(
        *world,
        "articulated_prismatic_motor_" + std::to_string(i),
        base,
        link,
        Eigen::Vector3d::UnitX(),
        anchor,
        Eigen::Vector3d::Zero());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.35 + 0.03 * static_cast<double>(i % 4)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -800.0),
        Eigen::VectorXd::Constant(1, 800.0));

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedPrismaticBreakableMotorWorld(
    std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot
      = world->addMultibody("articulated_prismatic_breakable_motor_robot");
  auto base = robot.addLink("articulated_prismatic_breakable_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    const Eigen::Vector3d anchor(
        0.35 * static_cast<double>(i + 1),
        0.07 * static_cast<double>(i % 3),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name
        = "articulated_prismatic_breakable_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_prismatic_breakable_link_" + std::to_string(i),
        base,
        floatingSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addPrismaticJoint(
        *world,
        "articulated_prismatic_breakable_motor_" + std::to_string(i),
        base,
        link,
        Eigen::Vector3d::UnitX(),
        anchor,
        Eigen::Vector3d::Zero());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.35 + 0.03 * static_cast<double>(i % 4)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -800.0),
        Eigen::VectorXd::Constant(1, 800.0));
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedWorldPrismaticBreakableMotorWorld(
    std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody(
      "articulated_world_prismatic_breakable_motor_robot");
  auto base = robot.addLink("articulated_world_prismatic_breakable_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    const Eigen::Vector3d anchor(
        0.35 * static_cast<double>(i + 1),
        0.07 * static_cast<double>(i % 3),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name
        = "articulated_world_prismatic_breakable_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_world_prismatic_breakable_link_" + std::to_string(i),
        base,
        floatingSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addPrismaticJoint(
        *world,
        "articulated_world_prismatic_breakable_motor_" + std::to_string(i),
        link,
        Eigen::Vector3d::UnitX(),
        anchor,
        Eigen::Vector3d::Zero());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.35 + 0.03 * static_cast<double>(i % 4)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -800.0),
        Eigen::VectorXd::Constant(1, 800.0));
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedWorldRevoluteBreakableMotorWorld(
    std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot
      = world->addMultibody("articulated_world_revolute_breakable_motor_robot");
  auto base = robot.addLink("articulated_world_revolute_breakable_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    const Eigen::Vector3d anchor(
        0.45 * static_cast<double>(i + 1),
        0.08 * static_cast<double>(i % 2),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name
        = "articulated_world_revolute_breakable_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_world_revolute_breakable_link_" + std::to_string(i),
        base,
        floatingSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addRevoluteJoint(
        *world,
        "articulated_world_revolute_breakable_motor_" + std::to_string(i),
        link,
        Eigen::Vector3d::UnitZ(),
        anchor,
        Eigen::Vector3d::Zero());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.5 + 0.04 * static_cast<double>(i % 4)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -800.0),
        Eigen::VectorXd::Constant(1, 800.0));
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedBreakableJointWorld(
    std::size_t jointCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d(0.0, -9.81, 0.0);
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_breakable_robot");
  auto base = robot.addLink("articulated_breakable_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(jointCount);
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    const Eigen::Vector3d anchor(
        0.4 * static_cast<double>(i + 1),
        0.06 * static_cast<double>(i % 2),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name = "articulated_breakable_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_breakable_link_" + std::to_string(i), base, floatingSpec);
    link.setMass(1.0 + 0.1 * static_cast<double>(i % 3));
    link.setInertia(Eigen::Vector3d(0.15, 0.2, 0.25).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addFixedJoint(
        *world,
        "articulated_breakable_fixed_" + std::to_string(i),
        base,
        link,
        anchor,
        Eigen::Vector3d::Zero());
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedWorldSphericalBreakableJointWorld(
    std::size_t jointCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d(0.0, -9.81, 0.0);
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot
      = world->addMultibody("articulated_world_spherical_breakable_robot");
  auto base = robot.addLink("articulated_world_spherical_breakable_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(jointCount);
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    const Eigen::Vector3d anchor(
        0.4 * static_cast<double>(i + 1),
        0.06 * static_cast<double>(i % 2),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name
        = "articulated_world_spherical_breakable_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_world_spherical_breakable_link_" + std::to_string(i),
        base,
        floatingSpec);
    link.setMass(1.0 + 0.1 * static_cast<double>(i % 3));
    link.setInertia(Eigen::Vector3d(0.15, 0.2, 0.25).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addSphericalJoint(
        *world,
        "articulated_world_spherical_breakable_socket_" + std::to_string(i),
        link,
        anchor,
        Eigen::Vector3d::Zero());
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedSphericalPairBreakableJointWorld(
    std::size_t jointCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d(0.0, -9.81, 0.0);
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot
      = world->addMultibody("articulated_spherical_pair_breakable_robot");
  auto base = robot.addLink("articulated_spherical_pair_breakable_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(jointCount);
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    const Eigen::Vector3d anchor(
        0.4 * static_cast<double>(i + 1),
        0.06 * static_cast<double>(i % 2),
        0.0);

    sx::JointSpec floatingSpec;
    floatingSpec.name
        = "articulated_spherical_pair_breakable_float_" + std::to_string(i);
    floatingSpec.type = sx::JointType::Floating;
    auto link = robot.addLink(
        "articulated_spherical_pair_breakable_link_" + std::to_string(i),
        base,
        floatingSpec);
    link.setMass(1.0 + 0.1 * static_cast<double>(i % 3));
    link.setInertia(Eigen::Vector3d(0.15, 0.2, 0.25).asDiagonal());
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose.head<3>() = anchor;
    link.getParentJoint().setPosition(pose);

    auto joint = addSphericalJoint(
        *world,
        "articulated_spherical_pair_breakable_socket_" + std::to_string(i),
        base,
        link,
        anchor,
        Eigen::Vector3d::Zero());
    joint.setBreakForce(1.0e12);

    links.push_back(link);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedCompliantJointWorld(
    std::size_t familyCount)
{
  constexpr double kStartStiffness = 10.0;
  constexpr double kLinearStiffness = 1000.0;
  constexpr double kAngularStiffness = 1000.0;

  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.002;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_compliant_joint_robot");
  auto base = robot.addLink("articulated_compliant_joint_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(3u * familyCount);
  joints.reserve(3u * familyCount);
  const std::array jointTypes{
      sx::JointType::Spherical,
      sx::JointType::Revolute,
      sx::JointType::Prismatic};
  for (std::size_t family = 0; family < jointTypes.size(); ++family) {
    for (std::size_t i = 0; i < familyCount; ++i) {
      const std::size_t flatIndex = family * familyCount + i;
      const Eigen::Vector3d anchor(
          0.35 * static_cast<double>(i + 1),
          0.2 * static_cast<double>(family),
          0.04 * static_cast<double>(i % 2));

      sx::JointSpec floatingSpec;
      floatingSpec.name
          = "articulated_compliant_float_" + std::to_string(flatIndex);
      floatingSpec.type = sx::JointType::Floating;
      auto link = robot.addLink(
          "articulated_compliant_link_" + std::to_string(flatIndex),
          base,
          floatingSpec);
      link.setMass(1.0 + 0.1 * static_cast<double>(i % 3));
      link.setInertia(Eigen::Vector3d(0.15, 0.2, 0.25).asDiagonal());
      Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
      pose.head<3>() = anchor;
      link.getParentJoint().setPosition(pose);
      link.getParentJoint().setVelocity(
          (Eigen::VectorXd(6) << 0.35, -0.25, 0.2, 0.4, -0.3, 0.5).finished());

      sx::Joint joint = addPointJoint(
          *world,
          "articulated_compliant_joint_" + std::to_string(flatIndex),
          link,
          jointTypes[family],
          family == 2u ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ(),
          anchor,
          Eigen::Vector3d::Zero());
      auto policy = joint.getConstraintProjectionPolicy();
      policy.startStiffness = kStartStiffness;
      policy.linearStiffness = kLinearStiffness;
      policy.angularStiffness = kAngularStiffness;
      joint.setConstraintProjectionPolicy(policy);

      links.push_back(link);
      joints.push_back(joint);
    }
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::unique_ptr<sx::World> makeArticulatedCompliantMotorWorld(
    std::size_t familyCount, bool breakable = false)
{
  constexpr double kStartStiffness = 20.0;
  constexpr double kLinearStiffness = 2000.0;
  constexpr double kAngularStiffness = 2000.0;

  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_compliant_motor_robot");
  auto base = robot.addLink("articulated_compliant_motor_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(4u * familyCount);
  joints.reserve(2u * familyCount);
  const std::array jointTypes{
      sx::JointType::Revolute, sx::JointType::Prismatic};
  const std::array axes{
      Eigen::Vector3d(1.0, 2.0, 3.0).normalized(),
      Eigen::Vector3d(1.0, -2.0, 0.5).normalized()};
  for (std::size_t family = 0; family < jointTypes.size(); ++family) {
    for (std::size_t i = 0; i < familyCount; ++i) {
      const std::size_t flatIndex = family * familyCount + i;
      const Eigen::Vector3d anchor(
          0.35 * static_cast<double>(i + 1),
          0.25 * static_cast<double>(family),
          0.04 * static_cast<double>(i % 2));

      const auto addFloatingLink = [&](std::string_view role, double mass) {
        sx::JointSpec floatingSpec;
        floatingSpec.name = "articulated_compliant_motor_" + std::string(role)
                            + "_float_" + std::to_string(flatIndex);
        floatingSpec.type = sx::JointType::Floating;
        auto link = robot.addLink(
            "articulated_compliant_motor_" + std::string(role) + "_link_"
                + std::to_string(flatIndex),
            base,
            floatingSpec);
        link.setMass(mass);
        link.setInertia(Eigen::Vector3d(0.15, 0.2, 0.25).asDiagonal());
        Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
        pose.head<3>() = anchor;
        link.getParentJoint().setPosition(pose);
        links.push_back(link);
        return link;
      };
      auto parent = addFloatingLink("parent", 1.0);
      auto child = addFloatingLink("child", 1.2);

      sx::Joint joint = addPointJoint(
          *world,
          "articulated_compliant_motor_" + std::to_string(flatIndex),
          parent,
          child,
          jointTypes[family],
          axes[family]);
      joint.setActuatorType(sx::ActuatorType::Velocity);
      joint.setCommandVelocity(
          Eigen::VectorXd::Constant(
              1, 0.3 + 0.03 * static_cast<double>(i % 4)));
      joint.setEffortLimits(
          Eigen::VectorXd::Constant(1, -800.0),
          Eigen::VectorXd::Constant(1, 800.0));
      auto policy = joint.getConstraintProjectionPolicy();
      policy.startStiffness = kStartStiffness;
      policy.linearStiffness = kLinearStiffness;
      policy.angularStiffness = kAngularStiffness;
      joint.setConstraintProjectionPolicy(policy);
      if (breakable) {
        joint.setBreakForce(1.0e12);
      }
      joints.push_back(joint);
    }
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

constexpr std::size_t kArticulatedHighRatioChainLinks = 5;
constexpr double kArticulatedHighRatioLinkLength = 0.45;
constexpr double kArticulatedHighRatioLightMass = 1.0;
constexpr double kArticulatedHighRatioTipMass = 200.0;
constexpr double kArticulatedHighRatioReplaySeconds = 1.0;
constexpr std::size_t kPaperScaleHighRatioChainLinks = 50;
constexpr double kPaperScaleHighRatioTipMass = 50000.0;
constexpr std::size_t kPaperScaleHighRatioReplaySteps = 32;
constexpr double kPaperScaleHighRatioReplaySeconds = 0.16;
constexpr std::size_t kPaperScaleHighRatioMaxIterations = 200;
constexpr double kPaperScaleHighRatioTolerance = 1e-9;

struct ArticulatedHighRatioChainFixture
{
  std::unique_ptr<sx::World> world;
  std::vector<sx::Joint> joints;
};

ArticulatedHighRatioChainFixture makeArticulatedHighRatioChainWorld(
    std::size_t linkCount,
    double tipMass,
    std::size_t maxIterations,
    double tolerance)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::MultibodyOptions multibodyOptions;
  multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  multibodyOptions.variationalMaxIterations = maxIterations;
  multibodyOptions.variationalTolerance = tolerance;
  world->setMultibodyOptions(multibodyOptions);

  auto robot = world->addMultibody("articulated_high_ratio_chain_robot");
  auto parent = robot.addLink("articulated_high_ratio_chain_base");

  std::vector<sx::Link> links;
  std::vector<sx::Joint> joints;
  links.reserve(linkCount);
  joints.reserve(linkCount);
  for (std::size_t i = 0; i < linkCount; ++i) {
    sx::JointSpec joint;
    joint.name = "articulated_high_ratio_chain_joint_" + std::to_string(i);
    joint.type = sx::JointType::Revolute;
    joint.axis = Eigen::Vector3d::UnitY();
    joint.transformFromParent = Eigen::Isometry3d(
        Eigen::Translation3d(
            i == 0
                ? Eigen::Vector3d::Zero()
                : Eigen::Vector3d(kArticulatedHighRatioLinkLength, 0.0, 0.0)));

    auto link = robot.addLink(
        "articulated_high_ratio_chain_link_" + std::to_string(i),
        parent,
        joint);
    const double mass
        = i == linkCount - 1 ? tipMass : kArticulatedHighRatioLightMass;
    const double transverseInertia = mass * kArticulatedHighRatioLinkLength
                                     * kArticulatedHighRatioLinkLength / 12.0;
    const double axisInertia = mass * 0.08 * 0.08 / 6.0;
    link.setMass(mass);
    link.setInertia(
        Eigen::Vector3d(axisInertia, transverseInertia, transverseInertia)
            .asDiagonal());

    joints.push_back(link.getParentJoint());
    links.push_back(link);
    parent = link;
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return {std::move(world), std::move(joints)};
}

ArticulatedHighRatioChainFixture makeArticulatedHighRatioChainWorld()
{
  return makeArticulatedHighRatioChainWorld(
      kArticulatedHighRatioChainLinks,
      kArticulatedHighRatioTipMass,
      sx::MultibodyOptions{}.variationalMaxIterations,
      sx::MultibodyOptions{}.variationalTolerance);
}

void resetArticulatedHighRatioChain(std::vector<sx::Joint>& joints)
{
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);
  for (auto& joint : joints) {
    joint.setPosition(zero);
    joint.setVelocity(zero);
  }
}

struct ArticulatedHighRatioReplayEnvelope
{
  double maxAbsPosition = 0.0;
  bool allFinite = true;
};

ArticulatedHighRatioReplayEnvelope measureArticulatedHighRatioReplayEnvelope(
    ArticulatedHighRatioChainFixture& fixture)
{
  fixture.world->restoreReplayFrame(0);
  ArticulatedHighRatioReplayEnvelope envelope;
  for (std::size_t step = 0; step < kPaperScaleHighRatioReplaySteps; ++step) {
    fixture.world->step();
    for (const auto& joint : fixture.joints) {
      if (!joint.getPosition().allFinite()
          || !joint.getVelocity().allFinite()) {
        envelope.allFinite = false;
        continue;
      }
      envelope.maxAbsPosition
          = std::max(envelope.maxAbsPosition, std::abs(joint.getPosition()[0]));
    }
  }
  fixture.world->restoreReplayFrame(0);
  return envelope;
}

std::vector<sx::Joint> makeRigidFixedJoints(
    sx::World& world, std::size_t jointCount)
{
  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world.addRigidBody("endpoint_base", baseOptions);

  std::vector<sx::Joint> joints;
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position
        = Eigen::Vector3d(static_cast<double>(i + 1), 0.0, 0.0);
    auto child
        = world.addRigidBody("endpoint_link_" + std::to_string(i), bodyOptions);
    joints.push_back(addFixedJoint(
        world, "endpoint_fixed_" + std::to_string(i), parent, child));
    parent = child;
  }

  return joints;
}

std::vector<entt::entity> makeEndpointClassificationEntities(
    sx::World& world, std::size_t endpointCount)
{
  std::vector<entt::entity> endpoints;
  endpoints.reserve(endpointCount * 2u);

  for (std::size_t i = 0; i < endpointCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position = Eigen::Vector3d(static_cast<double>(i), -0.5, 0.0);
    auto body = world.addRigidBody(
        "classifier_rigid_" + std::to_string(i), bodyOptions);
    endpoints.push_back(sx::detail::toRegistryEntity(body.getEntity()));
  }

  auto robot = world.addMultibody("classifier_robot");
  auto parent = robot.addLink("classifier_root");
  endpoints.push_back(sx::detail::toRegistryEntity(parent.getEntity()));
  for (std::size_t i = 1; i < endpointCount; ++i) {
    sx::JointSpec joint;
    joint.name = "classifier_joint_" + std::to_string(i);
    joint.type = sx::JointType::Revolute;
    joint.axis = Eigen::Vector3d::UnitZ();
    joint.transformFromParent
        = Eigen::Isometry3d(Eigen::Translation3d(Eigen::Vector3d::UnitX()));
    parent
        = robot.addLink("classifier_link_" + std::to_string(i), parent, joint);
    endpoints.push_back(sx::detail::toRegistryEntity(parent.getEntity()));
  }

  return endpoints;
}

enum class BenchmarkRuntimeIdentity
{
  PublicAvbdRigid,
  VariationalMultibody,
};

bool hasOnlyResolvedNote(
    const sx::World& world,
    std::string_view domain,
    std::string_view requested,
    std::string_view resolved)
{
  std::size_t domainCount = 0u;
  bool matches = false;
  for (const auto& note : world.getResolvedConfiguration().notes) {
    if (note.domain != domain) {
      continue;
    }
    ++domainCount;
    matches = note.requested == requested && note.resolved == resolved;
  }
  return domainCount == 1u && matches;
}

bool enterSimulationModeAndRecordRuntimeIdentity(
    benchmark::State& state,
    sx::World& world,
    BenchmarkRuntimeIdentity expected)
{
  world.enterSimulationMode();

  const auto& registry = sx::detail::registryOf(world);
  const bool hasPairConstraints
      = vbd::mayHaveAvbdRigidWorldPointJointConfigs(registry)
        || vbd::mayHaveAvbdRigidWorldDistanceSpringConfigs(registry);
  const bool pairInactive
      = !hasPairConstraints
        && hasOnlyResolvedNote(
            world, "rigid-pair-constraint", "inactive", "inactive");

  state.counters["runtime_identity_recorded"] = 1.0;
  state.counters["runtime_identity_applicable"] = 1.0;
  state.counters["runtime_identity_not_applicable"] = 0.0;
  state.counters["runtime_identity_public_avbd_rigid"]
      = expected == BenchmarkRuntimeIdentity::PublicAvbdRigid ? 1.0 : 0.0;
  state.counters["runtime_identity_variational_multibody"]
      = expected == BenchmarkRuntimeIdentity::VariationalMultibody ? 1.0 : 0.0;

  bool runtimeContractPassed = false;
  if (expected == BenchmarkRuntimeIdentity::PublicAvbdRigid) {
    const bool publicAvbdFamily
        = world.getRigidBodySolver() == sx::RigidBodySolver::Avbd;
    const bool resolvedRigidBody
        = hasOnlyResolvedNote(world, "rigid-body", "avbd", "avbd");
    const bool resolvedRigidContact
        = hasOnlyResolvedNote(world, "rigid-contact", "avbd", "avbd");
    const bool resolvedRigidPairConstraint
        = hasPairConstraints
          && hasOnlyResolvedNote(
              world, "rigid-pair-constraint", "avbd", "avbd");
    const bool resolvedParameterProfile = hasOnlyResolvedNote(
        world,
        "rigid-avbd-parameter-profile",
        "paper-2025-table-2",
        "paper-2025-table-2");
    constexpr auto profile = vbd::kAvbdRigidPaper2025Profile;

    state.counters["public_avbd_family"] = publicAvbdFamily ? 1.0 : 0.0;
    state.counters["resolved_rigid_body_avbd"] = resolvedRigidBody ? 1.0 : 0.0;
    state.counters["resolved_rigid_contact_avbd"]
        = resolvedRigidContact ? 1.0 : 0.0;
    state.counters["resolved_rigid_pair_constraint_avbd"]
        = resolvedRigidPairConstraint ? 1.0 : 0.0;
    state.counters["resolved_rigid_pair_constraint_not_applicable"]
        = pairInactive ? 1.0 : 0.0;
    state.counters["rigid_avbd_parameter_profile_paper_2025"]
        = resolvedParameterProfile ? 1.0 : 0.0;
    state.counters["rigid_avbd_alpha"] = profile.alpha;
    state.counters["rigid_avbd_beta"] = profile.beta;
    state.counters["rigid_avbd_gamma"] = profile.gamma;
    state.counters["resolved_multibody_variational"] = 0.0;

    runtimeContractPassed = publicAvbdFamily && resolvedRigidBody
                            && resolvedRigidContact
                            && (resolvedRigidPairConstraint || pairInactive)
                            && resolvedParameterProfile;
  } else {
    const bool publicSequentialImpulseFamily
        = world.getRigidBodySolver() == sx::RigidBodySolver::SequentialImpulse;
    const bool configuredVariationalMultibody
        = world.getMultibodyOptions().integrationFamily
          == sx::MultibodyIntegrationFamily::Variational;
    const bool resolvedRigidBody = hasOnlyResolvedNote(
        world, "rigid-body", "sequential-impulse", "sequential-impulse");
    const bool resolvedRigidContact = hasOnlyResolvedNote(
        world, "rigid-contact", "sequential-impulse", "sequential-impulse");
    const bool resolvedRigidPairConstraint = hasPairConstraints
                                             && hasOnlyResolvedNote(
                                                 world,
                                                 "rigid-pair-constraint",
                                                 "sequential-impulse",
                                                 "sequential-impulse");
    const bool resolvedVariationalMultibody
        = hasOnlyResolvedNote(world, "multibody", "variational", "variational");

    state.counters["public_avbd_family"] = 0.0;
    state.counters["public_sequential_impulse_family"]
        = publicSequentialImpulseFamily ? 1.0 : 0.0;
    state.counters["resolved_rigid_body_avbd"] = 0.0;
    state.counters["resolved_rigid_contact_avbd"] = 0.0;
    state.counters["resolved_rigid_pair_constraint_avbd"] = 0.0;
    state.counters["resolved_rigid_body_sequential_impulse"]
        = resolvedRigidBody ? 1.0 : 0.0;
    state.counters["resolved_rigid_contact_sequential_impulse"]
        = resolvedRigidContact ? 1.0 : 0.0;
    state.counters["resolved_rigid_pair_constraint_sequential_impulse"]
        = resolvedRigidPairConstraint ? 1.0 : 0.0;
    state.counters["resolved_rigid_pair_constraint_not_applicable"]
        = pairInactive ? 1.0 : 0.0;
    state.counters["rigid_avbd_parameter_profile_paper_2025"] = 0.0;
    state.counters["rigid_avbd_alpha"] = 0.0;
    state.counters["rigid_avbd_beta"] = 0.0;
    state.counters["rigid_avbd_gamma"] = 0.0;
    state.counters["configured_multibody_variational"]
        = configuredVariationalMultibody ? 1.0 : 0.0;
    state.counters["resolved_multibody_variational"]
        = resolvedVariationalMultibody ? 1.0 : 0.0;

    runtimeContractPassed = publicSequentialImpulseFamily
                            && configuredVariationalMultibody
                            && resolvedRigidBody && resolvedRigidContact
                            && (resolvedRigidPairConstraint || pairInactive)
                            && resolvedVariationalMultibody;
  }

  state.counters["runtime_identity_contract_passed"]
      = runtimeContractPassed ? 1.0 : 0.0;
  if (!runtimeContractPassed) {
    state.SkipWithError("benchmark runtime solver identity drifted");
  }
  return runtimeContractPassed;
}

void recordNotApplicableRuntimeIdentity(
    benchmark::State& state, bool emptyWorld, const sx::World* world = nullptr)
{
  const bool runtimeContractPassed
      = !emptyWorld
        || (world != nullptr && world->getRigidBodyCount() == 0u
            && world->getMultibodyCount() == 0u
            && world->getDeformableBodyCount() == 0u
            && world->getJointCount() == 0u);
  state.counters["runtime_identity_recorded"] = 1.0;
  state.counters["runtime_identity_applicable"] = 0.0;
  state.counters["runtime_identity_not_applicable"] = 1.0;
  state.counters["runtime_identity_api_only"] = emptyWorld ? 0.0 : 1.0;
  state.counters["runtime_identity_empty_world"] = emptyWorld ? 1.0 : 0.0;
  state.counters["runtime_identity_public_avbd_rigid"] = 0.0;
  state.counters["runtime_identity_variational_multibody"] = 0.0;
  state.counters["runtime_identity_contract_passed"]
      = runtimeContractPassed ? 1.0 : 0.0;
  if (!runtimeContractPassed) {
    state.SkipWithError("empty benchmark row gained simulation entities");
  }
}

} // namespace

//==============================================================================
static void BM_AvbdEmptyWorldStep(benchmark::State& state)
{
  auto world = makeAvbdEmptyWorld();
  world->enterSimulationMode();

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 0.0;
  state.counters["multibodies"] = 0.0;
  recordNotApplicableRuntimeIdentity(state, /*emptyWorld=*/true, world.get());
}
BENCHMARK(BM_AvbdEmptyWorldStep);

//==============================================================================
static void BM_AvbdRigidFixedJointCreate(benchmark::State& state)
{
  const auto linkCount = static_cast<std::size_t>(state.range(0));
  for (auto _ : state) {
    auto world = makeRigidFixedJointWorld(linkCount);
    benchmark::DoNotOptimize(world.get());
    benchmark::ClobberMemory();
  }
  state.counters["fixed_joints"] = static_cast<double>(linkCount);
  recordNotApplicableRuntimeIdentity(state, /*emptyWorld=*/false);
}
BENCHMARK(BM_AvbdRigidFixedJointCreate)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointEndpointAccess(benchmark::State& state)
{
  sx::World world;
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto joints = makeRigidFixedJoints(world, jointCount);

  for (auto _ : state) {
    for (const sx::Joint& joint : joints) {
      auto parent = joint.getParentRigidBody();
      auto child = joint.getChildRigidBody();
      benchmark::DoNotOptimize(parent.isValid());
      benchmark::DoNotOptimize(child.isValid());
    }
  }
  state.counters["fixed_joints"] = static_cast<double>(jointCount);
  recordNotApplicableRuntimeIdentity(state, /*emptyWorld=*/false);
}
BENCHMARK(BM_AvbdRigidFixedJointEndpointAccess)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointWorldLookup(benchmark::State& state)
{
  sx::World world;
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  (void)makeRigidFixedJoints(world, jointCount);

  std::vector<std::string> names;
  names.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    names.push_back("endpoint_fixed_" + std::to_string(i));
  }

  for (auto _ : state) {
    for (const std::string& name : names) {
      auto joint = world.getJoint(name);
      benchmark::DoNotOptimize(joint.has_value());
      if (joint.has_value()) {
        benchmark::DoNotOptimize(joint->getParentRigidBody().isValid());
        benchmark::DoNotOptimize(joint->getChildRigidBody().isValid());
      }
    }
    benchmark::DoNotOptimize(world.getJointCount());
  }
  state.counters["fixed_joints"] = static_cast<double>(jointCount);
  recordNotApplicableRuntimeIdentity(state, /*emptyWorld=*/false);
}
BENCHMARK(BM_AvbdRigidFixedJointWorldLookup)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointWorldList(benchmark::State& state)
{
  sx::World world;
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  (void)makeRigidFixedJoints(world, jointCount);

  for (auto _ : state) {
    const auto joints = world.getJoints();
    benchmark::DoNotOptimize(joints.data());
    benchmark::DoNotOptimize(joints.size());
  }
  state.counters["fixed_joints"] = static_cast<double>(jointCount);
  recordNotApplicableRuntimeIdentity(state, /*emptyWorld=*/false);
}
BENCHMARK(BM_AvbdRigidFixedJointWorldList)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidEndpointClassification(benchmark::State& state)
{
  sx::World world;
  const auto endpointCount = static_cast<std::size_t>(state.range(0));
  const std::vector<entt::entity> endpoints
      = makeEndpointClassificationEntities(world, endpointCount);
  const auto& registry = sx::detail::registryOf(world);

  for (auto _ : state) {
    std::size_t freeRigidEndpoints = 0;
    std::size_t multibodyEndpoints = 0;
    for (const entt::entity endpointEntity : endpoints) {
      const vbd::AvbdRigidWorldEndpoint endpoint
          = vbd::classifyAvbdRigidWorldEndpoint(registry, endpointEntity);
      if (endpoint.kind == vbd::AvbdRigidWorldEndpointKind::FreeRigidBody) {
        ++freeRigidEndpoints;
      } else if (
          endpoint.kind == vbd::AvbdRigidWorldEndpointKind::MultibodyLink) {
        ++multibodyEndpoints;
      }
    }
    benchmark::DoNotOptimize(freeRigidEndpoints);
    benchmark::DoNotOptimize(multibodyEndpoints);
  }

  state.counters["free_rigid_endpoints"] = static_cast<double>(endpointCount);
  state.counters["multibody_link_endpoints"]
      = static_cast<double>(endpointCount);
  recordNotApplicableRuntimeIdentity(state, /*emptyWorld=*/false);
}
BENCHMARK(BM_AvbdRigidEndpointClassification)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointStep(benchmark::State& state)
{
  const auto linkCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidFixedJointWorld(linkCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["fixed_joints"] = static_cast<double>(linkCount);
}
BENCHMARK(BM_AvbdRigidFixedJointStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidRevoluteMotorStep(benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidRevoluteMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdRigidRevoluteMotorStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidPrismaticMotorStep(benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidPrismaticMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdRigidPrismaticMotorStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdDemo2dMotorStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dMotorWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 2.0;
  state.counters["rigid_body_joints"] = 1.0;
  state.counters["motors"] = 1.0;
  state.counters["collision_shapes"] = 0.0;
  state.counters["source_scene_index"] = 17.0;
}
BENCHMARK(BM_AvbdDemo2dMotorStep);

//==============================================================================
static void BM_AvbdDemo2dGroundStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dGroundWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 1.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 1.0;
  state.counters["source_scene_index"] = 1.0;
}
BENCHMARK(BM_AvbdDemo2dGroundStep);

//==============================================================================
static void BM_AvbdDemo2dDynamicFrictionStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dDynamicFrictionWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 12.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 12.0;
  state.counters["source_scene_index"] = 2.0;
}
BENCHMARK(BM_AvbdDemo2dDynamicFrictionStep);

//==============================================================================
static void BM_AvbdDemo2dFrictionCoefficientSweep(benchmark::State& state)
{
  constexpr double kFrictionScale = 10.0;
  const double maxFriction
      = static_cast<double>(state.range(0)) / kFrictionScale;
  auto world = makeAvbdDemo2dDynamicFrictionWorld(maxFriction);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 12.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 12.0;
  state.counters["source_scene_index"] = 2.0;
  state.counters["max_friction"] = maxFriction;
  state.counters["min_friction"] = 0.0;
  state.counters["friction_samples"] = 11.0;
}
BENCHMARK(BM_AvbdDemo2dFrictionCoefficientSweep)
    ->Arg(0)
    ->Arg(5)
    ->Arg(10)
    ->Arg(25)
    ->Arg(50);

//==============================================================================
static void BM_AvbdDemo2dStaticFrictionStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dStaticFrictionWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 12.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 12.0;
  state.counters["source_scene_index"] = 3.0;
}
BENCHMARK(BM_AvbdDemo2dStaticFrictionStep);

//==============================================================================
static void BM_AvbdDemo2dPyramidStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dPyramidWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 211.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 211.0;
  state.counters["source_scene_index"] = 4.0;
}
BENCHMARK(BM_AvbdDemo2dPyramidStep);

//==============================================================================
static void BM_AvbdDemo2dCardsStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dCardsWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 41.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 41.0;
  state.counters["cards"] = 40.0;
  state.counters["source_scene_index"] = 5.0;
}
BENCHMARK(BM_AvbdDemo2dCardsStep);

//==============================================================================
static void BM_AvbdDemo2dStackStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dStackWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 21.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 21.0;
  state.counters["source_scene_index"] = 11.0;
}
BENCHMARK(BM_AvbdDemo2dStackStep);

//==============================================================================
static void BM_AvbdDemo2dStackRatioStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dStackRatioWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 7.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 7.0;
  state.counters["source_scene_index"] = 12.0;
}
BENCHMARK(BM_AvbdDemo2dStackRatioStep);

//==============================================================================
static void BM_AvbdDemo2dRodStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dRodWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 20.0;
  state.counters["rigid_body_joints"] = 19.0;
  state.counters["fixed_joints"] = 19.0;
  state.counters["collision_shapes"] = 20.0;
  state.counters["source_scene_index"] = 13.0;
}
BENCHMARK(BM_AvbdDemo2dRodStep);

//==============================================================================
static void BM_AvbdDemo2dJointGridStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dJointGridWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 625.0;
  state.counters["rigid_body_joints"] = 1200.0;
  state.counters["fixed_joints"] = 1200.0;
  state.counters["collision_shapes"] = 625.0;
  state.counters["ignored_collision_pairs"] = 1152.0;
  state.counters["source_scene_index"] = 15.0;
}
BENCHMARK(BM_AvbdDemo2dJointGridStep);

//==============================================================================
static void BM_AvbdDemo2dSoftBodyStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dSoftBodyWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 151.0;
  state.counters["rigid_body_joints"] = 260.0;
  state.counters["fixed_joints"] = 260.0;
  state.counters["finite_stiffness_fixed_joints"] = 260.0;
  state.counters["collision_shapes"] = 151.0;
  state.counters["ignored_collision_pairs"] = 224.0;
  state.counters["source_scene_index"] = 14.0;
}
BENCHMARK(BM_AvbdDemo2dSoftBodyStep);

//==============================================================================
static void BM_AvbdDemo2dRopeStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dRopeWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 20.0;
  state.counters["rigid_body_joints"] = 19.0;
  state.counters["linear_point_joints"] = 19.0;
  state.counters["collision_shapes"] = 20.0;
  state.counters["source_scene_index"] = 6.0;
}
BENCHMARK(BM_AvbdDemo2dRopeStep);

//==============================================================================
static void BM_AvbdDemo2dHeavyRopeStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dHeavyRopeWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 20.0;
  state.counters["rigid_body_joints"] = 19.0;
  state.counters["linear_point_joints"] = 19.0;
  state.counters["collision_shapes"] = 20.0;
  state.counters["source_scene_index"] = 7.0;
}
BENCHMARK(BM_AvbdDemo2dHeavyRopeStep);

//==============================================================================
static void BM_AvbdDemo2dHangingRopeStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dHangingRopeWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 50.0;
  state.counters["rigid_body_joints"] = 49.0;
  state.counters["linear_point_joints"] = 49.0;
  state.counters["collision_shapes"] = 50.0;
  state.counters["source_scene_index"] = 8.0;
}
BENCHMARK(BM_AvbdDemo2dHangingRopeStep);

//==============================================================================
static void BM_AvbdDemo2dSpringStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dSpringWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 2.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["distance_springs"] = 1.0;
  state.counters["collision_shapes"] = 2.0;
  state.counters["ignored_collision_pairs"] = 1.0;
  state.counters["source_scene_index"] = 9.0;
}
BENCHMARK(BM_AvbdDemo2dSpringStep);

//==============================================================================
static void BM_AvbdDemo2dSpringRatioStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dSpringRatioWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 8.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["distance_springs"] = 7.0;
  state.counters["collision_shapes"] = 8.0;
  state.counters["ignored_collision_pairs"] = 7.0;
  state.counters["source_scene_index"] = 10.0;
}
BENCHMARK(BM_AvbdDemo2dSpringRatioStep);

//==============================================================================
static void BM_AvbdDemo2dNetStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dNetWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 91.0;
  state.counters["rigid_body_joints"] = 39.0;
  state.counters["linear_point_joints"] = 39.0;
  state.counters["collision_shapes"] = 91.0;
  state.counters["source_scene_index"] = 16.0;
}
BENCHMARK(BM_AvbdDemo2dNetStep);

//==============================================================================
static void BM_AvbdDemo2dFractureStep(benchmark::State& state)
{
  auto world = makeAvbdDemo2dFractureWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 29.0;
  state.counters["rigid_body_joints"] = 10.0;
  state.counters["breakable_joints"] = 10.0;
  state.counters["joint_connected_collision_pairs"] = 10.0;
  state.counters["collision_shapes"] = 29.0;
  state.counters["source_scene_index"] = 18.0;
}
BENCHMARK(BM_AvbdDemo2dFractureStep);

//==============================================================================
static void BM_AvbdDemo3dGroundStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dGroundWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 2.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 2.0;
  state.counters["source_scene_index"] = 1.0;
}
BENCHMARK(BM_AvbdDemo3dGroundStep);

//==============================================================================
static void BM_AvbdDemo3dDynamicFrictionStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dDynamicFrictionWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 12.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 12.0;
  state.counters["source_scene_index"] = 2.0;
}
BENCHMARK(BM_AvbdDemo3dDynamicFrictionStep);

//==============================================================================
static void BM_AvbdDemo3dStaticFrictionStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dStaticFrictionWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 13.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 13.0;
  state.counters["source_scene_index"] = 3.0;
}
BENCHMARK(BM_AvbdDemo3dStaticFrictionStep);

//==============================================================================
static void BM_AvbdDemo3dPyramidStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dPyramidWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 137.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 137.0;
  state.counters["source_scene_index"] = 4.0;
}
BENCHMARK(BM_AvbdDemo3dPyramidStep);

//==============================================================================
static void BM_AvbdDemo3dRopeStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dRopeWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 21.0;
  state.counters["rigid_body_joints"] = 19.0;
  state.counters["linear_point_joints"] = 19.0;
  state.counters["collision_shapes"] = 21.0;
  state.counters["source_scene_index"] = 5.0;
}
BENCHMARK(BM_AvbdDemo3dRopeStep);

//==============================================================================
static void BM_AvbdDemo3dHeavyRopeStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dHeavyRopeWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 21.0;
  state.counters["rigid_body_joints"] = 19.0;
  state.counters["linear_point_joints"] = 19.0;
  state.counters["collision_shapes"] = 21.0;
  state.counters["source_scene_index"] = 6.0;
}
BENCHMARK(BM_AvbdDemo3dHeavyRopeStep);

//==============================================================================
static void BM_AvbdDemo3dSpringStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dSpringWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 3.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["distance_springs"] = 1.0;
  state.counters["collision_shapes"] = 3.0;
  state.counters["ignored_collision_pairs"] = 1.0;
  state.counters["source_scene_index"] = 7.0;
}
BENCHMARK(BM_AvbdDemo3dSpringStep);

//==============================================================================
static void BM_AvbdDemo3dSpringRatioStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dSpringRatioWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 9.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["distance_springs"] = 7.0;
  state.counters["collision_shapes"] = 9.0;
  state.counters["ignored_collision_pairs"] = 7.0;
  state.counters["source_scene_index"] = 8.0;
}
BENCHMARK(BM_AvbdDemo3dSpringRatioStep);

//==============================================================================
static void BM_AvbdDemo3dStackStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dStackWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 11.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 11.0;
  state.counters["source_scene_index"] = 9.0;
}
BENCHMARK(BM_AvbdDemo3dStackStep);

//==============================================================================
static void BM_AvbdDemo3dStackRatioStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dStackRatioWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 5.0;
  state.counters["rigid_body_joints"] = 0.0;
  state.counters["collision_shapes"] = 5.0;
  state.counters["source_scene_index"] = 10.0;
}
BENCHMARK(BM_AvbdDemo3dStackRatioStep);

//==============================================================================
static void BM_AvbdDemo3dSoftBodyStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dSoftBodyWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 193.0;
  state.counters["rigid_body_joints"] = 432.0;
  state.counters["fixed_joints"] = 432.0;
  state.counters["finite_stiffness_fixed_joints"] = 432.0;
  state.counters["collision_shapes"] = 193.0;
  state.counters["ignored_collision_pairs"] = 648.0;
  state.counters["source_scene_index"] = 11.0;
}
BENCHMARK(BM_AvbdDemo3dSoftBodyStep);

//==============================================================================
static void BM_AvbdDemo3dBridgeStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dBridgeWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 91.0;
  state.counters["rigid_body_joints"] = 78.0;
  state.counters["linear_point_joints"] = 78.0;
  state.counters["collision_shapes"] = 91.0;
  state.counters["source_scene_index"] = 12.0;
}
BENCHMARK(BM_AvbdDemo3dBridgeStep);

//==============================================================================
static void BM_AvbdDemo3dBreakableStep(benchmark::State& state)
{
  auto world = makeAvbdDemo3dBreakableWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["rigid_bodies"] = 19.0;
  state.counters["rigid_body_joints"] = 10.0;
  state.counters["breakable_joints"] = 10.0;
  state.counters["collision_shapes"] = 19.0;
  state.counters["source_scene_index"] = 13.0;
}
BENCHMARK(BM_AvbdDemo3dBreakableStep);

//==============================================================================
void runPaperBreakableWallStepBenchmark(
    benchmark::State& state,
    sx::RigidBodySolver solver,
    std::string_view solverKey)
{
  constexpr std::size_t kTrajectoryFrames = 120;
  auto fixture = makePaperBreakableWallWorld(solver);
  fixture.world->enterSimulationMode();
  std::string objectKey(solverKey);
  std::ranges::replace(objectKey, '-', '_');
  const std::string bodyPrefix = objectKey + "_paper_wall_ball_";

  const auto bodyNames = fixture.world->getRigidBodyNames();
  std::size_t collisionShapeCount = 0;
  std::size_t impactingBallCount = 0;
  for (const auto& name : bodyNames) {
    const auto body = fixture.world->getRigidBody(name);
    if (body.has_value() && body->hasCollisionShape()) {
      ++collisionShapeCount;
    }
    if (std::string_view(name).starts_with(bodyPrefix)) {
      ++impactingBallCount;
    }
  }

  const auto joints = fixture.world->getJoints();
  const auto breakableJointCount = static_cast<std::size_t>(
      std::ranges::count_if(joints, [](const sx::Joint& joint) {
        return std::isfinite(joint.getBreakForce())
               && joint.getBreakForce() > 0.0;
      }));
  const auto rigidConstraintIterations
      = fixture.world->getRigidConstraintOptions().iterations;
  const bool publicFamily = fixture.world->getRigidBodySolver() == solver;
  const bool defaultContactMethod
      = fixture.world->getContactSolverMethod()
        == sx::ContactSolverMethod::SequentialImpulse;
  bool solverProjectionPoliciesMatch = true;
  sx::JointConstraintProjectionPolicy firstMismatchedProjectionPolicy;
  for (const sx::Joint& joint : joints) {
    const auto policy = joint.getConstraintProjectionPolicy();
    const bool finiteVbdRows = solver == sx::RigidBodySolver::Vbd;
    const bool matches
        = policy.startStiffness == kPaperBreakableWallStartStiffness
          && (finiteVbdRows
                  ? policy.linearStiffness == kPaperBreakableWallStartStiffness
                  : std::isinf(policy.linearStiffness)
                        && policy.linearStiffness > 0.0)
          && (finiteVbdRows
                  ? policy.angularStiffness == kPaperBreakableWallStartStiffness
                  : std::isinf(policy.angularStiffness)
                        && policy.angularStiffness > 0.0);
    if (!matches) {
      solverProjectionPoliciesMatch = false;
      firstMismatchedProjectionPolicy = policy;
      break;
    }
  }
  const auto& resolved = fixture.world->getResolvedConfiguration();
  const auto hasResolvedNote = [&](std::string_view domain,
                                   std::string_view requested,
                                   std::string_view actual) {
    return std::ranges::count_if(
               resolved.notes,
               [&](const sx::compute::ResolvedConfigurationNote& note) {
                 return note.domain == domain && note.requested == requested
                        && note.resolved == actual;
               })
           == 1;
  };
  const bool resolvedRigidBody
      = hasResolvedNote("rigid-body", solverKey, solverKey);
  const bool resolvedRigidContact
      = hasResolvedNote("rigid-contact", solverKey, solverKey);
  const bool resolvedRigidPairConstraint
      = hasResolvedNote("rigid-pair-constraint", solverKey, solverKey);
  const std::string iterationText = std::to_string(rigidConstraintIterations);
  const bool resolvedRigidConstraintIterations = hasResolvedNote(
      "rigid-constraint-iterations", iterationText, iterationText);
  const bool resolvedAvbdParameterProfile = solver != sx::RigidBodySolver::Avbd
                                            || hasResolvedNote(
                                                "rigid-avbd-parameter-profile",
                                                "paper-2025-table-2",
                                                "paper-2025-table-2");
  std::uint64_t runtimeFailureMask = 0u;
  const auto requireRuntime
      = [&](const bool condition, const std::uint64_t bit) {
          if (!condition) {
            runtimeFailureMask |= bit;
          }
        };
  requireRuntime(
      fixture.world->getRigidBodyCount() == bodyNames.size()
          && fixture.world->getRigidBodyCount() == 256u,
      1u << 0u);
  requireRuntime(
      fixture.world->getJointCount() == joints.size()
          && fixture.world->getJointCount() == 712u,
      1u << 1u);
  requireRuntime(breakableJointCount == joints.size(), 1u << 2u);
  requireRuntime(collisionShapeCount == 256u, 1u << 3u);
  requireRuntime(impactingBallCount == 3u, 1u << 4u);
  requireRuntime(rigidConstraintIterations == 20u, 1u << 5u);
  requireRuntime(publicFamily, 1u << 6u);
  requireRuntime(defaultContactMethod, 1u << 7u);
  requireRuntime(solverProjectionPoliciesMatch, 1u << 8u);
  requireRuntime(fixture.effectiveSceneContractPassed, 1u << 9u);
  requireRuntime(resolvedRigidBody, 1u << 10u);
  requireRuntime(resolvedRigidContact, 1u << 11u);
  requireRuntime(resolvedRigidPairConstraint, 1u << 12u);
  requireRuntime(resolvedRigidConstraintIterations, 1u << 13u);
  requireRuntime(resolvedAvbdParameterProfile, 1u << 14u);
  requireRuntime(fixture.mutationAuditPassed, 1u << 15u);
  requireRuntime(fixture.matchesPythonSceneSpecFingerprint, 1u << 16u);
  const bool runtimeContractPassed = runtimeFailureMask == 0u;
  if (!runtimeContractPassed) {
    const std::string error = std::string(solverKey)
                              + " paper breakable-wall runtime configuration "
                                "drifted (effective-scene failure mask="
                              + std::to_string(
                                  fixture.effectiveSceneContractFailureMask)
                              + ", mutation-audit failure mask="
                              + std::to_string(fixture.mutationAuditFailureMask)
                              + ", runtime failure mask="
                              + std::to_string(runtimeFailureMask)
                              + ", first projection policy={"
                              + std::to_string(
                                  firstMismatchedProjectionPolicy.startStiffness)
                              + ","
                              + std::to_string(
                                  firstMismatchedProjectionPolicy.linearStiffness)
                              + ","
                              + std::to_string(
                                  firstMismatchedProjectionPolicy.angularStiffness)
                              + "})";
    state.SkipWithError(error.c_str());
    return;
  }

  const std::uint64_t sceneSpecFingerprint = fixture.sceneSpecFingerprint;
  const std::uint64_t solverConfigurationFingerprint
      = paperBreakableWallSolverConfigurationFingerprint(
          *fixture.world, joints);
  std::size_t frame = 0;

  for (auto _ : state) {
    fixture.world->step();
    benchmark::ClobberMemory();
    ++frame;
    if (frame == kTrajectoryFrames) {
      state.PauseTiming();
      fixture = makePaperBreakableWallWorld(solver);
      if (!fixture.effectiveSceneContractPassed || !fixture.mutationAuditPassed
          || fixture.sceneSpecFingerprint != sceneSpecFingerprint) {
        const std::string error = std::string(solverKey)
                                  + " paper breakable-wall scene fingerprint "
                                    "drifted";
        state.SkipWithError(error.c_str());
        return;
      }
      fixture.world->enterSimulationMode();
      const auto resetJoints = fixture.world->getJoints();
      if (paperBreakableWallSolverConfigurationFingerprint(
              *fixture.world, resetJoints)
          != solverConfigurationFingerprint) {
        const std::string error
            = std::string(solverKey)
              + " paper breakable-wall solver configuration fingerprint "
                "drifted";
        state.SkipWithError(error.c_str());
        return;
      }
      frame = 0;
      state.ResumeTiming();
    }
  }
  state.counters["rigid_bodies"]
      = static_cast<double>(fixture.world->getRigidBodyCount());
  state.counters["rigid_body_joints"]
      = static_cast<double>(fixture.world->getJointCount());
  state.counters["breakable_joints"] = static_cast<double>(breakableJointCount);
  state.counters["collision_shapes"] = static_cast<double>(collisionShapeCount);
  state.counters["impacting_balls"] = static_cast<double>(impactingBallCount);
  state.counters["rigid_constraint_iterations"]
      = static_cast<double>(rigidConstraintIterations);
  state.counters["contact_method_sequential_impulse"]
      = defaultContactMethod ? 1.0 : 0.0;
  state.counters["solver_projection_policies_match"]
      = solverProjectionPoliciesMatch ? 1.0 : 0.0;
  state.counters["trajectory_frames"] = static_cast<double>(kTrajectoryFrames);
  state.counters[std::string("public_") + std::string(solverKey) + "_family"]
      = publicFamily ? 1.0 : 0.0;
  state.counters[std::string("resolved_rigid_body_") + std::string(solverKey)]
      = resolvedRigidBody ? 1.0 : 0.0;
  state
      .counters[std::string("resolved_rigid_contact_") + std::string(solverKey)]
      = resolvedRigidContact ? 1.0 : 0.0;
  state.counters
      [std::string("resolved_rigid_pair_constraint_") + std::string(solverKey)]
      = resolvedRigidPairConstraint ? 1.0 : 0.0;
  state.counters["resolved_rigid_constraint_iterations"]
      = resolvedRigidConstraintIterations ? 1.0 : 0.0;
  constexpr auto avbdProfile = vbd::kAvbdRigidPaper2025Profile;
  const bool usesAvbdProfile = solver == sx::RigidBodySolver::Avbd;
  state.counters["rigid_avbd_parameter_profile_paper_2025"]
      = usesAvbdProfile && resolvedAvbdParameterProfile ? 1.0 : 0.0;
  state.counters["rigid_avbd_alpha"]
      = usesAvbdProfile ? avbdProfile.alpha : 0.0;
  state.counters["rigid_avbd_beta"] = usesAvbdProfile ? avbdProfile.beta : 0.0;
  state.counters["rigid_avbd_gamma"]
      = usesAvbdProfile ? avbdProfile.gamma : 0.0;
  state.counters["runtime_contract_passed"] = runtimeContractPassed ? 1.0 : 0.0;
  state.counters["effective_scene_contract_passed"]
      = fixture.effectiveSceneContractPassed ? 1.0 : 0.0;
  state.counters["effective_scene_mutation_audit_passed"]
      = fixture.mutationAuditPassed ? 1.0 : 0.0;
  state.counters["scene_spec_matches_python"]
      = fixture.matchesPythonSceneSpecFingerprint ? 1.0 : 0.0;
  state.counters["runtime_identity_recorded"] = 1.0;
  state.counters["runtime_identity_applicable"] = 1.0;
  state.counters["runtime_identity_not_applicable"] = 0.0;
  state.counters["runtime_identity_public_avbd_rigid"]
      = solver == sx::RigidBodySolver::Avbd ? 1.0 : 0.0;
  state.counters["runtime_identity_variational_multibody"] = 0.0;
  state.counters["runtime_identity_contract_passed"]
      = runtimeContractPassed ? 1.0 : 0.0;
  state.counters["scene_spec_fingerprint_hi"] = static_cast<double>(
      static_cast<std::uint32_t>(sceneSpecFingerprint >> 32u));
  state.counters["scene_spec_fingerprint_lo"]
      = static_cast<double>(static_cast<std::uint32_t>(sceneSpecFingerprint));
  state.counters["solver_configuration_fingerprint_hi"] = static_cast<double>(
      static_cast<std::uint32_t>(solverConfigurationFingerprint >> 32u));
  state.counters["solver_configuration_fingerprint_lo"] = static_cast<double>(
      static_cast<std::uint32_t>(solverConfigurationFingerprint));
}

//==============================================================================
static void BM_AvbdPaperBreakableWallStep(benchmark::State& state)
{
  runPaperBreakableWallStepBenchmark(state, sx::RigidBodySolver::Avbd, "avbd");
}
BENCHMARK(BM_AvbdPaperBreakableWallStep)->Iterations(120);

//==============================================================================
static void BM_VbdPaperBreakableWallStep(benchmark::State& state)
{
  runPaperBreakableWallStepBenchmark(state, sx::RigidBodySolver::Vbd, "vbd");
}
BENCHMARK(BM_VbdPaperBreakableWallStep)->Iterations(120);

//==============================================================================
static void BM_SequentialImpulsePaperBreakableWallStep(benchmark::State& state)
{
  runPaperBreakableWallStepBenchmark(
      state, sx::RigidBodySolver::SequentialImpulse, "sequential-impulse");
}
BENCHMARK(BM_SequentialImpulsePaperBreakableWallStep)->Iterations(120);

//==============================================================================
static void BM_AvbdRigidBreakableJointStep(benchmark::State& state)
{
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidBreakableJointWorld(jointCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["breakable_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdRigidBreakableJointStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidSphericalBreakableJointStep(benchmark::State& state)
{
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidSphericalBreakableJointWorld(jointCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::PublicAvbdRigid)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["breakable_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdRigidSphericalBreakableJointStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedRevoluteMotorStep(benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedRevoluteMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdArticulatedRevoluteMotorStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedBreakableMotorStep(benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedBreakableMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
  state.counters["breakable_motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdArticulatedBreakableMotorStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedPrismaticMotorStep(benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedPrismaticMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdArticulatedPrismaticMotorStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedPrismaticBreakableMotorStep(
    benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedPrismaticBreakableMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
  state.counters["breakable_motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdArticulatedPrismaticBreakableMotorStep)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedWorldPrismaticBreakableMotorStep(
    benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedWorldPrismaticBreakableMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
  state.counters["breakable_motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdArticulatedWorldPrismaticBreakableMotorStep)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedWorldRevoluteBreakableMotorStep(
    benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedWorldRevoluteBreakableMotorWorld(motorCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
  state.counters["breakable_motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdArticulatedWorldRevoluteBreakableMotorStep)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedBreakableJointStep(benchmark::State& state)
{
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedBreakableJointWorld(jointCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["breakable_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdArticulatedBreakableJointStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedWorldSphericalBreakableJointStep(
    benchmark::State& state)
{
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedWorldSphericalBreakableJointWorld(jointCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["breakable_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdArticulatedWorldSphericalBreakableJointStep)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedSphericalPairBreakableJointStep(
    benchmark::State& state)
{
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedSphericalPairBreakableJointWorld(jointCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["breakable_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdArticulatedSphericalPairBreakableJointStep)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32);

//==============================================================================
static void BM_AvbdArticulatedCompliantJointStep(benchmark::State& state)
{
  const auto familyCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedCompliantJointWorld(familyCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["family_instances"] = static_cast<double>(familyCount);
  state.counters["spherical_joints"] = static_cast<double>(familyCount);
  state.counters["revolute_joints"] = static_cast<double>(familyCount);
  state.counters["prismatic_joints"] = static_cast<double>(familyCount);
  state.counters["compliant_joints"] = static_cast<double>(3u * familyCount);
}
BENCHMARK(BM_AvbdArticulatedCompliantJointStep)->Arg(1)->Arg(4)->Arg(16);

//==============================================================================
static void BM_AvbdArticulatedCompliantMotorStep(benchmark::State& state)
{
  const auto familyCount = static_cast<std::size_t>(state.range(0));
  auto world = makeArticulatedCompliantMotorWorld(familyCount);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["family_instances"] = static_cast<double>(familyCount);
  state.counters["revolute_motors"] = static_cast<double>(familyCount);
  state.counters["prismatic_motors"] = static_cast<double>(familyCount);
  state.counters["compliant_motors"] = static_cast<double>(2u * familyCount);
}
BENCHMARK(BM_AvbdArticulatedCompliantMotorStep)->Arg(1)->Arg(4)->Arg(16);

//==============================================================================
static void BM_AvbdArticulatedCompliantBreakableMotorStep(
    benchmark::State& state)
{
  const auto familyCount = static_cast<std::size_t>(state.range(0));
  auto world
      = makeArticulatedCompliantMotorWorld(familyCount, /*breakable=*/true);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state, *world, BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["family_instances"] = static_cast<double>(familyCount);
  state.counters["revolute_motors"] = static_cast<double>(familyCount);
  state.counters["prismatic_motors"] = static_cast<double>(familyCount);
  state.counters["breakable_motors"] = static_cast<double>(2u * familyCount);
}
BENCHMARK(BM_AvbdArticulatedCompliantBreakableMotorStep)
    ->Arg(1)
    ->Arg(4)
    ->Arg(16);

//==============================================================================
static void BM_AvbdArticulatedHighRatioChainStep(benchmark::State& state)
{
  auto fixture = makeArticulatedHighRatioChainWorld();
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state,
          *fixture.world,
          BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }
  double lastResetTime = 0.0;

  for (auto _ : state) {
    if (fixture.world->getTime() - lastResetTime
        >= kArticulatedHighRatioReplaySeconds) {
      state.PauseTiming();
      resetArticulatedHighRatioChain(fixture.joints);
      lastResetTime = fixture.world->getTime();
      state.ResumeTiming();
    }
    fixture.world->step();
    benchmark::ClobberMemory();
  }
  state.counters["links"]
      = static_cast<double>(kArticulatedHighRatioChainLinks);
  state.counters["mass_ratio"]
      = kArticulatedHighRatioTipMass / kArticulatedHighRatioLightMass;
}
BENCHMARK(BM_AvbdArticulatedHighRatioChainStep);

//==============================================================================
static void BM_AvbdPaperScaleHighRatioChainStep(benchmark::State& state)
{
  auto fixture = makeArticulatedHighRatioChainWorld(
      kPaperScaleHighRatioChainLinks,
      kPaperScaleHighRatioTipMass,
      kPaperScaleHighRatioMaxIterations,
      kPaperScaleHighRatioTolerance);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state,
          *fixture.world,
          BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }
  fixture.world->setReplayRecordingEnabled(true);
  std::size_t stepsSinceReset = 0;

  for (auto _ : state) {
    if (stepsSinceReset >= kPaperScaleHighRatioReplaySteps) {
      state.PauseTiming();
      fixture.world->restoreReplayFrame(0);
      stepsSinceReset = 0;
      state.ResumeTiming();
    }
    fixture.world->step();
    ++stepsSinceReset;
    benchmark::ClobberMemory();
  }
  state.counters["links"] = static_cast<double>(kPaperScaleHighRatioChainLinks);
  state.counters["mass_ratio"]
      = kPaperScaleHighRatioTipMass / kArticulatedHighRatioLightMass;
  state.counters["max_iterations"]
      = static_cast<double>(kPaperScaleHighRatioMaxIterations);
  state.counters["tolerance"] = kPaperScaleHighRatioTolerance;
  state.counters["replay_seconds"] = kPaperScaleHighRatioReplaySeconds;
  state.counters["replay_steps"]
      = static_cast<double>(kPaperScaleHighRatioReplaySteps);
  const auto envelope = measureArticulatedHighRatioReplayEnvelope(fixture);
  state.counters["finite_replay"] = envelope.allFinite ? 1.0 : 0.0;
  state.counters["max_abs_position"] = envelope.maxAbsPosition;
}
BENCHMARK(BM_AvbdPaperScaleHighRatioChainStep);

//==============================================================================
static void BM_AvbdPaperScaleHighRatioChainIterationSweep(
    benchmark::State& state)
{
  const auto maxIterations = static_cast<std::size_t>(state.range(0));
  auto fixture = makeArticulatedHighRatioChainWorld(
      kPaperScaleHighRatioChainLinks,
      kPaperScaleHighRatioTipMass,
      maxIterations,
      kPaperScaleHighRatioTolerance);
  if (!enterSimulationModeAndRecordRuntimeIdentity(
          state,
          *fixture.world,
          BenchmarkRuntimeIdentity::VariationalMultibody)) {
    return;
  }
  fixture.world->setReplayRecordingEnabled(true);
  std::size_t stepsSinceReset = 0;

  for (auto _ : state) {
    if (stepsSinceReset >= kPaperScaleHighRatioReplaySteps) {
      state.PauseTiming();
      fixture.world->restoreReplayFrame(0);
      stepsSinceReset = 0;
      state.ResumeTiming();
    }
    fixture.world->step();
    ++stepsSinceReset;
    benchmark::ClobberMemory();
  }
  state.counters["links"] = static_cast<double>(kPaperScaleHighRatioChainLinks);
  state.counters["mass_ratio"]
      = kPaperScaleHighRatioTipMass / kArticulatedHighRatioLightMass;
  state.counters["max_iterations"] = static_cast<double>(maxIterations);
  state.counters["tolerance"] = kPaperScaleHighRatioTolerance;
  state.counters["replay_seconds"] = kPaperScaleHighRatioReplaySeconds;
  state.counters["replay_steps"]
      = static_cast<double>(kPaperScaleHighRatioReplaySteps);
  const auto envelope = measureArticulatedHighRatioReplayEnvelope(fixture);
  state.counters["finite_replay"] = envelope.allFinite ? 1.0 : 0.0;
  state.counters["max_abs_position"] = envelope.maxAbsPosition;
}
BENCHMARK(BM_AvbdPaperScaleHighRatioChainIterationSweep)
    ->Arg(25)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200);

namespace {

std::filesystem::path runningExecutablePath(const char* argv0)
{
  std::error_code error;
#if defined(__linux__)
  auto path = std::filesystem::read_symlink("/proc/self/exe", error);
  if (!error) {
    return std::filesystem::canonical(path);
  }
#endif
  if (argv0 == nullptr || *argv0 == '\0') {
    throw std::runtime_error("benchmark argv[0] is unavailable");
  }
  auto fallbackPath = std::filesystem::canonical(argv0, error);
  if (error) {
    throw std::runtime_error(
        "cannot resolve running benchmark executable: " + error.message());
  }
  return fallbackPath;
}

bool hasCallerInjectedContext(int argc, char** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument.starts_with("--benchmark_context")) {
      return true;
    }
  }
  return false;
}

char** processEnvironment()
{
#if defined(_WIN32)
  return _environ;
#elif defined(__APPLE__)
  return *_NSGetEnviron();
#else
  return environ;
#endif
}

// Google Benchmark reads every one of its flags from the matching uppercased
// BENCHMARK_* environment variable, so BENCHMARK_CONTEXT alone can inject
// arbitrary context entries into the emitted JSON. Refuse rather than trust the
// caller to have sanitized the environment.
std::optional<std::string> injectedBenchmarkEnvironmentVariable()
{
  static constexpr std::string_view benchmarkEnvironmentPrefix = "BENCHMARK_";
  for (char** entry = processEnvironment();
       entry != nullptr && *entry != nullptr;
       ++entry) {
    const std::string_view assignment(*entry);
    const auto separator = assignment.find('=');
    if (separator == std::string_view::npos) {
      continue;
    }
    const std::string_view name = assignment.substr(0, separator);
    if (name.starts_with(benchmarkEnvironmentPrefix)
        && !assignment.substr(separator + 1).empty()) {
      return std::string(name);
    }
  }
  return std::nullopt;
}

} // namespace

int main(int argc, char** argv)
{
  benchmark::MaybeReenterWithoutASLR(argc, argv);
  if (hasCallerInjectedContext(argc, argv)) {
    std::cerr << "Caller-injected benchmark context is forbidden.\n";
    return 1;
  }
  if (const auto injected = injectedBenchmarkEnvironmentVariable()) {
    std::cerr << "Caller-injected benchmark environment is forbidden: "
              << *injected << ".\n";
    return 1;
  }

  try {
    benchmark::AddCustomContext(
        "dart_benchmark_executable_path",
        runningExecutablePath(argv == nullptr ? nullptr : argv[0]).string());
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  benchmark::AddCustomContext(
      "dart_benchmark_source_sha256", DART_FIGURE13_BENCHMARK_SOURCE_SHA256);
  benchmark::AddCustomContext(
      "dart_capture_source_git_head", DART_CAPTURE_SOURCE_GIT_HEAD);
  benchmark::AddCustomContext(
      "dart_capture_source_provenance_digest",
      DART_CAPTURE_SOURCE_PROVENANCE_DIGEST);
  benchmark::AddCustomContext(
      "dart_build_configuration_digest",
      DART_CAPTURE_BUILD_CONFIGURATION_DIGEST);
  benchmark::AddCustomContext(
      "dart_cmake_build_type", DART_FIGURE13_CMAKE_BUILD_TYPE);
  benchmark::AddCustomContext("dart_compiler_id", DART_FIGURE13_COMPILER_ID);
  benchmark::AddCustomContext(
      "dart_compiler_version", DART_FIGURE13_COMPILER_VERSION);
#if defined(NDEBUG)
  benchmark::AddCustomContext("dart_ndebug", "1");
#else
  benchmark::AddCustomContext("dart_ndebug", "0");
#endif
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  benchmark::AddCustomContext("dart_optimization_enabled", "1");
#else
  benchmark::AddCustomContext("dart_optimization_enabled", "0");
#endif

  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
    return 1;
  }
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
