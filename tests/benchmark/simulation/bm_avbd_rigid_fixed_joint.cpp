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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

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

std::unique_ptr<sx::World> makeRigidBreakableJointWorld(std::size_t jointCount)
{
  sx::WorldOptions options;
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
}
BENCHMARK(BM_AvbdRigidEndpointClassification)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointStep(benchmark::State& state)
{
  const auto linkCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidFixedJointWorld(linkCount);
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
static void BM_AvbdRigidBreakableJointStep(benchmark::State& state)
{
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidBreakableJointWorld(jointCount);
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
  world->enterSimulationMode();

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
static void BM_AvbdArticulatedHighRatioChainStep(benchmark::State& state)
{
  auto fixture = makeArticulatedHighRatioChainWorld();
  fixture.world->enterSimulationMode();
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
  fixture.world->enterSimulationMode();
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
  fixture.world->enterSimulationMode();
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

BENCHMARK_MAIN();
