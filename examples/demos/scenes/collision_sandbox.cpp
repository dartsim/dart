/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

// Static showcase of the collision_sandbox pair-test registry. The original
// standalone collision_sandbox tool was an interactive workbench for
// benchmarking native broad/narrow-phase collision queries; that workbench
// is gone (its UI was tightly coupled to ImGui state), but its catalog of
// canonical shape pairs (pair_registry) is preserved here so the curated
// test pairs remain discoverable from `dart-demos`.

#include "collision_sandbox/pair_registry.hpp"
#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/native/shapes/shape.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart::examples::demos {

namespace {

namespace sandbox = dart::examples::collision_sandbox;
namespace collision = dart::collision::native;

constexpr double kPairColumnSpacing = 3.5;
constexpr double kPairOriginZ = 0.0;
constexpr int kPairColumns = 3;

std::shared_ptr<dart::dynamics::Shape> dynamicsShapeFor(
    collision::ShapeType type, const sandbox::ShapeParameters& params)
{
  switch (type) {
    case collision::ShapeType::Sphere:
      return std::make_shared<dart::dynamics::SphereShape>(params.radius);
    case collision::ShapeType::Box:
      return std::make_shared<dart::dynamics::BoxShape>(
          2.0 * params.halfExtents);
    case collision::ShapeType::Capsule:
      return std::make_shared<dart::dynamics::CapsuleShape>(
          params.radius, params.height);
    case collision::ShapeType::Cylinder:
      return std::make_shared<dart::dynamics::CylinderShape>(
          params.radius, params.height);
    case collision::ShapeType::Plane:
    case collision::ShapeType::Mesh:
    case collision::ShapeType::Convex:
    case collision::ShapeType::Sdf:
    case collision::ShapeType::Compound:
      // Visual placeholder for shapes without a dynamics-shape analogue:
      // a small sphere so the pair still shows up in the catalog.
      return std::make_shared<dart::dynamics::SphereShape>(0.2);
  }
  return std::make_shared<dart::dynamics::SphereShape>(params.radius);
}

void addPairFrame(
    dart::simulation::World& world,
    const std::string& name,
    const Eigen::Isometry3d& origin,
    collision::ShapeType type,
    const sandbox::ShapeParameters& params,
    const Eigen::Vector4d& rgba)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name, origin);
  frame->setShape(dynamicsShapeFor(type, params));
  frame->getVisualAspect(true)->setRGBA(rgba);
  world.addSimpleFrame(frame);
}

void addPairCase(
    dart::simulation::World& world,
    const sandbox::PairCase& pair,
    const Eigen::Vector3d& columnOrigin)
{
  const auto pose = sandbox::defaultPairPose(pair);
  const auto paramsA = sandbox::defaultShapeParameters(pair.shapeA);
  const auto paramsB = sandbox::defaultShapeParameters(pair.shapeB);

  Eigen::Isometry3d originA = pose.transformA;
  originA.pretranslate(columnOrigin);
  Eigen::Isometry3d originB = pose.transformB;
  originB.pretranslate(columnOrigin);

  const Eigen::Vector4d color = pair.supportsContact()
                                    ? Eigen::Vector4d(0.20, 0.55, 0.90, 0.85)
                                    : Eigen::Vector4d(0.90, 0.45, 0.20, 0.85);

  addPairFrame(world, pair.id + "_A", originA, pair.shapeA, paramsA, color);
  addPairFrame(
      world,
      pair.id + "_B",
      originB,
      pair.shapeB,
      paramsB,
      color * 0.85 + Eigen::Vector4d(0.0, 0.0, 0.0, 0.15));
}

dart::dynamics::SimpleFramePtr makeGroundVisual()
{
  auto ground = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "ground_visual");
  ground->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(20.0, 20.0, 0.05)));
  ground->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.78, 0.78, 0.78, 1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -1.5);
  ground->setTransform(tf);
  return ground;
}

dart::gui::Panel makeSandboxPanel(std::size_t numPairs)
{
  dart::gui::Panel panel;
  panel.title = "Collision Sandbox";
  panel.buildWithContext =
      [numPairs](dart::gui::PanelBuilder& builder, dart::gui::PanelContext&) {
        builder.text(
            "Static showcase of " + std::to_string(numPairs)
            + " canonical pair tests");
        builder.text("blue: contact pair; orange: distance-only / unsupported");
        builder.separator();
        builder.text(
            "Interactive broad/narrow-phase benchmarking lives in the "
            "Python collision-native tooling — this scene is the visual "
            "index of the canonical pair registry.");
      };
  return panel;
}

dart::gui::OrbitCamera makeCamera(std::size_t numPairs)
{
  dart::gui::OrbitCamera camera;
  const int columns = std::min(kPairColumns, static_cast<int>(numPairs));
  const int rows = static_cast<int>((numPairs + columns - 1) / columns);
  camera.target = Eigen::Vector3d(
      kPairColumnSpacing * 0.5 * (columns - 1),
      kPairColumnSpacing * 0.5 * (rows - 1),
      kPairOriginZ);
  camera.distance = 3.0 + 1.4 * std::max(columns, rows);
  camera.yaw = 0.6;
  camera.pitch = 0.45;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeCollisionSandboxScene()
{
  auto world = dart::simulation::World::create("collision_sandbox");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSimpleFrame(makeGroundVisual());

  const auto pairs = sandbox::pairCases();
  std::size_t laidOut = 0;
  for (const auto& pair : pairs) {
    const int col = static_cast<int>(laidOut % kPairColumns);
    const int row = static_cast<int>(laidOut / kPairColumns);
    const Eigen::Vector3d origin(
        col * kPairColumnSpacing, row * kPairColumnSpacing, kPairOriginZ);
    addPairCase(*world, pair, origin);
    ++laidOut;
  }

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.simulateWorld = false;
  options.camera = makeCamera(laidOut);
  options.panels.push_back(makeSandboxPanel(laidOut));
  return options;
}

} // namespace dart::examples::demos
