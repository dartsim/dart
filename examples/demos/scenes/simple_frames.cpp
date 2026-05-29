/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/arrow_shape.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart::examples::demos {

namespace {

void setColor(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& frame,
    const Eigen::Vector4d& color)
{
  frame->getVisualAspect(true)->setRGBA(color);
}

std::shared_ptr<dart::dynamics::SimpleFrame> addFrame(
    dart::simulation::World& world,
    const std::shared_ptr<dart::dynamics::SimpleFrame>& frame)
{
  world.addSimpleFrame(frame);
  return frame;
}

dart::simulation::WorldPtr createSimpleFramesWorld()
{
  auto world = dart::simulation::World::create("simple_frames");
  world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translate(Eigen::Vector3d(0.1, -0.1, 0.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translate(Eigen::Vector3d(0.0, 0.1, 0.0));
  tf2.rotate(Eigen::AngleAxisd(0.7853981633974483, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translate(Eigen::Vector3d(0.0, 0.0, 0.1));
  tf3.rotate(Eigen::AngleAxisd(1.0471975511965976, Eigen::Vector3d::UnitY()));

  auto f1 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "F1", tf1);
  f1->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f1, Eigen::Vector4d(0.92, 0.32, 0.24, 1.0));
  addFrame(*world, f1);

  auto f2 = f1->spawnChildSimpleFrame("F2", tf2);
  f2->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f2, Eigen::Vector4d(0.24, 0.58, 0.92, 1.0));
  addFrame(*world, f2);

  auto f3 = f2->spawnChildSimpleFrame("F3", tf3);
  f3->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f3, Eigen::Vector4d(0.28, 0.76, 0.34, 1.0));
  addFrame(*world, f3);

  auto markerRoot = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "A");
  markerRoot->setShape(
      std::make_shared<dart::dynamics::EllipsoidShape>(
          Eigen::Vector3d(0.02, 0.02, 0.02)));
  setColor(markerRoot, Eigen::Vector4d(0.95, 0.75, 0.20, 1.0));
  addFrame(*world, markerRoot);

  const auto addMarker
      = [&](const std::string& name, const Eigen::Isometry3d& transform) {
          auto marker = markerRoot->spawnChildSimpleFrame(name, transform);
          marker->setShape(
              std::make_shared<dart::dynamics::EllipsoidShape>(
                  Eigen::Vector3d(0.01, 0.01, 0.01)));
          setColor(marker, Eigen::Vector4d(0.95, 0.75, 0.20, 1.0));
          addFrame(*world, marker);
        };
  addMarker("A1", f1->getTransform(markerRoot.get()));
  addMarker("A2", f2->getTransform(markerRoot.get()));
  addMarker("A3", f3->getTransform(markerRoot.get()));

  auto arrow = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "arrow");
  auto arrowShape = std::make_shared<dart::dynamics::ArrowShape>(
      Eigen::Vector3d(0.1, -0.1, 0.0),
      Eigen::Vector3d(0.1, 0.0, 0.0),
      dart::dynamics::ArrowShape::Properties(0.002, 1.8),
      Eigen::Vector4d(1.0, 0.5, 0.5, 1.0));
  arrow->setShape(arrowShape);
  setColor(arrow, Eigen::Vector4d(1.0, 0.5, 0.5, 1.0));
  addFrame(*world, arrow);

  return world;
}

dart::gui::OrbitCamera makeSimpleFramesCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.0);
  camera.yaw = 0.4636476090008061;
  camera.pitch = 0.7297276562269663;
  camera.distance = 3.0;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeSimpleFramesScene()
{
  dart::gui::ApplicationOptions options;
  options.world = createSimpleFramesWorld();
  options.camera = makeSimpleFramesCamera();
  return options;
}

} // namespace dart::examples::demos
