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

#include <dart/gui/application.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

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
      dart::dynamics::Frame::World(), "box_frame_1", tf1);
  f1->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f1, Eigen::Vector4d(0.92, 0.32, 0.24, 1.0));
  addFrame(*world, f1);

  auto f2 = f1->spawnChildSimpleFrame("box_frame_2", tf2);
  f2->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f2, Eigen::Vector4d(0.24, 0.58, 0.92, 1.0));
  addFrame(*world, f2);

  auto f3 = f2->spawnChildSimpleFrame("box_frame_3", tf3);
  f3->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f3, Eigen::Vector4d(0.28, 0.76, 0.34, 1.0));
  addFrame(*world, f3);

  auto markerRoot = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "marker_root");
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
  addMarker("marker_1", f1->getTransform(markerRoot.get()));
  addMarker("marker_2", f2->getTransform(markerRoot.get()));
  addMarker("marker_3", f3->getTransform(markerRoot.get()));

  auto arrow = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "arrow");
  auto arrowShape = std::make_shared<dart::dynamics::LineSegmentShape>(
      Eigen::Vector3d(0.1, -0.1, 0.0), Eigen::Vector3d(0.1, 0.0, 0.0), 2.0f);
  arrowShape->addVertex(Eigen::Vector3d(0.075, -0.025, 0.0), 1);
  arrowShape->addVertex(Eigen::Vector3d(0.125, -0.025, 0.0), 1);
  arrow->setShape(arrowShape);
  setColor(arrow, Eigen::Vector4d(1.0, 0.5, 0.5, 1.0));
  addFrame(*world, arrow);

  return world;
}

dart::gui::RunOptions makeSimpleFramesRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
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

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createSimpleFramesWorld();
  options.runDefaults = makeSimpleFramesRunDefaults();
  options.camera = makeSimpleFramesCamera();
  return dart::gui::runApplication(argc, argv, options);
}
