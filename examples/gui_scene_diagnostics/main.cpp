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
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/scene.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstdlib>

namespace {

struct ExampleOptions
{
  dart::gui::RunOptions run;
};

void printUsage(const char* argv0)
{
  std::cout << "Usage: " << argv0 << " [--frames N] [--width N] [--height N]\n";
}

bool parseInt(std::string_view value, int& output)
{
  if (value.empty()) {
    return false;
  }

  const std::string str(value);
  char* end = nullptr;
  const long result = std::strtol(str.c_str(), &end, 10);
  if (!end || *end != '\0') {
    return false;
  }
  if (result < std::numeric_limits<int>::min()
      || result > std::numeric_limits<int>::max()) {
    return false;
  }

  output = static_cast<int>(result);
  return true;
}

bool parseArgs(int argc, char* argv[], ExampleOptions& options)
{
  options.run.maxFrames = 10;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    }

    int value = 0;
    if (arg == "--frames" && i + 1 < argc) {
      if (!parseInt(argv[++i], value) || value <= 0) {
        std::cerr << "Invalid frame count: " << argv[i] << "\n";
        return false;
      }
      options.run.maxFrames = value;
      continue;
    }

    if (arg == "--width" && i + 1 < argc) {
      if (!parseInt(argv[++i], value)) {
        std::cerr << "Invalid width: " << argv[i] << "\n";
        return false;
      }
      options.run.width = value;
      continue;
    }

    if (arg == "--height" && i + 1 < argc) {
      if (!parseInt(argv[++i], value)) {
        std::cerr << "Invalid height: " << argv[i] << "\n";
        return false;
      }
      options.run.height = value;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    printUsage(argv[0]);
    return false;
  }

  dart::gui::normalizeRunOptions(options.run);
  return true;
}

std::shared_ptr<dart::simulation::World> createDiagnosticWorld()
{
  using dart::dynamics::BoxShape;
  using dart::dynamics::CollisionAspect;
  using dart::dynamics::DynamicsAspect;
  using dart::dynamics::Frame;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::SimpleFrame;
  using dart::dynamics::Skeleton;
  using dart::dynamics::VisualAspect;
  using dart::dynamics::WeldJoint;

  auto world = dart::simulation::World::create("gui_scene_diagnostics");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto body = Skeleton::create("diagnostic_box");
  auto [boxJoint, boxBody] = body->createJointAndBodyNodePair<FreeJoint>();
  boxBody->setName("box_body");
  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
  boxTransform.translation() = Eigen::Vector3d(0.0, 0.0, 0.45);
  boxJoint->setTransformFromParentBodyNode(boxTransform);
  auto boxShape = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.3, 0.3));
  auto* boxShapeNode = boxBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape, "box_visual");
  boxShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.55, 0.9));
  world->addSkeleton(body);

  auto ground = Skeleton::create("ground");
  auto groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
  groundBody->setName("ground_body");
  auto* groundShapeNode = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(3.0, 3.0, 0.1)),
      "ground_visual");
  groundShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.7, 0.7, 0.7));
  world->addSkeleton(ground);

  Eigen::Isometry3d frameTransform = Eigen::Isometry3d::Identity();
  frameTransform.translation() = Eigen::Vector3d(-0.65, 0.45, 0.35);
  auto frame = SimpleFrame::createShared(
      Frame::World(), "diagnostic_interactive_frame", frameTransform);
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));
  frame->getVisualAspect(true)->setColor(Eigen::Vector3d(0.95, 0.72, 0.18));
  world->addSimpleFrame(frame);

  return world;
}

std::string describeHit(
    const std::vector<dart::gui::RenderableDescriptor>& renderables,
    const std::optional<dart::gui::PickHit>& hit)
{
  if (!hit) {
    return "none";
  }

  const auto& renderable = renderables[hit->renderableIndex];
  if (renderable.skeletonName.empty()) {
    return renderable.shapeFrameName;
  }
  return renderable.skeletonName + "/" + renderable.bodyName + "/"
         + renderable.shapeNodeName;
}

} // namespace

int main(int argc, char* argv[])
{
  ExampleOptions options;
  if (!parseArgs(argc, argv, options)) {
    return 1;
  }

  auto world = createDiagnosticWorld();

  int renderedFrames = 0;
  while (!dart::gui::shouldStopAfterFrame(options.run, renderedFrames)) {
    world->step();
    ++renderedFrames;
  }

  const auto renderables = dart::gui::extractRenderables(*world);

  dart::gui::DebugDrawOptions debugOptions;
  debugOptions.drawBodyFrames = true;
  debugOptions.drawCentersOfMass = true;
  const auto debugLines = dart::gui::extractDebugLines(*world, debugOptions);

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.45);
  const auto basis = dart::gui::makeOrbitCameraBasis(camera);
  const auto centerRay = dart::gui::makePerspectivePickRay(
      camera,
      options.run.width * 0.5,
      options.run.height * 0.5,
      options.run.width,
      options.run.height);
  const auto hit = dart::gui::pickNearestRenderable(renderables, centerRay);
  std::vector<dart::gui::DebugLineDescriptor> selectionLines;
  if (hit) {
    selectionLines
        = dart::gui::makeSelectionDebugLines(renderables[hit->renderableIndex]);
  }

  std::cout << "frames: " << renderedFrames << "\n"
            << "renderables: " << renderables.size() << "\n"
            << "debug lines: " << debugLines.size() << "\n"
            << "selection lines: " << selectionLines.size() << "\n"
            << "camera eye: " << basis.eye.transpose() << "\n"
            << "center pick: " << describeHit(renderables, hit) << "\n";

  return renderables.empty() ? 1 : 0;
}
