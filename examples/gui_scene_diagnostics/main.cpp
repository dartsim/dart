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

#include <dart/gui/application.hpp>
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
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

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
  auto world = createDiagnosticWorld();

  // Startup inspector pass over the renderer-neutral descriptors.
  const auto renderables = dart::gui::extractRenderables(*world);

  dart::gui::DebugDrawOptions debugOptions;
  debugOptions.drawBodyFrames = true;
  debugOptions.drawCentersOfMass = true;
  const auto debugLines = dart::gui::extractDebugLines(*world, debugOptions);

  // The application owns the CLI (`--frames`, `--width`, `--height`,
  // `--screenshot`, ...);
  // this example only changes the defaults: a short, always-headless run so it
  // stays a console diagnostic and never opens a window.
  dart::gui::RunOptions runDefaults;
  runDefaults.windowTitle = "gui_scene_diagnostics";
  runDefaults.headless = true;
  runDefaults.maxFrames = 10;

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.45);
  const auto basis = dart::gui::makeOrbitCameraBasis(camera);
  const auto centerRay = dart::gui::makePerspectivePickRay(
      camera,
      runDefaults.width * 0.5,
      runDefaults.height * 0.5,
      runDefaults.width,
      runDefaults.height);
  const auto hit = dart::gui::pickNearestRenderable(renderables, centerRay);

  // Custom debug geometry flows through the component seam: the per-frame
  // debugProvider feeds the built-in overlay (unlit, no-shadow, always on
  // top), so the example no longer carries its own overlay wiring. The
  // provider re-picks each frame, so the selection highlight follows the
  // falling box while the world simulates. Re-extracting the whole world per
  // frame is fine for this three-renderable scene; real hosts should derive
  // provider geometry from state they already track (the provider runs every
  // frame on the render thread).
  auto providerFrames = std::make_shared<int>(0);
  auto providerLines = std::make_shared<std::size_t>(0);

  dart::gui::ApplicationOptions appOptions;
  appOptions.world = world;
  appOptions.runDefaults = runDefaults;
  appOptions.camera = camera;
  appOptions.debugProvider = [world, centerRay, providerFrames, providerLines] {
    dart::gui::DebugScene debugScene;
    const auto frameRenderables = dart::gui::extractRenderables(*world);
    const auto frameHit
        = dart::gui::pickNearestRenderable(frameRenderables, centerRay);
    if (frameHit) {
      const auto& renderable = frameRenderables[frameHit->renderableIndex];
      debugScene.lines = dart::gui::makeSelectionDebugLines(renderable);
      dart::gui::DebugLabelDescriptor label;
      label.position = renderable.worldTransform.translation();
      label.text = "center pick";
      debugScene.labels.push_back(label);
    }
    ++(*providerFrames);
    *providerLines = debugScene.lines.size();
    return debugScene;
  };

  const int appResult = dart::gui::runApplication(argc, argv, appOptions);

  std::cout << "renderables: " << renderables.size() << "\n"
            << "debug lines: " << debugLines.size() << "\n"
            << "camera eye: " << basis.eye.transpose() << "\n"
            << "center pick: " << describeHit(renderables, hit) << "\n"
            << "debug provider frames: " << *providerFrames << "\n"
            << "debug provider selection lines: " << *providerLines << "\n";

  if (appResult != 0) {
    return appResult;
  }
  return renderables.empty() ? 1 : 0;
}
