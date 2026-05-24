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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/gui/application.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace legacy_sim = dart::simulation;
namespace sx = dart::simulation::experimental;

namespace {

Eigen::Vector4d rgba(double r, double g, double b, double a = 1.0)
{
  return {r, g, b, a};
}

std::shared_ptr<dynamics::Shape> makeVisualShape(
    const sx::CollisionShape& shape)
{
  switch (shape.type) {
    case sx::CollisionShapeType::Sphere:
      return std::make_shared<dynamics::SphereShape>(shape.radius);
    case sx::CollisionShapeType::Box:
      return std::make_shared<dynamics::BoxShape>(2.0 * shape.halfExtents);
  }
  return std::make_shared<dynamics::SphereShape>(0.25);
}

struct RenderBody
{
  sx::RigidBody body;
  dynamics::SimpleFramePtr frame;
};

struct ExampleState
{
  sx::World physicsWorld;
  legacy_sim::WorldPtr renderWorld
      = legacy_sim::World::create("experimental_rigid_body_gui");
  std::vector<RenderBody> bodies;

  void addBody(
      const std::string& name,
      const sx::RigidBodyOptions& options,
      const sx::CollisionShape& shape,
      const Eigen::Vector4d& color)
  {
    auto body = physicsWorld.addRigidBody(name, options);
    body.setCollisionShape(shape);
    body.setRestitution(options.isStatic ? 0.0 : 0.15);
    body.setFriction(0.85);

    auto frame = dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(), name + "_visual", body.getTransform());
    frame->setShape(makeVisualShape(shape));
    frame->getVisualAspect(true)->setRGBA(color);
    renderWorld->addSimpleFrame(frame);
    bodies.push_back({body, std::move(frame)});
  }

  void syncRenderFrames()
  {
    for (auto& body : bodies) {
      body.frame->setTransform(body.body.getTransform());
    }
  }

  void step()
  {
    physicsWorld.step();
    syncRenderFrames();
  }
};

std::shared_ptr<ExampleState> makeExampleState()
{
  auto state = std::make_shared<ExampleState>();

  constexpr double timeStep = 1.0 / 120.0;
  state->physicsWorld.setTimeStep(timeStep);
  state->renderWorld->setTimeStep(timeStep);

  sx::RigidBodyOptions ground;
  ground.position = Eigen::Vector3d(0.0, 0.0, -0.08);
  ground.isStatic = true;
  state->addBody(
      "ground",
      ground,
      sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 3.0, 0.08)),
      rgba(0.42, 0.45, 0.48));

  for (int i = 0; i < 4; ++i) {
    sx::RigidBodyOptions options;
    options.mass = 1.0 + 0.25 * i;
    options.position
        = Eigen::Vector3d(-0.75 + 0.5 * i, 0.18 * (i % 2), 1.2 + 0.45 * i);
    options.linearVelocity = Eigen::Vector3d(0.45 - 0.25 * i, 0.0, 0.0);
    options.angularVelocity = Eigen::Vector3d(0.0, 0.4 + 0.2 * i, 0.25);

    const double radius = 0.18 + 0.03 * i;
    state->addBody(
        "falling_sphere_" + std::to_string(i),
        options,
        sx::CollisionShape::makeSphere(radius),
        rgba(0.16 + 0.12 * i, 0.42, 0.92 - 0.1 * i));
  }

  sx::RigidBodyOptions box;
  box.mass = 2.0;
  box.position = Eigen::Vector3d(0.85, -0.35, 2.2);
  box.orientation
      = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  box.angularVelocity = Eigen::Vector3d(0.2, -0.35, 0.1);
  state->addBody(
      "falling_box",
      box,
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.24, 0.16)),
      rgba(0.93, 0.56, 0.18));

  state->physicsWorld.enterSimulationMode();
  state->syncRenderFrames();
  return state;
}

gui::RunOptions makeRunDefaults()
{
  gui::RunOptions options;
  options.windowTitle = "DART Experimental Rigid Body";
  options.width = 1280;
  options.height = 720;
  return options;
}

gui::OrbitCamera makeCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.65);
  camera.yaw = -0.78;
  camera.pitch = 0.34;
  camera.distance = 5.2;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  const auto state = makeExampleState();

  gui::ApplicationOptions options;
  options.world = state->renderWorld;
  options.runDefaults = makeRunDefaults();
  options.camera = makeCamera();
  options.simulateWorld = false;
  options.preStep = [state]() {
    state->step();
  };

  return gui::runApplication(argc, argv, options);
}
